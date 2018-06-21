#include "naobehavior.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <cctype>
#include <exception>

#include "../skills/skillparser.h"
#include "../rvdraw/rvdraw.h"
#include <assert.h>

// For UT Walk
#include <common/InterfaceInfo.h>
#include <motion/MotionModule.h>

extern int agentBodyType;


/*
 * namedParams_ are a mapping between parameters and their values
 */
NaoBehavior::
NaoBehavior(const std::string teamName, int uNum, const map<string, string>& namedParams_, const string& rsg_) :
    namedParams( namedParams_ ),
    rsg( rsg_ )
{

    //cout << "Constructing of Nao Behavior" << endl;

    srand ((unsigned)time(NULL) );
    srand48((unsigned)time(NULL));

    classname = "NaoBehavior"; //TODO: eliminate it...

    mInit = false;
    initBeamed = false;

    agentTeamName = teamName;
    agentUNum = uNum;

    scoreMe = 0;
    scoreOpp = 0;

    worldModel = new WorldModel();
    bodyModel = new BodyModel(worldModel);

    memory_ = new Memory(false,true);

    memory_->getOrAddBlockByName(frame_info_,"frame_info");
    memory_->getOrAddBlockByName(vision_frame_info_,"vision_frame_info");
    frame_info_->source = MEMORY_SIM; // set to simulaor
    vision_frame_info_->source = MEMORY_SIM;

    memory_->getOrAddBlockByName(raw_sensors_,"raw_sensors");
    memory_->getOrAddBlockByName(raw_joint_angles_,"raw_joint_angles");
    memory_->getOrAddBlockByName(processed_joint_angles_,"processed_joint_angles");
    memory_->getOrAddBlockByName(raw_joint_commands_,"raw_joint_commands");
    memory_->getOrAddBlockByName(processed_joint_commands_,"processed_joint_commands");
    memory_->getOrAddBlockByName(sim_effectors_,"sim_effectors");

    core = new MotionCore(CORE_SIM, true, *memory_);
    fParsedVision = false;
    particleFilter = new PFLocalization( worldModel, bodyModel, core);

    parser = new Parser(worldModel, bodyModel, teamName, particleFilter,
                        vision_frame_info_,
                        frame_info_,
                        raw_joint_angles_,
                        raw_sensors_ );



    initBeamed = false;
    initialized = false;
    beamTime = -1;
    hoverTime = 2.25;

    fallState = 0;
    fallenLeft = false;
    fallenRight = false;
    fallenDown = false;
    fallenUp = false;
    fallTimeStamp = -1;
    fallTimeWait = -1;

    lastGetupRecoveryTime = -1.0;

    monMsg = "";

    // TODO: Treat paths more correctly? (system independent way)
    try {
        readSkillsFromFile( "./skills/stand.skl" );
      /* if(agentBodyType == 1) {
        	readSkillsFromFile( "./skills/kick_t1.skl" );
	}
	else {*/
        	readSkillsFromFile( "./skills/kick.skl" );
	//}

        // ik skills
        readSkillsFromFile( "./skills/kick_ik_0.skl" );
        // end ik skills

    }
    catch( std::string& what ) {
        cerr << "Exception caught: " << what << endl;
        exit(1);
    }
    catch (std::exception& e)
    {
        cerr << e.what() << endl;
        exit(1);
    }

    // initialize just so reset Skill doesnt segfault
    skill = SKILL_STAND;
    resetSkills();
    resetKickState();

    // Uncomment this to use ground truth data for localization
    //worldModel->setUseGroundTruthDataForLocalization(true);
}

NaoBehavior::~NaoBehavior() {

    delete parser;
    delete worldModel;
    delete bodyModel;
    delete particleFilter;
    delete core;
}

string NaoBehavior::Init() {
    cout << "Loading rsg: " << "(scene " << rsg << ")" << endl;
    return "(scene " + rsg + ")";
}





string NaoBehavior::Think(const std::string& message) {

    //  cout << "(NaoBehavior) received message " << message << endl;

    fParsedVision = false;
    bool parseSuccess = parser->parse(message, fParsedVision);
    if(!parseSuccess && (worldModel->getPlayMode() != PM_BEFORE_KICK_OFF)) {
//    cout << "****************************************\n";
//    cout << "Could not parse message: " << message << "\n";
//    cout << "****************************************\n";
    }

    //  cout << "\nparseSuccess: " << parseSuccess << "\n";
    //  worldModel->display();
    bodyModel->refresh();
    if(fParsedVision) {
        if (!worldModel->isFallen()) {
            parser->processVision();
        } else {
            parser->processSightings(true /*fIgnoreVision*/);
        }
    }
    this->updateFitness();
    //  bodyModel->display();
    //  bodyModel->displayDerived();

    // Example usage of the roboviz drawing system and RVSender in rvdraw.cc.
    // Draw agent positions and orientations
    /*
    worldModel->getRVSender()->clearStaticDrawings();
    VecPosition pos = worldModel->getMyPosition();
    VecPosition dir = VecPosition(1,0,0);
    dir = dir.rotateAboutZ(-worldModel->getMyAngDeg());
    worldModel->getRVSender()->drawPoint(pos.getX(), pos.getY(), 10);
    worldModel->getRVSender()->drawLine(pos.getX(), pos.getY(), pos.getX()+dir.getX(), pos.getY()+dir.getY());
    */

    calculateAngles();


    if (frame_info_->start_time == -1) {
        frame_info_->start_time = frame_info_->seconds_since_start;
        vision_frame_info_->start_time = frame_info_->start_time;
    }
    frame_info_->seconds_since_start= frame_info_->seconds_since_start - frame_info_->start_time;

    raw_joint_angles_->values_[RHipYawPitch] = raw_joint_angles_->values_[LHipYawPitch];

    preProcessJoints();  // Apply the correct sign to the joint angles

    postProcessJoints(); // Flip the joint angles back

    string action;

    if (!mInit) {

        mInit = true;
        stringstream ss;
        ss << "(init (unum " << agentUNum << ")(teamname " << agentTeamName << "))";
        action = ss.str();
        return action;
    }

    if (worldModel->getLastPlayMode() != worldModel->getPlayMode() &&
            (worldModel->getPlayMode() == PM_BEFORE_KICK_OFF ||
             worldModel->getPlayMode() == PM_GOAL_LEFT ||
             worldModel->getPlayMode() == PM_GOAL_RIGHT)) {
        initBeamed = false;
    }

    // Record game score
    if (worldModel->getScoreLeft() != -1 && worldModel->getScoreRight() != -1) {
        scoreMe = worldModel->getSide() == SIDE_LEFT ? worldModel->getScoreLeft() : worldModel->getScoreRight();
        scoreOpp = worldModel->getSide() == SIDE_LEFT ? worldModel->getScoreRight() : worldModel->getScoreLeft();
    }


    if ((worldModel->getPlayMode() == PM_GOAL_LEFT || worldModel->getPlayMode() == PM_GOAL_RIGHT || worldModel->getPlayMode() == PM_BEFORE_KICK_OFF) && worldModel->getLastPlayMode() != worldModel->getPlayMode()) {
        beamTime = worldModel->getTime() + hoverTime;
    }

    else if(beamTime >= 0 && worldModel->getTime() >= beamTime) {
        //initialized = false;
        initBeamed = false;
        beamTime = -1.0;
    }


    if (worldModel->getPlayMode() != worldModel->getLastPlayMode()) {
        worldModel->setLastDifferentPlayMode(worldModel->getLastPlayMode());
    }
    worldModel->setLastPlayMode(worldModel->getPlayMode());

    if(!initialized) {
        if(!worldModel->getUNumSet() || !worldModel->getSideSet()) {
            //      cout << "UNum and side not received yet.\n";
            action = "";
            return action;
        }

        if(!initBeamed) {
            initBeamed = true;


            double beamX, beamY, beamAngle;

            // Call a virtual function
            // It could either be implemented here (real game)
            // or in the inherited classes
            // Parameters are being filled in the beam function.
            this->beam( beamX, beamY, beamAngle );
            stringstream ss;
            ss << "(beam " << beamX << " " << beamY << " " << beamAngle << ")";
            particleFilter->setForBeam(beamX, beamY, beamAngle);
            action = ss.str();
            return action;
        }
        else {
            // Not Initialized
            bodyModel->setInitialHead();
            bodyModel->setInitialArm(ARM_LEFT);
            bodyModel->setInitialArm(ARM_RIGHT);
            bodyModel->setInitialLeg(LEG_LEFT);
            bodyModel->setInitialLeg(LEG_RIGHT);
            initialized = true;
        }
    }

    if(!initBeamed) {
        initBeamed = true;

        double beamX, beamY, beamAngle;

        // Call a virtual function
        // It could either be implemented here (real game)
        // or in the inherited classes - for optimization agent.
        // Parameters are being filled in the beam function.
        this->beam( beamX, beamY, beamAngle );
        stringstream ss;
        ss << "(beam " << beamX << " " << beamY << " " << beamAngle << ")";
        particleFilter->setForBeam(beamX, beamY, beamAngle);
        action = ss.str();
    }

    frame_info_->frame_id++;
    act();

    worldModel->getRVSender()->refresh();

    action = action + composeAction();

    //std::cout << "Sending action: " << action << "\n";
    return action;
}

void NaoBehavior::act() {
    refresh();//更新XYZ我的位置、球的位置

    const double LAST_LINE_SIGHTING_THRESH = 0.1;
    if (worldModel->getTime()-worldModel->getLastLineSightingTime() > LAST_LINE_SIGHTING_THRESH) {
        worldModel->setLocalized(false);
    }


    // If the ball gets too far away, reset kick state
    if(me.getDistanceTo(ball) > 1) {
        resetKickState();//这里的reset指让现在的踢、踢的种类都变成NONE
    }

    //worldModel->getRVSender()->drawPoint("me", me.getX(), me.getY(), 20);
    int pm = worldModel->getPlayMode();
    bool resetForKickoff = pm == PM_BEFORE_KICK_OFF || pm == PM_GOAL_LEFT || pm == PM_GOAL_RIGHT;//当playmode 为这些状态时就认为对开球的复位完成



    if(checkingFall()) {     //如果正在采取一些措施防止摔倒或正在从摔倒状态恢复，则返回真
        resetSkills();
        bodyModel->setUseOmniWalk(false);
        return;
    }
    else if(resetForKickoff) {
        if (beamablePlayMode() && (worldModel->isFallen() || worldModel->getTime() <= beamTime)) {
            initBeamed = false;
        }
        resetSkills();
        skill = SKILL_STAND;
        core->move(0,0,0);
        velocity.paramSet = WalkRequestBlock::PARAMS_DEFAULT;
    }
    else {
        if(skills[skill]->done( bodyModel, worldModel) ||
                bodyModel->useOmniWalk()) {
            skills[skill]->reset();
            resetScales();
            SkillType currentSkill = selectSkill();


            if (currentSkill != SKILL_WALK_OMNI) {
                velocity.paramSet = WalkRequestBlock::PARAMS_DEFAULT;
            }

            bodyModel->setUseOmniWalk(true);
            switch(currentSkill) {
            case SKILL_WALK_OMNI:
                core->move(velocity.paramSet, velocity.x, velocity.y, velocity.rot);
                break;
            case SKILL_STAND:
                core->move(0,0,0);
                break;
            default:
                bodyModel->setUseOmniWalk(false);
            }

            if (bodyModel->useOmniWalk()) {
                resetSkills();
            } else {

                /*EnumParser<SkillType> enumParser;
                cout << "Skill: " << enumParser.getStringFromEnum(skill) << endl;*/

                // Transitions, coding a finite state machine...

                SkillType lastSkill = worldModel->getLastSkill();
                skill = currentSkill;


            }
        }
    }
    //  cout << "Executing: " << EnumParser<SkillType>::getStringFromEnum(skill) << endl;
    //  cerr << "Selected skill: " << SkillType2Str[skill] << " time: " << worldModel->getTime() << endl;
    //    LOG_ST(skill);
    skills[skill]->execute( bodyModel, worldModel );



    worldModel->setLastSkill(skill);
    // to be used by odometry
    if (bodyModel->useOmniWalk()) {
        worldModel->addExecutedSkill(SKILL_WALK_OMNI);
    } else {
        worldModel->addExecutedSkill( skill );
    }




    //Set the head turn behavior
    VecPosition me = worldModel->getMyPosition();
    me.setZ(0);
    ball = worldModel->getBall();
    ball.setZ(0);
    // Currently, every 2 seconds
    static double panOffset = drand48() * 4.0;
    int panState = ( static_cast<int>( worldModel->getTime()+panOffset ) ) % 4;
    double ballDistance, ballAngle;

    getTargetDistanceAndAngle(ball, ballDistance, ballAngle);
    //SkillType lastSkill = worldModel->getLastSkill();

    if (worldModel->isFallen()) {
        bodyModel->setScale(EFF_H1, 0.5);
        bodyModel->setTargetAngle(EFF_H1, 0);
    } else if (ballDistance < 1.0 && worldModel->getWorldObject(WO_BALL)->validPosition) {
        // close to the ball, focusing on the ball and turning head 30 degrees
        if( panState == 0 || panState == 2 ) {
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, ballAngle);
        } else {
            int direction = (panState == 1) ? 1 : -1;
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, ballAngle+(direction*30.0));
        }
    } else {
        // default behavior
        if( panState == 0 || panState == 2 ) {
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, 0);
        } else {
            int direction = (panState == 1) ? 1 : -1;
            bodyModel->setScale(EFF_H1, 0.3);
            bodyModel->setTargetAngle(EFF_H1, direction*120);// 30.0); // 120.0);
        }
    }
}


/*
 * Throws string
 */
void NaoBehavior::readSkillsFromFile( const std::string& filename) {
//  cerr << "Loading skills from file " << filename << endl;


    // Load a skill file to memory. Assuming a file is < 4K

    int buffsize = 65536;
    char buff[buffsize];
    int numRead;

    fstream skillFile( filename.c_str(), ios_base::in );
    skillFile.read( buff, buffsize );
    if( !skillFile.eof() ) {
        throw "failed to read the whole skill file " + filename;
    }
    numRead = skillFile.gcount();

    // padding with \0 at the end
    buff[numRead] = '\0';



    // Preprocessing: replace parameters by values.

    string skillDescription("");
    skillDescription.reserve( buffsize );
    for( int i = 0; i < numRead; ++i ) {
        char c = buff[i];
        if( c == '$' ) {
            // parameter - replace it

            string param("");
            i += 1;
            while( i < numRead && ( isalnum( buff[i] ) || buff[i] == '_' ) ) {
                param += buff[i];
                ++i;
            }

            map<string, string>::const_iterator it = namedParams.find( param );
            if( it == namedParams.end() ) {
                throw "Missing parameter in skill file " + filename + ": " + param;
            }
            skillDescription += it->second;

            if( i < numRead )
                skillDescription += buff[i];

        } else {
            // not a param, just concatenate c
            skillDescription += c;

        }
    }




    // Parse

    SkillParser parser( skills, bodyModel );
    parse_info<iterator_t> info = parse( skillDescription.c_str(),
                                         parser,
                                         ( space_p | comment_p("#") )
                                       );



    // check results
    if (info.hit)
    {
//    cout << "-------------------------\n";
//    cout << "Parsing succeeded\n";
//    cout << "-------------------------\n";
//    cout << "stop "  << info.stop << endl;
//    cout << "full " << info.full << endl;
//    cout << "length " << info.length << endl;
    }
    else
    {
        cout << "-------------------------\n";
        cout << "Parsing failed\n";
        //            cout << "stopped at: \": " << info.stop << "\"\n";
        cout << "-------------------------\n";
//    throw "Parsing failed";
    }

}


bool NaoBehavior::isRightSkill( SkillType skill ) {
    string skillStr = EnumParser<SkillType>::getStringFromEnum( skill );
    return skillStr.find("RIGHT") != string::npos;
}

bool NaoBehavior::isLeftSkill( SkillType skill ) {
    string skillStr = EnumParser<SkillType>::getStringFromEnum( skill );
    return skillStr.find("LEFT") != string::npos;
}


double NaoBehavior::
trim(const double& value, const double& min, const double&max)
{
    double ret;
    if (value > max)
        ret = max;
    else if (value < min)
        ret = min;
    else
        ret = value;

    return ret;
}

void NaoBehavior::calculateAngles() {

    float  accX = raw_sensors_->values_[accelX];
    float  accY = raw_sensors_->values_[accelY];
    float  accZ = raw_sensors_->values_[accelZ];

    raw_sensors_->values_[angleX] = atan2(accY,accZ);
    raw_sensors_->values_[angleY] = -atan2(accX,accZ);

    //raw_sensors_->values_[gyroX] = 0; // = 1000000.0;
    //raw_sensors_->values_[gyroY] = 0; //= 1000000.0;
}

void NaoBehavior::preProcessJoints() {
    for (int i=0; i<NUM_JOINTS; i++) {
        processed_joint_angles_->values_[i] = spark_joint_signs[i] * raw_joint_angles_->values_[i];
    }
}

void NaoBehavior::postProcessJoints() {
    raw_joint_commands_->angle_time_ = processed_joint_commands_->angle_time_;
    raw_joint_commands_->stiffness_time_ = processed_joint_commands_->stiffness_time_;
    for (int i=0; i<NUM_JOINTS; i++) {
        raw_joint_commands_->angles_[i] = spark_joint_signs[i] * processed_joint_commands_->angles_[i]; // apply joint signs to convert to the robot's reference frame
        raw_joint_commands_->stiffness_[i] = processed_joint_commands_->stiffness_[i];
    }
    raw_joint_commands_->send_stiffness_ = processed_joint_commands_->send_stiffness_;
    processed_joint_commands_->send_stiffness_ = false;
}

void NaoBehavior::resetSkills() {
    skills[skill]->reset();

    skill = SKILL_STAND;
    skillState = 0;

    kickDirection = VecPosition(1.0, 0, 0);

    resetScales();

    kickType = KICK_FORWARD;

    skills[worldModel->getLastSkill()]->reset();
    worldModel->setLastSkill(skill);

    bodyModel->setUseOmniWalk(true);
}

void NaoBehavior::resetScales() {
    for (int e = int(EFF_H1); e < int(EFF_NUM); e++) {
        bodyModel->setScale(e, 1.0);
    }
}


// Determines whether a collision will occur while moving to a target, adjusting accordingly when necessary
VecPosition NaoBehavior::collisionAvoidance(bool avoidTeammate, bool avoidOpponent, bool avoidBall, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, bool fKeepDistance) {
    // Obstacle avoidance
    VecPosition closestObjPos = VecPosition(100, 100, 0);
    double closestObjDistance = me.getDistanceTo(closestObjPos);

    // Avoid the ball if flag is set
    if(avoidBall) {
        if (abs(me.getAngleBetweenPoints(target, ball)) < 90.0
                && (fKeepDistance || me.getDistanceTo(ball) <= me.getDistanceTo(target))) {
            closestObjPos = ball;
            closestObjDistance = me.getDistanceTo(ball);
        }
    }

    // Avoid all of your teamates if flag is set
    if(avoidTeammate) {
        for(int i = WO_TEAMMATE1; i <= WO_TEAMMATE11; ++i) {
            // Skip ourself
            if (worldModel->getUNum() == i - WO_TEAMMATE1 + 1) {
                continue;
            }
            WorldObject* teammate = worldModel->getWorldObject( i );
            if (teammate->validPosition == true) {
                VecPosition temp = teammate->pos;
                temp.setZ(0);
                if (abs(me.getAngleBetweenPoints(target, temp)) < 90.0) {
                    if (!fKeepDistance && me.getDistanceTo(temp) > me.getDistanceTo(target)) {
                        continue;
                    }
                    double distance = me.getDistanceTo(temp);
                    if (distance < closestObjDistance) {
                        closestObjDistance = distance;
                        closestObjPos = temp;
                    }
                }
            }
        }
    }

    // Avoid opponents if flag is set
    if(avoidOpponent) {
        if (closestObjDistance > PROXIMITY_THRESH) {
            for(int i = WO_OPPONENT1; i <= WO_OPPONENT11; ++i) {
                WorldObject* opponent = worldModel->getWorldObject( i );
                if (opponent->validPosition == true) {
                    VecPosition temp = opponent->pos;
                    temp.setZ(0);
                    if (abs(me.getAngleBetweenPoints(target, temp)) < 90.0 &&
                            me.getDistanceTo(temp) < me.getDistanceTo(target)) {
                        double distance = me.getDistanceTo(temp);
                        if (distance < closestObjDistance) {
                            closestObjDistance = distance;
                            closestObjPos = temp;
                        }
                    }
                }
            }
        }
    }

    // Determine where you need to move to avoid the closest object you want to avoid
    if (closestObjDistance <= PROXIMITY_THRESH) {
        VecPosition originalTarget = target;
        target = collisionAvoidanceCorrection(me, PROXIMITY_THRESH, COLLISION_THRESH, target, closestObjPos);
    }

    return target;
}

VecPosition NaoBehavior::collisionAvoidanceCorrection(VecPosition start, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle) {
    double obstacleDist = start.getDistanceTo(obstacle);

    if (abs(start.getAngleBetweenPoints(target, obstacle)) >= 90.0 ||
            obstacleDist > PROXIMITY_THRESH) {
        return target;
    }


    VecPosition obstacleDir = (obstacle-start).normalize();

    VecPosition left90 = start + VecPosition(0, 0, 1).crossProduct(obstacleDir)*1.0;
    VecPosition right90 = start - VecPosition(0, 0, 1).crossProduct(obstacleDir)*1.0;
    if (target.getDistanceTo(left90) > target.getDistanceTo(right90)) {
        target = right90;
    } else {
        target = left90;
    }

    if (obstacleDist <= COLLISION_THRESH) {
        // We're way too close so also back away
        target += (start-obstacle).normalize()*1.0;
    }
    return target;
}

VecPosition NaoBehavior::collisionAvoidanceApproach(double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle) {
    return collisionAvoidanceApproach(me, PROXIMITY_THRESH, COLLISION_THRESH, target, obstacle);
}

VecPosition NaoBehavior::collisionAvoidanceApproach(VecPosition start, double PROXIMITY_THRESH, double COLLISION_THRESH, VecPosition target, VecPosition obstacle) {
  //避免碰撞的方法
    double distanceToObstacle = start.getDistanceTo(obstacle);               //obstacle障碍物
    if (fabs(start.getAngleBetweenPoints(target, obstacle)) >= 90.0 ||
            distanceToObstacle > start.getDistanceTo(target)) {                    //如果start到target和obstacle的夹角大于90°，就直接返回target，因为不会被影响。
        return target;
    }

    if (distanceToObstacle <= PROXIMITY_THRESH) {//小于接近阀值？？
        return collisionAvoidanceCorrection(start, PROXIMITY_THRESH, COLLISION_THRESH, target, obstacle);   //避免碰撞的修正修改
    }

    VecPosition start2Target = target-start;
    VecPosition start2TargetDir = VecPosition(start2Target).normalize();
    VecPosition start2Obstacle = obstacle-start;
    VecPosition start2ObstacleDir = VecPosition(start2Obstacle).normalize();


    VecPosition closestPathPoint = start+
                                   (start2TargetDir*(start2Obstacle.dotProduct(start2TargetDir)));   //最近的路径点

    double pathDistanceFromObstacle = (obstacle-closestPathPoint).getMagnitude();  //障碍物路径距离
    VecPosition originalTarget = target;
    if (pathDistanceFromObstacle < PROXIMITY_THRESH) {//小于接近阀值？？
        target = obstacle + (closestPathPoint-obstacle).normalize()*PROXIMITY_THRESH;
    }
    return target;

}




SkillType NaoBehavior::getWalk(const double& direction, const double& rotation, double speed, bool fAllowOver180Turn)
{
    return getWalk(WalkRequestBlock::PARAMS_DEFAULT, direction, rotation, speed, fAllowOver180Turn);
}

SkillType NaoBehavior::getWalk(WalkRequestBlock::ParamSet paramSet, const double& direction, double rotation, double speed, bool fAllowOver180Turn)
{
    double reqDirection, relSpeed;

    if (worldModel->getTime()-lastGetupRecoveryTime < 1.0 && abs(direction) > 90) {
        // Don't try and walk backwards if we just got up as we are probably unstable c，所以不要尝试倒着走
        speed = 0;
    }

    // Convert direction angle to the range [0, 360)     0°是相对X的负半轴
    reqDirection = fmod(direction, 360.0);
    reqDirection += (reqDirection < 0) ? 360 : 0;
    assert((reqDirection >= 0) && (reqDirection <= 360));

    // Trim the relative speed
    relSpeed = trim(speed, 0, 1);

    double tanReqDirection, tanMaxSpeed;
    double maxSpeedX, maxSpeedY, maxRot;

    // Desired velocity and rotation as a percentage of the maximum speed.
    double relSpeedX, relSpeedY, relRot;

    // Get the maximum speed.
    maxSpeedX = core->motion_->getMaxXSpeed(); //core->walkEngine.p.speedMax.translation.x;
    maxSpeedY = core->motion_->getMaxYSpeed(); // core->walkEngine.p.speedMax.translation.y;

    relRot = rotation;
    // There is no reason to request a turn > 180 or < -180 as in that case
    // we should just turn the other way instead
    if (!fAllowOver180Turn) {
        if (relRot > 180) {
            relRot -= 360.0;
        } else if (relRot < -180) {
            relRot += 360.0;
        }
    }

    relRot = rotation / 180;//转换成弧度

    // Truncate to (+/-)1.0
    relRot = trim(relRot, -1, 1);

    // Calculate tangent. Due to floating point error and the special way the walk
    // engine treats walk requests with only one non-zero component, it is necessary
    // to explicitly set values for direction requests that are multiples of 90.
    //计算切线。由于浮点错误和特殊的方式，步行引擎将步行请求只有一个非零组件，它是必要的明确设置的方向请求的值为90的倍数。
    if ((reqDirection == 0) || (reqDirection == 180))
        tanReqDirection = 0;
    else if ((reqDirection == 90) || (reqDirection == 270))
        tanReqDirection = INFINITY;
    else
        tanReqDirection = abs(tanDeg(reqDirection));

    tanMaxSpeed = maxSpeedY / maxSpeedX;

    // Determine the maximum relative speeds that will result in
    // a walk in the appropriate direction.
    //确定最大相对速度，使在适当的方向。
    if (tanReqDirection < tanMaxSpeed)
    {
        relSpeedX = 1;
        relSpeedY = tanReqDirection / tanMaxSpeed;
    }
    else
    {
        relSpeedX = tanMaxSpeed / tanReqDirection;
        relSpeedY = 1;
    } 

    // Get signs correct. Forward is positive X. Left is positive Y.获取正确的符号，前进是+X，左是+Y
    if (reqDirection > 180)//从这里可以看出0°是相对X的负半轴
        relSpeedY *= -1;

    if ((reqDirection > 90) && (reqDirection < 270))
        relSpeedX *= -1;

    // Abrubt stops or changes in direction can be unstable with the approach      //突然的停止或者改变可能与走到球那不稳定
    // ball walk parameter set so check for this and stabilize if need be//检查这个参数是否需要重新设置。。。。具体意思不清楚？？？
    static WalkRequestBlock::ParamSet lastWalkParamSet = WalkRequestBlock::PARAMS_DEFAULT;
    static bool fLastWalkParamRequestWasApproach = false;
    static double lastWalkParamRequestApproachTime = 999999999;
    bool fStabilize = false;//是否稳
    if (paramSet == WalkRequestBlock::PARAMS_APPROACH_BALL) {
        if (!fLastWalkParamRequestWasApproach) {
            lastWalkParamRequestApproachTime = worldModel->getTime();
        }
        fLastWalkParamRequestWasApproach = true;

        if (lastWalkParamSet != WalkRequestBlock::PARAMS_APPROACH_BALL && (speed < .5 || abs(direction) > 45)) {
            if (worldModel->getTime()-lastWalkParamRequestApproachTime < .5) {
                paramSet = WalkRequestBlock::PARAMS_DEFAULT;
                fStabilize = true;
                relSpeed, relRot = 0;
            }
        }
    } else {
        fLastWalkParamRequestWasApproach = false;
    }

    if (lastWalkParamSet != paramSet) {
        lastWalkParamSet = paramSet;
    }


    // Sanity checks. The absolute value of these variables must be <= 1.
    // However, because of floating point error, it's possible that they are
    // slightly greater than one.
    //检查。这些变量的绝对值必须是< = 1。
    assert(abs(relSpeedX) < 1.001);
    assert(abs(relSpeedY) < 1.001);
    assert(abs(relRot) < 1.001);


    // Record the desired velocity and return the SKILL_WALK_OMNI.
    // NaoBehavior::act() will use the speed components in velocity
    // generate a request to the omnidirectional walk engine whenever the
    // SKILL_WALK_OMNI is invoked.
    //记录所需的速度和返回的skill_walk_omni。NaoBehavior：：act()将速度用速度分量产生请求的全向行走引擎每当skill_walk_omni调用。
    velocity = WalkVelocity(paramSet, relSpeed * relSpeedX, relSpeed * relSpeedY, relRot);

    if (fStabilize) {
        // Stabilize
        return SKILL_STAND;
    }

    return SKILL_WALK_OMNI;
}

// Currently untuned. For example, there's no slow down...
SkillType NaoBehavior::goToTargetRelative(const VecPosition& targetLoc, const double& targetRot, const double speed, bool fAllowOver180Turn, WalkRequestBlock::ParamSet paramSet)
{
    double walkDirection, walkRotation, walkSpeed;

    walkDirection = targetLoc.getTheta();//行走方向
    walkRotation = targetRot;//行走转向
    walkSpeed = speed;//行走速度

    walkSpeed = trim(walkSpeed, 0.1, 1);//使行走速度在0.1~1之间

    if (targetLoc.getMagnitude() == 0)//相对目标向量的模，具体作用未知？？？？
        walkSpeed = 0;

    return getWalk(paramSet, walkDirection, walkRotation, walkSpeed, fAllowOver180Turn);
}


//Assumes target = z-0. Maybe needs further tuning
SkillType NaoBehavior::goToTarget(const VecPosition &target) {
    double distance, angle;
    getTargetDistanceAndAngle(target, distance, angle);

    const double distanceThreshold = 1;//阀值，零界值
    const double angleThreshold = getLimitingAngleForward() * .9;
    VecPosition relativeTarget = VecPosition(distance, angle, 0, POLAR);
    //首先转向目标，尽可能的使用最大速度。
    // Turn to the angle we want to walk in first, since we want to walk with
    // maximum forwards speeds if possible.
    /*if (abs(angle) > angleThreshold)
    {
    return goToTargetRelative(VecPosition(), angle);
    }*/

    // [patmac] Speed/quickness adjustment
    // For now just go full speed in the direction of the target and also turn
    // toward our heading.
    SIM::AngDeg turnAngle = angle;

    // If we are within distanceThreshold of the target, we walk directly to the target
    if (me.getDistanceTo(target) < distanceThreshold) {
        turnAngle = 0;
    }

    // Walk in the direction that we want.
    return goToTargetRelative(relativeTarget, turnAngle);
}

double NaoBehavior::getLimitingAngleForward() {
    double maxSpeedX = core->motion_->getMaxXSpeed(); //core->walkEngine.p.speedMax.translation.x;
    double maxSpeedY = core->motion_->getMaxYSpeed(); // core->walkEngine.p.speedMax.translation.y;
    return abs(atan2Deg(maxSpeedY, maxSpeedX));
}


void NaoBehavior::refresh() {
    //(1, 0, 0) is the front of my direction, use l2g to transform local vecpos to global vecpos
    myXDirection = worldModel->l2g(VecPosition(1.0, 0, 0)) - worldModel->l2g(VecPosition(0, 0, 0));
    myXDirection.setZ(0);
    myXDirection.normalize();

    myYDirection = worldModel->l2g(VecPosition(0, 1.0, 0)) - worldModel->l2g(VecPosition(0, 0, 0));
    myYDirection.setZ(0);
    myYDirection.normalize();

    //Anomalous
    myZDirection = worldModel->l2g(VecPosition(0, 0, 1.0)) - worldModel->l2g(VecPosition(0, 0, 0));
    myZDirection.normalize();

    me = worldModel->getMyPosition(); // ->l2g(VecPosition(0, 0, 0)); // <- had consistency problems
    me.setZ(0);
    
    myGoal =VecPosition(-15, 0, 0);
    oppGoal =VecPosition(15, 0, 0);

    ball = worldModel->getBall();
    ball.setZ(0);
}



//Assumes target it z-0.//获取目标角度函数
void NaoBehavior::getTargetDistanceAndAngle(const VecPosition &target, double &distance, double &angle) {
    VecPosition targetDirection = VecPosition(target) - me;
    targetDirection.setZ(0);

    // distance
    distance = targetDirection.getMagnitude();

    // angle
    targetDirection.normalize();

    angle = VecPosition(0, 0, 0).getAngleBetweenPoints(myXDirection, targetDirection);
    if (isnan(angle)) {//这里是检查is not a number
        //cout << "BAD angle!\n";
        angle = 0;
    }
    if(myYDirection.dotProduct(targetDirection) < 0) {
        angle = -angle;
    }
}


bool NaoBehavior::beamablePlayMode() {
    int pm = worldModel->getPlayMode();
    return pm == PM_BEFORE_KICK_OFF || pm == PM_GOAL_LEFT || pm == PM_GOAL_RIGHT;
}


/* Set message to be send to the monitor port将要发送的消息发送到监视端口 */
void NaoBehavior::setMonMessage(const std::string& msg) {
    monMsg = msg;
}

/* Get message to be sent to the monitor port.  Also flushes message */
string NaoBehavior::getMonMessage() {
    string ret = monMsg;
    monMsg = "";
    return ret;
}



int NaoBehavior::nearest2BallTM() {
    double distance = 10000;
    int nearestP2ball = 1;
    for (int i = WO_TEAMMATE1;i < WO_TEAMMATE1 + 11;i++)
    {
        WorldObject* teammate = worldModel->getWorldObject(i);
        if(!teammate->validPosition)
            continue;
        VecPosition temp = teammate->pos;
        double tDistance = temp.getDistanceTo(ball);
        if(tDistance < distance)
        {
            distance = tDistance;
            nearestP2ball = i;
        }
    }
    return nearestP2ball;
}

int NaoBehavior::nearest2BallOP() {
    double distance = 10000;
    int nearestOP2ball = WO_OPPONENT1;
    for (int i = WO_OPPONENT1;i < WO_OPPONENT1 + 11;i++)
    {
        WorldObject* opponent = worldModel->getWorldObject(i);
        if(!opponent->validPosition)
            continue;
        VecPosition temp = opponent->pos;
        double tDistance = temp.getDistanceTo(ball);
        if(tDistance < distance)
        {
            distance = tDistance;
            nearestOP2ball = i;
        }
    }
    return nearestOP2ball;
}

int NaoBehavior::furthestTM2Ball()
{
    double distance = 0;
    int farthestTM = 1;
    for (int i = WO_TEAMMATE1;i < WO_TEAMMATE1 + 11;i++)
    {
        WorldObject* teammate = worldModel->getWorldObject(i);
        VecPosition temp = teammate->pos;
        double tDistance = temp.getDistanceTo(ball);
        if(tDistance > distance)
        {
            distance = tDistance;
            farthestTM = i;
        }
    }
    return farthestTM;
}

vector<VecPosition> NaoBehavior::getDangerOP(vector<int>& numbers){
    vector<VecPosition> pos;
    for(int i = WO_OPPONENT1;i < WO_OPPONENT1+11;i++)
    {
        WorldObject* op = worldModel->getWorldObject(i);
        if(!op->validPosition)
            continue;
        VecPosition opos = op->pos;
        if(opos.getDistanceTo(ball)<5) {
            pos.push_back(opos);
            numbers.push_back(i);
        }
    }
    return pos;
}

vector<VecPosition> NaoBehavior::getTeammate(vector<int>& teamNum)
{
    vector<VecPosition> pos;
    for(int i = WO_TEAMMATE1;i < WO_TEAMMATE1+11;i++)
    {
        WorldObject* teammate = worldModel->getWorldObject(i);
        if(!teammate->validPosition)
            continue;
        VecPosition teamPosition = teammate->pos;
        if(teamPosition.getDistanceTo(ball)<5) {
            pos.push_back(teamPosition);
            teamNum.push_back(i);
        }
    }
    return pos;
}
bool NaoBehavior::ballInOpponent()
{
    int nearestTM = nearest2BallTM(),
            nearestOP = nearest2BallOP();
    VecPosition teamPosition = worldModel->getWorldObject(nearestTM)->pos,
            opponentPosition = worldModel->getWorldObject(nearestOP)->pos;
    if(teamPosition.getDistanceTo(ball)>opponentPosition.getDistanceTo(ball))
        return true;
    return false;
}
double NaoBehavior::OP2MeDistance()
{
    double distance = 100000;
    for(int i =WO_OPPONENT1;i<WO_OPPONENT1+11;i++)
    {
        if(!worldModel->getWorldObject(i)->validPosition)
            continue;
        double tempDistance = worldModel->getWorldObject(i)->pos.getDistanceTo(me);
        if(tempDistance<distance)
        {
            distance = tempDistance;
        }
    }
    return distance;
}
SkillType  NaoBehavior::testPlayMode()
{
    if(worldModel->getUNum()==1)
        return kickBall(KICK_FORWARD, oppGoal);
    else if(worldModel->getUNum()==2) {
        VecPosition target = worldModel->getWorldObject(1)->pos;
        return goToTarget(target);
    }
    return SKILL_STAND;
}
VecPosition NaoBehavior::goalKickCorrect(VecPosition myGoal)
{
    for(int i=WO_OPPONENT1;i<WO_OPPONENT1+11;i++)
    {
        if(!worldModel->getWorldObject(i)->validPosition)
            continue;
        VecPosition op = worldModel->getWorldObject(i)->pos;
        if(op.getX()<me.getX()||op.getX()>myGoal.getX())
            continue;
        // TODO:判断角度，包括正负，之后需要修正目标位置，根据距离以及周围的单位修正
        if(op.getDistanceTo(me)<10)
        if(me.getAngleBetweenPoints(op,myGoal)<15)
        {
            VecPosition v1 = (op-me).normalize(),
                    v2 = (myGoal-me).normalize();
            if(v1.getX()>v2.getX())
            {
                return VecPosition(0,2,0)+myGoal;
            }
            else
                return VecPosition(0,-2,0)+myGoal;
        }
    }
    return myGoal;
}
SkillType NaoBehavior::playOn()
{
    vector<int> teamNum, opponentNum;
    vector<VecPosition> team,opponent;
    // not use yet
    team = getTeammate(teamNum);
    opponent = getDangerOP(opponentNum);
    double myDistance2Ball = worldModel->getMyPosition().getDistanceTo(ball);
    if(ballInOpponent())
    {
        if(myDistance2Ball<3)
            return kickBall(KICK_IK,oppGoal);
        if(ball.getX()>2)
        {
            if(myDistance2Ball>12)
            {
                // TODO:此处需要调用设定位置的函数
                return goToTarget(VecPosition(ball.getX()-10,ball.getY(),0));
            }
            if(myDistance2Ball>5){
                VecPosition target = VecPosition(ball.getX()-3, ball.getY(),0);
                return goToTarget(target);
            }
            else{
                // TODO:
                if(OP2MeDistance()>5)
                    return kickBall(KICK_FORWARD,oppGoal);
                else
                    return kickBall(KICK_DRIBBLE,oppGoal);
            }
        }
        else if(ball.getX()>-5)
        {

        }
        else    // 球离敌方球门很近时
        {
            return kickBall(KICK_DRIBBLE, VecPosition(0,2,0));
        }
    }
    else    // 球在我方控制下
    {
        if(ball.getX()>10)
        {

        }
        else if(ball.getX()>0)
        {

        }
        else
        {

        }
    }

    return SKILL_STAND;
}


SkillType NaoBehavior::kickOffRight()
{
    return SKILL_STAND;
}
SkillType NaoBehavior::kickOffLeft()
{
    return SKILL_STAND;
}
SkillType NaoBehavior::goalKickLeft()
{
    return SKILL_STAND;
}
SkillType NaoBehavior::goalKickRight()
{
    return SKILL_STAND;
}
SkillType NaoBehavior::kickInLeft()
{
    return SKILL_STAND;
}
SkillType NaoBehavior::kickInRight()
{
    return SKILL_STAND;
}
SkillType NaoBehavior::cornerKickRight()
{
    return SKILL_STAND;
}
SkillType NaoBehavior::cornerKickLeft()
{
    return SKILL_STAND;
}