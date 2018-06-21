#include "gazebobehavior.h"

GazeboBehavior::
GazeboBehavior( const std::string teamName,
                int uNum,
                const map<string, string>& namedParams_,
                const string& rsg_)
    : NaoBehavior( teamName,
                   uNum,
                   namedParams_,
                   rsg_) {
}

void GazeboBehavior::
beam( double& beamX, double& beamY, double& beamAngle ) {
    beamX = -HALF_FIELD_X+.5;
    beamY = 0;
    beamAngle = 0;
}

SkillType GazeboBehavior::
selectSkill() {

  if(ball.getDistanceTo(VecPosition(-15,0,0))>2)
    return goToTarget(VecPosition(1,ball.getY()/(15-abs(ball.getX())),0));
  else
  {
    if(ball.getY()>=0)
      return kickBall(KICK_DRIBBLE,VecPosition(ball.getX(),5,0));
    else
      return kickBall(KICK_DRIBBLE,VecPosition(ball.getX(),-5,0));
  }
  
    // Walk to the ball
    //return goToTarget(ball);

    //return SKILL_STAND;
}
