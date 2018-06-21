#include "pkbehaviors.h"
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
#include "naobehavior.h"
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
/////// GOALIE
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

PKGoalieBehavior::
PKGoalieBehavior( const std::string teamName,
                  int uNum,
                  const map<string, string>& namedParams_,
                  const string& rsg_)
    : NaoBehavior( teamName,
                   uNum,
                   namedParams_,
                   rsg_) {
}

void PKGoalieBehavior::
beam( double& beamX, double& beamY, double& beamAngle ) {
    beamX = -HALF_FIELD_X+.5;
    beamY = 0;
    beamAngle = 0;
}




SkillType PKGoalieBehavior::
selectSkill() {
     double distance,angle;
    getTargetDistanceAndAngle(ball,distance,angle);
   
    double a = ball.getDistanceTo(myGoal);
    
    
    
  if(ball.getDistanceTo(myGoal)<1.9||ball.getY()<2&&ball.getX()>-2&&ball.getX()<-13)
  {
      // TODO: need to accomplish it
    return SKILL_STAND;
  }
  else if(!ballInOpponent()&&ball.getDistanceTo(myGoal)<1.9)
  {
    return kickBall(KICK_FORWARD,VecPosition(15,10,0));
  }
  else if(me.getX()>ball.getX())
  {
    
    VecPosition targetPos1=collisionAvoidance(true,false,true,1,1,VecPosition(ball.getX()-0.2,ball.getY(),0));
    return goToTarget(targetPos1);
   
  }
  else
  {
    return goToTarget(VecPosition((-15 + ((ball.getX()+15)/a)),(ball.getY()/a),0));
  }
  /* if(ball.getDistanceTo(myGoal)<5)
   {
      if(ball.getY()>0&me.getY()<ball.getY())
     {
       return	kickBall(KICK_IK, VecPosition(10,10,0));
       
    }else if(ball.getY()<0&&me.getY()<ball.getY())
    {
      return kickBall(KICK_IK, VecPosition(10,-10, 0));
      
    }else
    {
     
		if((ball.getX()+15)>(10-abs(ball.getY())))
		{
		    if(ball.getY()<0)
		    {
		      return kickBall(KICK_IK,VecPosition(ball.getX(),-10,0));
		    }
		    else
		    {
		      return kickBall(KICK_IK,VecPosition(ball.getX(),10,0));
		    }
		}
		else
		{
		    return kickBall(KICK_IK,VecPosition(15,ball.getY(),0));   
		}
	 
    }
   }
   else
   {
      return Golie_Walk_Defend();
   }
   */
   
  
}





////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
/////// SHOOTER
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
PKShooterBehavior::
PKShooterBehavior( const std::string teamName,
                   int uNum,
                   const map<string, string>& namedParams_,
                   const string& rsg_ )
    : NaoBehavior( teamName,
                   uNum,
                   namedParams_,
                   rsg_ ) {
}

void PKShooterBehavior::
beam( double& beamX, double& beamY, double& beamAngle ) {
    beamX = -0.5;
    beamY = 0;
    beamAngle = 0;
}

SkillType PKShooterBehavior::
selectSkill() {
  if(ball.getX()<3)
      return kickBall(KICK_DRIBBLE,VecPosition(15,5,0));
  else if(ball.getX()>5)
    return kickBall(KICK_FORWARD,oppGoal);
  if(ball.getDistanceTo(oppGoal)<5)
  {
    return kickBall(KICK_DRIBBLE,oppGoal);
  }
  else 
    return kickBall(KICK_DRIBBLE,VecPosition(6,3,0));
   /* else if(ball.getY()>=-5&&ball.getY()<=5)
      return kickBall(KICK_FORWARD,oppGoal);
    else if(ball.getY()>5&&ball.getX()<13)
      return kickBall(KICK_DRIBBLE,VecPosition(ball.getX(),4,0));
    else if(ball.getY()<-5&&ball.getX()<13)
      return kickBall(KICK_DRIBBLE,VecPosition(ball.getX(),-4,0));
    else
      return kickBall(KICK_DRIBBLE,oppGoal);*/
   // return kickBall(KICK_DRIBBLE,oppGoal); 
}
