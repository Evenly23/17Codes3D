#include "naobehavior.h"
#include "../rvdraw/rvdraw.h"

extern int agentBodyType;

/*
 * Real game beaming.
 * Filling params x y angle
 */
void NaoBehavior::beam( double& beamX, double& beamY, double& beamAngle ) {
    int beamPosition[11][2] = {
            {-2,0},{-7,0},{-12,-1},
            {-13,0},{-13,1},{-13,-1},
            {-15,0},{-15,2},{-15,-2},
            {-10,4}, {-10,-4}
    };
    int num = worldModel->getUNum()-1;
    beamX = beamPosition[num][0];
    beamY = beamPosition[num][1];
    beamAngle = 0;
}



SkillType NaoBehavior::selectSkill() {
    int side = 0;
    side = worldModel->getSide();

    //return testPlayMode();

    if (side == 0) {

        switch (worldModel->getPlayMode()) {
            case PM_PLAY_ON:  //ok
                playOn();
                break;
            case PM_KICK_OFF_LEFT:  //ok
                kickOffLeft();
                break;
            case PM_KICK_OFF_RIGHT:  //ok
                kickOffRight();
                break;
            case PM_GOAL_KICK_LEFT:
                goalKickLeft();
                break;
            case PM_GOAL_KICK_RIGHT:  //ok
                goalKickRight();
                break;
            case PM_KICK_IN_LEFT:  //ok
                kickInLeft();
                break;
            case PM_KICK_IN_RIGHT:
                kickInRight();
                break;
            case PM_CORNER_KICK_LEFT:
                cornerKickLeft();
                break;
            case PM_CORNER_KICK_RIGHT:
                cornerKickRight();
                break;
		 case PM_FREE_KICK_LEFT:
            return kickInRight();

        case PM_FREE_KICK_RIGHT:
            return kickInLeft();
            default:
                playOn();  //ok
                break;
        }
    } else {
        switch (worldModel->getPlayMode()) {
            case PM_PLAY_ON:  //ok
                playOn();
                break;
            case PM_KICK_OFF_RIGHT:  //ok
                kickOffLeft();
                break;
            case PM_KICK_OFF_LEFT:  //ok
                kickOffRight();
                break;
            case PM_GOAL_KICK_RIGHT:
                goalKickLeft();
                break;
            case PM_GOAL_KICK_LEFT:  //ok
                goalKickRight();
                break;
            case PM_KICK_IN_RIGHT:  //ok
                kickInLeft();
                break;
            case PM_KICK_IN_LEFT:
                kickInRight();
                break;
            case PM_CORNER_KICK_RIGHT:
                cornerKickLeft();
                break;
            case PM_CORNER_KICK_LEFT:
                cornerKickRight();
                break;
		 case PM_FREE_KICK_LEFT:
            return kickInRight();

        case PM_FREE_KICK_RIGHT:
            return kickInLeft();
            default:
                playOn();  //ok
                break;

        }
    }
    return playOn();
}
