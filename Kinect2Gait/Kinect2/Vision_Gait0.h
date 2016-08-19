#ifndef VISION_GAIT0_H
#define VISION_GAIT0_H

#include <Robot_Gait.h>


#define N_CALIBRATION 70
using namespace aris::dynamic;

enum robotMove
{
    nomove = 0,
    turn = 1,
    flatmove = 2,
    bodymove = 3,
    stepup = 4,
    stepdown = 5,
};

struct VISION_WALK_PARAM
{
    int count = 0;
    robotMove movetype = nomove;
    int totalCount = 2000;
    double turndata = 0;
    double movedata[3] = {0, 0, 0};
    double bodymovedata[3] = {0, 0, 0};
    double stepupdata[6] = {0, 0, 0, 0, 0, 0};
    double stepdowndata[6] = {0, 0, 0, 0, 0, 0};
};

struct VISION_CALIBRATION_PARAM final:public aris::server::GaitParamBase
{
    //int count=0;
    int gaitLength=2000;// from zero to the targeting posture
    //int totalCount=N_CALIBRATION*3000;
    int localCount=0;
    int postureNum=N_CALIBRATION;
    int postureCount=0;
    //double targetPosture[6]={0,0,0,0,0,0};
    double postureLib[6*N_CALIBRATION];
};

int RobotVisionWalk(Robots::RobotBase &robot, const VISION_WALK_PARAM &param);

void RobotBody(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam);

void RobotStepUp(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam);

void RobotStepDown(Robots::RobotBase &robot, const VISION_WALK_PARAM &pParam);

#endif // VISION_GAIT0_H
