#include "VisionWalk.h"

namespace VisionWalk
{

VisionWalkWrapper visionWalkWrapper;

VISION_WALK_PARAM visionWalkParam;

aris::control::Pipe<int> VisionWalkWrapper::visionPipe(true);
std::thread VisionWalkWrapper::visionThread ;

TerrainAnalysis VisionWalkWrapper::terrainAnalysisResult;

TerrainType0 VisionWalkWrapper::terrain0 = terrainNotKnown;

atomic_bool VisionWalkWrapper::isTerrainAnalysisFinished(false);
atomic_bool VisionWalkWrapper::isSending(false);
atomic_bool VisionWalkWrapper::isStop(false);

void VisionWalkWrapper::VisionStart()
{
    visionThread = std::thread([]()
    {
        while(true)
        {
            int a;
            visionPipe.recvInNrt(a);

            kinect2.InitMap();
            terrainAnalysisResult.TerrainAnalyze(kinect2.visData.gridMap);

            if(terrain0 == terrainNotKnown)
            {
                if(terrainAnalysisResult.Terrain != FlatTerrain)
                {
                    /*Adjust x y z theta*/
                    double paramAdjust[4] = {0, 0, 0, 0};

                    bool adjustFinished = false;
                    visionAdjust(paramAdjust, &adjustFinished);

                    if(adjustFinished == false)
                    {
                        /*let robot move*/
                        if(paramAdjust[3] != 0)
                        {
                            visionWalkParam.movetype = turn;
                            visionWalkParam.turndata = paramAdjust[3];
                            visionWalkParam.totalCount = 6000/2;
                            cout<<"terrain turn"<<endl;
                        }
                        else
                        {
                            visionWalkParam.movetype = flatmove;
                            memcpy(visionWalkParam.movedata,paramAdjust,3*sizeof(double));
                            visionWalkParam.totalCount = 5000/2;
                            cout<<"terrain move"<<endl;
                        }
                    }
                    else
                    {
                        switch (terrainAnalysisResult.Terrain)
                        {
                        case StepUpTerrain:
                        {
                            cout<<"Move Body Up "<<endl;
                            /*the robot move body*/
                            double movebody[3] = {0, 0.2, 0};
                            visionWalkParam.movetype = bodymove;
                            memcpy(visionWalkParam.bodymovedata, movebody,3*sizeof(double));
                            visionWalkParam.totalCount = 2500;
                            terrain0 = terrainStepUp;
                        }
                            break;
                        case StepDownTerrain:
                        {
                            terrain0 = terrainStepDown;
                            double nextfootpos[7] = {0, 0, 0, 0, 0, 0, 0};
                            visionStepDown(nextfootpos);
                            visionWalkParam.movetype = stepdown;
                            visionWalkParam.totalCount = 18000;
                            memcpy(visionWalkParam.stepdowndata,nextfootpos,sizeof(nextfootpos));
                        }
                            break;
                        case DitchTerrain:
                        {
                            terrain0 = terrainStepOver;
                            double stepoverdata[4] = {0, 0, 0, 0};
                            visionStepOver(stepoverdata);
                            visionWalkParam.movetype = flatmove;
                            visionWalkParam.totalCount = 5000/2;
                            memcpy(visionWalkParam.movedata,stepoverdata + 1, 3*sizeof(double));
                        }
                            break;
                        default:
                            break;
                        }
                    }
                }
                else
                {
                    cout<<"FLAT TERRAIN MOVE"<<endl;
                    cout<<"MOVE FORWARD: "<<0.325<<endl;
                    double move_data[3] = {0, 0, 0.325};

                    visionWalkParam.movetype = flatmove;
                    visionWalkParam.totalCount = 5000/2;
                    memcpy(visionWalkParam.movedata,move_data,sizeof(move_data));
                }
            }
            else
            {
                switch (terrain0)
                {
                case terrainStepUp:
                {
                    double nextfootpos[7] = {0, 0, 0, 0, 0, 0, 0};
                    visionStepUp(nextfootpos);

                    visionWalkParam.movetype = stepup;
                    visionWalkParam.totalCount = 18000;
                    memcpy(visionWalkParam.stepupdata, nextfootpos,sizeof(nextfootpos));

                    if(int(nextfootpos[6]) == 4)
                    {
                        terrain0 = terrainNotKnown;
                    }
                }
                    break;
                case terrainStepDown:
                {
                    double nextfootpos[7] = {0, 0, 0, 0, 0, 0, 0};
                    visionStepDown(nextfootpos);

                    if(int(nextfootpos[6]) == 5)
                    {
                        cout<<"Move Body Down "<<endl;
                        double movebody[3] = {0, -0.2, 0};
                        visionWalkParam.movetype = bodymove;
                        visionWalkParam.totalCount = 2500;
                        memcpy(visionWalkParam.bodymovedata, movebody, sizeof(movebody));
                        terrain0 = terrainNotKnown;
                    }
                    else
                    {
                        visionWalkParam.movetype = stepdown;
                        visionWalkParam.totalCount = 18000;
                        memcpy(visionWalkParam.stepdowndata,nextfootpos,sizeof(nextfootpos));
                    }
                }
                    break;
                case terrainStepOver:
                {
                    double stepoverdata[4] = {0, 0, 0, 0};
                    visionStepOver(stepoverdata);
                    visionWalkParam.movetype = flatmove;
                    visionWalkParam.totalCount = 5000/2;
                    memcpy(visionWalkParam.movedata,stepoverdata + 1, 3*sizeof(double));

                    if(int(stepoverdata[0]) == 4)
                    {
                        terrain0 = terrainNotKnown;
                    }
                }
                    break;
                default:
                    break;
                }
            }
            isTerrainAnalysisFinished = true;
            cout<<"terrrainended"<<endl;
        }
    });
}

auto VisionWalkWrapper::ParsevisionWalk(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    aris::server::GaitParamBase param;
    msg_out.copyStruct(param);
}

auto VisionWalkWrapper::visionWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int
{
    static bool isFirstTime = true;

    if (isTerrainAnalysisFinished)
    {
        if(isFirstTime)
        {
            visionWalkParam.count = 0;
            isFirstTime = false;
        }

        auto &robot = static_cast<Robots::RobotBase &>(model);

        switch(visionWalkParam.movetype)
        {
        case turn:
        {

            int remainCount = RobotVisionWalk(robot, visionWalkParam);
            visionWalkParam.count++;

            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;
        case flatmove:
        {
            int remainCount = RobotVisionWalk(robot, visionWalkParam);
            visionWalkParam.count++;

            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;
        case bodymove:
        {
            RobotBody(robot, visionWalkParam);
            int remainCount = visionWalkParam.totalCount - visionWalkParam.count - 1;
            visionWalkParam.count++;
            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;
        case stepup:
        {
            RobotStepUp(robot, visionWalkParam);
            int remainCount = visionWalkParam.totalCount - visionWalkParam.count - 1;
            visionWalkParam.count++;
            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;
        case stepdown:
        {
            RobotStepDown(robot, visionWalkParam);
            int remainCount = visionWalkParam.totalCount - visionWalkParam.count - 1;
            visionWalkParam.count++;
            if(remainCount == 0 && isStop == true)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return 0;
            }
            if(remainCount == 0 && isStop == false)
            {
                isStop = false;
                isFirstTime = true;
                isSending = false;
                isTerrainAnalysisFinished = false;
                return -1;
            }
        }
            break;
        case stopmove:
        {
            isStop = false;
            isFirstTime = true;
            isSending = false;
            isTerrainAnalysisFinished = false;
            return 0;
        }
            break;
        }
    }
    else
    {
        if(isSending)
        {
            return -1;
        }
        else
        {
            visionPipe.sendToNrt(6);
            isSending = true;
            return -1;
        }
    }
}

auto VisionWalkWrapper::StopvisionWalk(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    isStop = true;
}

}

