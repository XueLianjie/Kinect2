#include "PassStepDitch.h"

namespace PassStepDitch
{

PassStepDitchWrapper adjustWrapper;

VISION_WALK_PARAM visionWalkParam;

aris::control::Pipe<int> PassStepDitchWrapper::passStepDitchPipe(true);
std::thread PassStepDitchWrapper::passStepDitchThread;

TerrainAnalysis PassStepDitchWrapper::terrainAnalysisResult;

atomic_bool PassStepDitchWrapper::isTerrainAnalysisFinished(false);
atomic_bool PassStepDitchWrapper::isSending(false);
atomic_bool PassStepDitchWrapper::isStop(false);

void PassStepDitchWrapper::AdjustStart()
{
    passStepDitchThread = std::thread([]()
    {
        while(true)
        {
            int a;
            passStepDitchPipe.recvInNrt(a);

            kinect2.InitMap();
            terrainAnalysisResult.TerrainAnalyze(kinect2.visData.gridMap);

            if(terrainAnalysisResult.terrain.terrainType != FlatTerrain)
            {
                //Adjust x y z theta
                double paramAdjust[4] = {0, 0, 0, 0};
                bool adjustFinished = false;
                terrainAnalysisResult.visionAdjust(paramAdjust, &adjustFinished);

                if(adjustFinished == false)
                {
                    //let robot move
                    if(paramAdjust[3] != 0)
                    {
                        visionWalkParam.movetype = turn;
                        visionWalkParam.turndata = paramAdjust[3] * 2;
                        visionWalkParam.totalCount = 6000/2;
                        cout<<"terrain turn"<<endl;
                    }
                    else
                    {
                        visionWalkParam.movetype = flatmove;
                        paramAdjust[2] *= 2;
                        memcpy(visionWalkParam.movedata, paramAdjust, 3 * sizeof(double));
                        visionWalkParam.totalCount = 5000/2;
                        cout<<"terrain move"<<endl;
                    }
                }
                else
                {
                    visionWalkParam.movetype = stopmove;
                    cout<<terrainAnalysisResult.terrain.terrainData[0]<<" "<<terrainAnalysisResult.terrain.terrainData[1]
                                                                     <<" "<<terrainAnalysisResult.terrain.terrainData[2]<<endl;
                }
            }
            else
            {
                cout<<"FLAT TERRAIN MOVE"<<endl;
                cout<<"MOVE FORWARD: "<<0.325<<endl;
                double move_data[3] = {0, 0, 0.325};
                move_data[2] *= 2;

                visionWalkParam.movetype = flatmove;
                visionWalkParam.totalCount = 5000/2;
                memcpy(visionWalkParam.movedata,move_data,sizeof(move_data));
            }
            isTerrainAnalysisFinished = true;
            cout<<"terrrain ended"<<endl;
        }
    });
}

auto PassStepDitchWrapper::PassStepDitchParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    aris::server::GaitParamBase param;
    msg_out.copyStruct(param);
}

auto PassStepDitchWrapper::PassStepDitchGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int
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
            passStepDitchPipe.sendToNrt(6);
            isSending = true;
            return -1;
        }
    }
}

auto PassStepDitchWrapper::StopPassStepDitchParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    isStop = true;
}

}
