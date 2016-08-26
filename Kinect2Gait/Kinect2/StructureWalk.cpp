#include "StructureWalk.h"

namespace StructureWalk
{

atomic_int upGaitCMD{GAIT_CMD::NOCMD};

auto UpWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    UpGaitParam param;

    for (auto &i : params)
    {
        if (i.first == "init")
        {
            param.d = PassStepDitch::adjustWrapper.terrainAnalysisResult.terrain.terrainData[0];
            param.h = PassStepDitch::adjustWrapper.terrainAnalysisResult.terrain.terrainData[1];
            cout<<"Step Distance: "<< param.d <<endl;
            cout<<"Step Height: "<< param.h <<endl;
        }
        else if (i.first == "distance")
        {
            param.d = stod(i.second);
        }
        else if (i.first == "height")
        {
            param.h = stod(i.second);
        }
        else if (i.first == "go")
        {
            upGaitCMD = GO;
        }
        else
        {
            cout<<"Parse Failed!"<<endl;
        }
    }
    msg.copyStruct(param);
}

auto UpWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & param_in)->int
{
    static int period = 1;
    static int localCount = 0;
    auto &robot = static_cast<Robots::RobotBase &>(model);
    auto &param = static_cast<const UpGaitParam &>(param_in);

    static aris::dynamic::FloatMarker beginMak{ robot.ground() };

    int timeNow = param.count;

    switch (upGaitCMD)
    {
    case GAIT_CMD::NOCMD:
        break;
    case GAIT_CMD::GO:
        if(period == 1)
        {
            float bodymovedata[3]{0};
            bodymovedata[1] = param.h;
            int remainCount =  RobotBody(robot, localCount, bodymovedata);
            localCount++;
            if(remainCount == 0)
            {
                period++;
                localCount = 0;
            }
        }
        if(period == 2)
        {
            float stepheight = param.h;
            int remainCount =  RobotStepUp(robot, localCount, stepheight);
            localCount++;
            if(remainCount == 0)
            {
                period = 1;
                localCount = 0;
                upGaitCMD = STOP;
            }
        }
        break;
    case GAIT_CMD::STOP:
        return 0;
        break;
    default:
        break;
    }

    return 1;

}
}
