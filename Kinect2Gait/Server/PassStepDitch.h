#ifndef PASSSTEPDITCH_H
#define PASSSTEPDITCH_H

#include <iostream>

#include <thread>
#include <aris.h>
#include <rtdk.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>
#include "VisionSensor.h"
#include "Vision_Terrain.h"
#include "Vision_Gait.h"

using namespace std;
using namespace aris::core;

namespace PassStepDitch
{
class PassStepDitchWrapper
{
public:
    static aris::control::Pipe<int> passStepDitchPipe;
    static std::thread passStepDitchThread;

    static TerrainAnalysis terrainAnalysisResult;

    static void AdjustStart();

    static auto PassStepDitchParse(const string &cmd, const map<string, string> &param, aris::core::Msg &msg) -> void;
    static auto StopPassStepDitchParse(const string &cmd, const map<string, string> &param, aris::core::Msg &msg) -> void;
    static auto PassStepDitchGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in) -> int;

private:
    static atomic_bool isTerrainAnalysisFinished;
    static atomic_bool isSending;
    static atomic_bool isStop;
};
extern PassStepDitchWrapper adjustWrapper;
}


#endif // PASSSTEPDITCH_H
