#ifndef VISIONWALK_H
#define VISIONWALK_H

#include <iostream>

#include <thread>
#include <aris.h>
#include <rtdk.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>
#include "VisionSensor.h"
#include "Vision_Terrain0.h"
#include "Vision_Gait0.h"
#include "Vision_Control0.h"

namespace VisionWalk
{

enum TerrainType0
{
    terrainNotKnown = 0,
    terrainStepUp = 1,
    terrainStepDown = 2,
    terrainStepOver = 3,
};

class VisionWalkWrapper
{
public:
    static aris::control::Pipe<int> visionPipe;
    static std::thread visionThread ;

    static TerrainAnalysis terrainAnalysisResult;

    static TerrainType0 terrain0;

    static void VisionStart();

    static auto ParsevisionWalk(const string &cmd, const map<string, string> &param, aris::core::Msg &msg) -> void;
    static auto StopvisionWalk(const string &cmd, const map<string, string> &param, aris::core::Msg &msg) -> void;
    static auto visionWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in) -> int;

private:
    static atomic_bool isTerrainAnalysisFinished;
    static atomic_bool isSending;
    static atomic_bool isStop;
};
extern VisionWalkWrapper visionWalkWrapper;

}

#endif // VISIONWALK_H
