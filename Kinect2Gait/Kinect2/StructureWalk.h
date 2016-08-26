#ifndef STRUCTUREWALK_H
#define STRUCTUREWALK_H

#include <atomic>
#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

#include "Server/PassStepDitch.h"

using namespace  std;

namespace StructureWalk
{

enum GAIT_CMD
{
    NOCMD = 0,
    GO = 1,
    STOP = 2,
};

struct UpGaitParam final: public aris::server::GaitParamBase
{
    double d;
    double h;
};

auto UpWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void;
auto UpWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & cali_param)->int;

}

#endif // STRUCTUREWALK_H
