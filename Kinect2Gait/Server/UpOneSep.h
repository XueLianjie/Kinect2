#ifndef UPONESEP_H
#define UPONESEP_H

#include <aris.h>
#include <Robot_Gait.h>

struct UpOneStepParam final :public aris::server::GaitParamBase
{
    double stepHeight;
    double bodyHeight;
    double normalHeight;
    std::int32_t totalCount;
};
void ParseUpOnestep(const std::string &cmd, const map<std::string, std::string> &params, aris::core::Msg &msg);
int UpOneStepGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in);

#endif // UPONESEP_H
