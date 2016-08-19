#ifndef KINECT2GAIT_H
#define KINECT2GAIT_H

#include <aris.h>
#include <Robot_Base.h>

struct Kinect2WalkParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 3000 };
    std::int32_t n{ 2 };
    double d{ 0.5 };
    double h{ 0.05 };
    double alpha{ 0.3 };
    double beta{ 0.3 };
};
auto kinect2WalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto kinect2WalkGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param)->int;

#endif // KINECT2GAIT_H
