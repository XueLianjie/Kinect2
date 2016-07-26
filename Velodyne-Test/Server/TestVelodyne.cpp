#include "TestVelodyne.h"

namespace Testvelodyne
{

aris::sensor::VELODYNE TestVelodyneWrapper::velodyne1;
std::thread TestVelodyneWrapper::visionThread;

void TestVelodyneWrapper::VelodyneStart()
{
    velodyne1.start();

    visionThread = std::thread([]()
    {
        while(true)
        {
            auto visiondata = velodyne1.getSensorData();

            cout<<"avoidAnalysisFinished"<<endl;
        }
    });
}

}
