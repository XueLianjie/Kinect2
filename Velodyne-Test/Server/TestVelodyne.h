#ifndef TESTVELODYNE_H
#define TESTVELODYNE_H

#include <iostream>

#include <thread>
#include <aris.h>
#include <rtdk.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>
#include "Aris_Vision.h"

using namespace aris::core;
using namespace std;

namespace Testvelodyne
{

class TestVelodyneWrapper
{
public:

    static aris::sensor::VELODYNE velodyne1;
    static std::thread visionThread;

    static void VelodyneStart();
};
static TestVelodyneWrapper wrapper;
}

#endif // TESTVELODYNE_H
