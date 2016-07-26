#ifndef ARIS_VISION_H_
#define ARIS_VISION_H_

#define linux 1

#include "aris_sensor.h"
#include <memory>
#include <stdlib.h>
#include <iostream>
#include <cstring>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace aris
{

namespace sensor
{

struct GridMap
{
    bool isStepOK;
    float X, Y, Z;
    float pointNum;
    float normalVector;
    float planePara[4];
    float planeDegree;
};

struct VISION_DATA
{
    unsigned long long timeStamp;
    GridMap pGridMap[400][400];
    float gridMap[400][400];
    int obstacleMap[400][400];
};


class VELODYNE_BASE: public SensorBase<VISION_DATA>
{
public:
    VELODYNE_BASE();
    virtual ~VELODYNE_BASE();

protected:
    virtual auto init()->void;
    virtual auto release()->void;
    virtual auto updateData(VISION_DATA &data)->void;

private:
    class VELODYNE_BASE_STRUCT;
    std::auto_ptr<VELODYNE_BASE_STRUCT> mVelodyneStruct;
};

class VELODYNE: public VELODYNE_BASE
{
public:
    VELODYNE();
    ~VELODYNE();
private:
    virtual void updateData(VISION_DATA &data);
};

}
}

#endif // ARIS_VISION_H_
