#ifndef KINECT2TEST_H
#define KINECT2TEST_H

#include <stdlib.h>
#include <memory>
#include <iostream>

using namespace std;

namespace Kinect2TestSensor
{

struct VISION_DATA
{
    unsigned long long timeStamp;
    // float poitCloud[512][424][3];
    float gridMap[400][400];
    int obstacleMap[400][400];
};

class KINECT2TEST
{
public:
    KINECT2TEST();
    ~KINECT2TEST();

    void Start();
    void InitMap();
    void UpdateConMap();
    void InitMapTest();
    void UpdateConMapTest();
    void SavePcd();
    void Stop();
    void GetPose(const float *);
    void SaveMap();
    VISION_DATA visData;
private:
   void Update();
   class KINECT2TEST_STRUCT;
   std::auto_ptr<KINECT2TEST_STRUCT> mKinect2TestStruct;
};

}

#endif // KINECT2TEST_H
