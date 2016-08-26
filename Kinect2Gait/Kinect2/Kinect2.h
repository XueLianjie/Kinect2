#ifndef KINECT2_H
#define KINECT2_H

#include <stdlib.h>
#include <memory>
#include <iostream>

using namespace std;

namespace Kinect2Sensor
{

struct VISION_DATA
{
    unsigned long long timeStamp;
    // float poitCloud[512][424][3];
    float gridMap[400][400];
    int obstacleMap[400][400];
};

class KINECT2
{
public:
    KINECT2();
    ~KINECT2();

    void Start();
    void InitMap();
    void UpdateConMap();
    void SavePcd();
    void Stop();
    void GetPose(const float * nowPose);
    void SaveMap();
    VISION_DATA visData;
private:
   void Update();
   class KINECT2_STRUCT;
   std::auto_ptr<KINECT2_STRUCT> mKinect2Struct;
};

}

#endif // KINECT2_H
