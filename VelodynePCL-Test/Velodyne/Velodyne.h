#ifndef VELODYNE_H
#define VELODYNE_H

#include <stdlib.h>
#include <memory>


namespace VelodyneSensor
{

struct VISION_DATA
{
    unsigned long long timeStamp;
    float gridMap[400][400];
    int obstacleMap[400][400];
};

class VELODYNE
{
public:
    VELODYNE();
    ~VELODYNE();

    void Start();
    void Update();
    void Stop();
    VISION_DATA visData;
private:
   class VELODYNE_STRUCT;
   std::auto_ptr<VELODYNE_STRUCT> mVelodyneStruct;
};

}



#endif // VELODYNE_H
