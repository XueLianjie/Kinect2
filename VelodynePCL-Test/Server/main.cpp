#include "Velodyne.h"
#include <chrono>
#include <thread>

int main()
{
    VelodyneSensor::VELODYNE mVelodyne;
    mVelodyne.Start();
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        mVelodyne.Update();
//        cout<<mVelodyne.visData.gridMap[200][200]<<endl;
    }
    mVelodyne.Stop();

    char a;
    cin>>a;
}
