#include "Velodyne.h"
#include <iostream>
#include <chrono>
#include <thread>
using namespace std;

int main()
{
    VelodyneSensor::VELODYNE mVelodyne;
    mVelodyne.Start();
    while(true)
    {
        mVelodyne.Update();
        cout<<mVelodyne.visData.gridMap[200][200]<<endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    mVelodyne.Stop();

    char a;
    cin>>a;

}
