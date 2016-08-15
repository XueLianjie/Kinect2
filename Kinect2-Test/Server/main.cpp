#include "Kinect2.h"
#include <chrono>
#include <thread>

int main()
{
    Kinect2Sensor::KINECT2 kinect2;
    kinect2.Start();
    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        kinect2.Update();
        kinect2.SavePcd();
    }
    kinect2.Stop();

    char a;
    cin>>a;
}
