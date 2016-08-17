#include "Kinect2.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
#include <libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <Eigen/Dense>

namespace Kinect2Sensor
{

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef typename Cloud::Ptr CloudPtr;

void GenPointCoud(const CloudPtr &rawCloud, CloudPtr &adjCloud)
{
    cout<<"Point: "<<rawCloud->points.front().x<<" "<<rawCloud->points.front().y<<" "<< rawCloud->points.front().z<<endl;

    Eigen::Matrix4f matrixSTS;
    matrixSTS << -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix4f matrixSTR;
    matrixSTR <<  0.9995, 0.0286, 0.0159, -0.0437,
            -0.0043, 0.5949, -0.8038, 0.3875,
            -0.0324, 0.8033, 0.5947, 0.5138,
            0, 0, 0, 1;

    Eigen::Matrix4f matrixRTG;
    matrixRTG <<  1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix4f matrixSTG;
    
    matrixSTG = matrixRTG * matrixSTR * matrixSTS;

    pcl::transformPointCloud(*rawCloud, *adjCloud, matrixSTG);
}

void GenGridMap(const CloudPtr &adjCloud, VISION_DATA &cdata)
{
    int cGridNum[400][400] = {0};

    for (size_t i = 0; i < adjCloud->points.size(); ++i)
    {
        if(adjCloud->points[i].x > -2 && adjCloud->points[i].x < 2 &&
                adjCloud->points[i].z > 0 && adjCloud->points[i].z < 4)
        {
            int m = 0, n = 0;
            n = floor(adjCloud->points[i].x / 0.01) + 200;
            m = floor(adjCloud->points[i].z / 0.01);

            //Mean
            cdata.gridMap[m][n] = (cdata.gridMap[m][n] * cGridNum[m][n] + adjCloud->points[i].y) / (cGridNum[m][n] + 1);

            cGridNum[m][n] = cGridNum[m][n] + 1;
        }
    }
}

void GenObstacleMap(VISION_DATA &cdata)
{
    for(int i =0; i < 400; i++)
    {
        for(int j = 0; j < 400; j++)
        {
            if(cdata.gridMap[i][j] > 0.1)
            {
                cdata.obstacleMap[i][j] = 1;
            }
        }
    }
}

class KINECT2::KINECT2_STRUCT
{
    friend class KINECT2;
private:
    std::string  serial;
    int frameNum = 0;

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;
    libfreenect2::Registration *registration = 0;
    libfreenect2::SyncMultiFrameListener *listener = 0;
    libfreenect2::FrameMap frames;

    void Initialize();
};

KINECT2::KINECT2():mKinect2Struct(new KINECT2::KINECT2_STRUCT)
{
    ;
}

KINECT2::~KINECT2()
{
    mKinect2Struct->dev->stop();
    mKinect2Struct->dev->close();
}

void KINECT2::Start()
{
    mKinect2Struct->Initialize();
    SavePcd();
}

void KINECT2::Stop()
{
    mKinect2Struct->dev->stop();
    mKinect2Struct->dev->close();
}

void KINECT2::SavePcd()
{
    mKinect2Struct->listener->waitForNewFrame(mKinect2Struct->frames);
    libfreenect2::Frame *depth = mKinect2Struct->frames[libfreenect2::Frame::Depth];

    libfreenect2::Frame undistorted(512, 424, 4);

    mKinect2Struct->registration->undistortDepth(depth, &undistorted);

    CloudPtr cloud(new Cloud(512, 424));
    pcl::PointXYZ *itp = &cloud->points[0];

    for(int row = 0; row < 424; ++row)
    {
        for(int col = 0; col < 512; ++col, ++itp)
        {
            float x, y, z;
            mKinect2Struct->registration->getPointXYZ(&undistorted, row, col, x, y, z);
            itp->x = x;
            itp->y = y;
            itp->z = z;
        }
    }

    Cloud tempCloud = *cloud;
    tempCloud.width = 512*424;
    tempCloud.height = 1;
    std::stringstream out;
    out<< mKinect2Struct->frameNum;
    std::string fileName = "cloud" + out.str() +".pcd";
    pcl::io::savePCDFileASCII(fileName, tempCloud);
    mKinect2Struct->frameNum++;

    mKinect2Struct->listener->release(mKinect2Struct->frames);
}

void KINECT2::Update()
{
    memset(&visData, 0, sizeof(visData));

    mKinect2Struct->listener->waitForNewFrame(mKinect2Struct->frames);
    libfreenect2::Frame *depth = mKinect2Struct->frames[libfreenect2::Frame::Depth];

    libfreenect2::Frame undistorted(512, 424, 4);

    mKinect2Struct->registration->undistortDepth(depth, &undistorted);

    visData.timeStamp = undistorted.timestamp;

    CloudPtr cloud(new Cloud(512, 424));
    pcl::PointXYZ *itp = &cloud->points[0];

    for(int row = 0; row < 424; ++row)
    {
        for(int col = 0; col < 512; ++col, ++itp)
        {
            float x, y, z;
            mKinect2Struct->registration->getPointXYZ(&undistorted, row, col, x, y, z);
            itp->x = x;
            itp->y = y;
            itp->z = z;
        }
    }

    CloudPtr transformcloud(new Cloud(512, 424));
    GenPointCoud(cloud, transformcloud);

    GenGridMap(transformcloud, visData);

    GenObstacleMap(visData);

    mKinect2Struct->listener->release(mKinect2Struct->frames);
}
void KINECT2::KINECT2_STRUCT::Initialize()
{
    if(freenect2.enumerateDevices() == 0)
    {
        std::cout<<"no kinect2 connected!"<<std::endl;
        exit(-1);
    }
    else
    {
       serial = freenect2.getDefaultDeviceSerialNumber();
    }

    pipeline = new libfreenect2::CpuPacketPipeline();

    dev = freenect2.openDevice(serial, pipeline);

    int types = 0;
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    listener = new libfreenect2::SyncMultiFrameListener(types);

    dev->setIrAndDepthFrameListener(listener);
    dev->start();

    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
}

}

