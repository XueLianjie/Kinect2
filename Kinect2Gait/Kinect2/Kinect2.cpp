#include "Kinect2.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
#include <libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <Eigen/Dense>

namespace Kinect2Sensor
{

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef typename Cloud::Ptr CloudPtr;

void GenPointCoud(const CloudPtr &rawCloud, CloudPtr &adjCloud)
{
    adjCloud->clear();
    //    cout<<"Point: "<<rawCloud->points.front().x<<" "<<rawCloud->points.front().y<<" "<< rawCloud->points.front().z<<endl;

    cout<<"111"<<endl;

    std::vector<int> mapping;
    CloudPtr newcloud(new Cloud);
    pcl::removeNaNFromPointCloud(*rawCloud, *newcloud, mapping);

    cout<<"222"<<endl;

//    CloudPtr newcloud1(new Cloud);
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor(true);
//    sor.setInputCloud(newcloud);
//    cout<<"223"<<endl;
//    sor.setMeanK(1000);
//    sor.setStddevMulThresh(1.0);
//    cout<<"224"<<endl;
//    sor.filter(*newcloud1);

//    cout<<"333"<<endl;

    Eigen::Matrix4f matrixSTS;
    matrixSTS << -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix4f matrixSTR;
    matrixSTR <<  0.9996, 0.0250, 0.0108, -0.0459,
            -0.0061, 0.5924, -0.8056, 0.3380,
            -0.0266, 0.8053, 0.5923, 0.5216,
            0, 0, 0, 1;

    Eigen::Matrix4f matrixRTG;
    matrixRTG <<  1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix4f matrixSTG;
    
    matrixSTG = matrixRTG * matrixSTR * matrixSTS;

    CloudPtr newcloud2(new Cloud);
    pcl::transformPointCloud(*newcloud, *newcloud2, matrixSTG);

    cout<<"444"<<endl;

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (newcloud2);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 2.0);
    pass.filter (*adjCloud);

    cout<<"555"<<endl;
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
public:
    KINECT2_STRUCT();
    ~KINECT2_STRUCT();
private:
    std::string  serial;
    int frameNum = 0;
    CloudPtr mPointCloud;
    // CloudPtr lastPointCloud;
    float lastGridMap[400][400];
    float nowGridMap[400][400];
    float robPose[16] = {0};

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

KINECT2::KINECT2_STRUCT::KINECT2_STRUCT():mPointCloud(new Cloud)//, lastPointCloud(new Cloud(512, 424))
{
    ;
}

KINECT2::KINECT2_STRUCT::~KINECT2_STRUCT()
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

void KINECT2::SaveMap()
{
    std::stringstream out;
    out<<mKinect2Struct->frameNum;
    std::string filename = "GridMap" + out.str() + ".txt";
    std::ofstream Gridmapfile(filename);

    if (Gridmapfile.is_open())
    {
        for(int i = 0; i < 400; i++)
        {
            for(int j = 0; j < 400; j++)
            {
                Gridmapfile<<visData.gridMap[i][j]<<" ";
            }
            Gridmapfile<<std::endl;
        }
    }
    mKinect2Struct->frameNum++;
}

void KINECT2::GetPose(const float * nowPose)
{
    memcpy(mKinect2Struct->robPose, nowPose, 16 * sizeof(float));
}

void KINECT2::InitMap()
{
    Update();

    memset(mKinect2Struct->nowGridMap, 0, 400 * 400 * sizeof(float));

    int cGridNum[400][400] = {0};

    for (size_t i = 0; i < mKinect2Struct->mPointCloud->points.size(); ++i)
    {
        if(mKinect2Struct->mPointCloud->points[i].x > -2 && mKinect2Struct->mPointCloud->points[i].x < 2 &&
                mKinect2Struct->mPointCloud->points[i].z > 0.7 && mKinect2Struct->mPointCloud->points[i].z < 2)
        {
            int m = 0, n = 0;
            n = floor(mKinect2Struct->mPointCloud->points[i].x / 0.01) + 200;
            m = floor(mKinect2Struct->mPointCloud->points[i].z / 0.01) + 200;

            //Mean
            mKinect2Struct->nowGridMap[m][n] = (mKinect2Struct->nowGridMap[m][n] * cGridNum[m][n] + mKinect2Struct->mPointCloud->points[i].y) / (cGridNum[m][n] + 1);

            cGridNum[m][n] = cGridNum[m][n] + 1;
        }
    }

    // fill the backpart
    for(int i = 0; i < 400; i++)
    {
        for(int j = 275; j < 400; j++)
        {
            if(mKinect2Struct->nowGridMap[j][i] != 0)
            {
                for(int k = j - 1; k >= 0; k--)
                {
                    mKinect2Struct->nowGridMap[k][i] = mKinect2Struct->nowGridMap[j][i];
                }
                break;
            }
        }
    }

    memcpy(mKinect2Struct->lastGridMap, mKinect2Struct->nowGridMap, 400 * 400 * sizeof(float));

    // *mKinect2Struct->lastPointCloud = *mKinect2Struct->mPointCloud;

    memcpy(visData.gridMap, mKinect2Struct->nowGridMap, 400 * 400 * sizeof(float));
}

void KINECT2::UpdateConMap()
{
    Update();

    memset(mKinect2Struct->nowGridMap, 0, 400 * 400 * sizeof(float));

    int cGridNum[400][400] = {0};

    for (size_t i = 0; i < mKinect2Struct->mPointCloud->points.size(); ++i)
    {
        if(mKinect2Struct->mPointCloud->points[i].x > -2 && mKinect2Struct->mPointCloud->points[i].x < 2 &&
                mKinect2Struct->mPointCloud->points[i].z > 0.7 && mKinect2Struct->mPointCloud->points[i].z < 2)
        {
            int m = 0, n = 0;
            n = floor(mKinect2Struct->mPointCloud->points[i].x / 0.01) + 200;
            m = floor(mKinect2Struct->mPointCloud->points[i].z / 0.01) + 200;

            //Mean
            mKinect2Struct->nowGridMap[m][n] = (mKinect2Struct->nowGridMap[m][n] * cGridNum[m][n] + mKinect2Struct->mPointCloud->points[i].y) / (cGridNum[m][n] + 1);

            cGridNum[m][n] = cGridNum[m][n] + 1;
        }
    }

    Eigen::Matrix4f posMatrix;
    posMatrix<< mKinect2Struct->robPose[0], mKinect2Struct->robPose[1], mKinect2Struct->robPose[2], mKinect2Struct->robPose[3],
            mKinect2Struct->robPose[4], mKinect2Struct->robPose[5], mKinect2Struct->robPose[6], mKinect2Struct->robPose[7],
            mKinect2Struct->robPose[8], mKinect2Struct->robPose[9], mKinect2Struct->robPose[10], mKinect2Struct->robPose[11],
            mKinect2Struct->robPose[12], mKinect2Struct->robPose[13], mKinect2Struct->robPose[14], mKinect2Struct->robPose[15];

    Eigen::Matrix4f invPosMatrix = posMatrix.inverse();

    //    CloudPtr mLastPointCloud;
    //    pcl::transformPointCloud(*mKinect2Struct->lastPointCloud, *mLastPointCloud, invPosMatrix);

    //    for (size_t i = 0; i < mLastPointCloud->points.size(); ++i)
    //    {
    //        if(mLastPointCloud->points[i].x > -2 && mLastPointCloud->points[i].x < 2 &&
    //                mLastPointCloud->points[i].z > -2 && mLastPointCloud->points[i].z < 2)
    //        {
    //            int m = 0, n = 0;
    //            n = floor(mLastPointCloud->points[i].x / 0.01) + 200;
    //            m = floor(mLastPointCloud->points[i].z / 0.01) + 200;

    //            //Mean
    //            mKinect2Struct->nowGridMap[m][n] = (mKinect2Struct->nowGridMap[m][n] * cGridNum[m][n] + mLastPointCloud->points[i].y) / (cGridNum[m][n] + 1);

    //            cGridNum[m][n] = cGridNum[m][n] + 1;
    //        }
    //    }

    for(int i = 0; i < 400; i++)
    {
        for(int j = 0; j < 400; j++)
        {
            if(mKinect2Struct->lastGridMap[i][j] != 0)
            {
                for(int p = 0; p <= 10; p++)
                {
                    for(int q = 0; q <= 10; q++)
                    {
                        float tempPoint[3];
                        tempPoint[0] = 0.01 * (j - 200) + 0.001 * q;
                        tempPoint[1] = mKinect2Struct->lastGridMap[i][j];
                        tempPoint[2] = 0.01 * (i - 200) + 0.001 * p;

                        float tempTransPoint[3];
                        tempTransPoint[0] = invPosMatrix(0, 0) * tempPoint[0] + invPosMatrix(0, 1) * tempPoint[1] + invPosMatrix(0, 2) * tempPoint[2] + invPosMatrix(0, 3);
                        tempTransPoint[1] = invPosMatrix(1, 0) * tempPoint[0] + invPosMatrix(1, 1) * tempPoint[1] + invPosMatrix(1, 2) * tempPoint[2] + invPosMatrix(1, 3);
                        tempTransPoint[2] = invPosMatrix(2, 0) * tempPoint[0] + invPosMatrix(2, 1) * tempPoint[1] + invPosMatrix(2, 2) * tempPoint[2] + invPosMatrix(2, 3);

                        if(tempTransPoint[0] > -2 && tempTransPoint[0] < 2 &&
                                tempTransPoint[2] > -2 && tempTransPoint[2] < 2)
                        {
                            int m = 0, n = 0;
                            n = floor(tempTransPoint[0] / 0.01) + 200;
                            m = floor(tempTransPoint[2] / 0.01) + 200;

                            //Mean
                            mKinect2Struct->nowGridMap[m][n] = (mKinect2Struct->nowGridMap[m][n] * cGridNum[m][n] + tempTransPoint[1]) / (cGridNum[m][n] + 1);

                            cGridNum[m][n] = cGridNum[m][n] + 1;
                        }
                    }
                }
            }
        }
    }

    memcpy(mKinect2Struct->lastGridMap, mKinect2Struct->nowGridMap, 400 * 400 * sizeof(float));

    //  *mKinect2Struct->lastPointCloud = *mKinect2Struct->mPointCloud;

    memcpy(visData.gridMap, mKinect2Struct->nowGridMap, 400 * 400 * sizeof(float));
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

    cout<<"aaa"<<endl;
    cloud->is_dense = false;
    GenPointCoud(cloud, mKinect2Struct->mPointCloud);

    mKinect2Struct->listener->release(mKinect2Struct->frames);

    std::stringstream out;
    out<< mKinect2Struct->frameNum;
    std::string fileName = "cloud" + out.str() +".pcd";
    pcl::io::savePCDFileASCII(fileName, *mKinect2Struct->mPointCloud);
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

    memset(nowGridMap, 0, 400 * 400 * sizeof(float));
    memset(lastGridMap, 0, 400 * 400 * sizeof(float));

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

