#include "Kinect2Test.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
#include <libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <Eigen/Dense>

namespace Kinect2TestSensor
{

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef typename Cloud::Ptr CloudPtr;

void GenPointCoud(const CloudPtr &rawCloud, CloudPtr &adjCloud)
{
    //    cout<<"Point: "<<rawCloud->points.front().x<<" "<<rawCloud->points.front().y<<" "<< rawCloud->points.front().z<<endl;

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

class KINECT2TEST::KINECT2TEST_STRUCT
{
    friend class KINECT2TEST;
public:
    KINECT2TEST_STRUCT();
    ~KINECT2TEST_STRUCT();
private:
    std::string  serial;
    int frameNum = 0;
    CloudPtr mPointCloud;
    float lastGridMap[400][400];
    float nowGridMap[400][400];
    float robPose[16] = {0};

//    libfreenect2::Freenect2 freenect2;
//    libfreenect2::Freenect2Device *dev = 0;
//    libfreenect2::PacketPipeline *pipeline = 0;
//    libfreenect2::Registration *registration = 0;
//    libfreenect2::SyncMultiFrameListener *listener = 0;
//    libfreenect2::FrameMap frames;

    void Initialize();
};

KINECT2TEST::KINECT2TEST():mKinect2TestStruct(new KINECT2TEST::KINECT2TEST_STRUCT)
{
    ;
}

KINECT2TEST::KINECT2TEST_STRUCT::KINECT2TEST_STRUCT():mPointCloud(new Cloud(512, 424))
{
    ;
}

KINECT2TEST::KINECT2TEST_STRUCT::~KINECT2TEST_STRUCT()
{
    ;
}

KINECT2TEST::~KINECT2TEST()
{
//    mKinect2TestStruct->dev->stop();
//    mKinect2TestStruct->dev->close();
}

void KINECT2TEST::Start()
{
//    mKinect2TestStruct->Initialize();
    SavePcd();
}

void KINECT2TEST::Stop()
{
//    mKinect2TestStruct->dev->stop();
//    mKinect2TestStruct->dev->close();
}

void KINECT2TEST::SaveMap()
{
    std::stringstream out;
    out<<mKinect2TestStruct->frameNum;
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
    mKinect2TestStruct->frameNum++;
}

void KINECT2TEST::GetPose(const float * nowPose)
{
    memcpy(mKinect2TestStruct->robPose, nowPose, 16 * sizeof(float));
    cout<<mKinect2TestStruct->robPose[0]<<" "<<mKinect2TestStruct->robPose[1]<<" "<<mKinect2TestStruct->robPose[2]<<" "<<mKinect2TestStruct->robPose[3]<<endl
                                      <<mKinect2TestStruct->robPose[4]<<" "<<mKinect2TestStruct->robPose[5]<<" "<<mKinect2TestStruct->robPose[6]<<" "<<mKinect2TestStruct->robPose[7]<<endl
                                        <<mKinect2TestStruct->robPose[8]<<" "<<mKinect2TestStruct->robPose[9]<<" "<<mKinect2TestStruct->robPose[10]<<" "<<mKinect2TestStruct->robPose[11]<<endl
                                          <<mKinect2TestStruct->robPose[12]<<" "<<mKinect2TestStruct->robPose[13]<<" "<<mKinect2TestStruct->robPose[14]<<" "<<mKinect2TestStruct->robPose[15]<<endl;
}

void KINECT2TEST::InitMap()
{
    Update();

//    memset(mKinect2TestStruct->nowGridMap, 0, 400 * 400 * sizeof(float));

    int cGridNum[400][400] = {0};

//    for (size_t i = 0; i < mKinect2TestStruct->mPointCloud->points.size(); ++i)
//    {
//        if(mKinect2TestStruct->mPointCloud->points[i].x > -2 && mKinect2TestStruct->mPointCloud->points[i].x < 2 &&
//                mKinect2TestStruct->mPointCloud->points[i].z > 0.7 && mKinect2TestStruct->mPointCloud->points[i].z < 2)
//        {
//            int m = 0, n = 0;
//            n = floor(mKinect2TestStruct->mPointCloud->points[i].x / 0.01) + 200;
//            m = floor(mKinect2TestStruct->mPointCloud->points[i].z / 0.01) + 200;

//            //Mean
//            mKinect2TestStruct->nowGridMap[m][n] = (mKinect2TestStruct->nowGridMap[m][n] * cGridNum[m][n] + mKinect2TestStruct->mPointCloud->points[i].y) / (cGridNum[m][n] + 1);

//            cGridNum[m][n] = cGridNum[m][n] + 1;
//        }
//    }

    // fill the backpart
    for(int i = 0; i < 400; i++)
    {
        for(int j = 275; j < 400; j++)
        {
            if(mKinect2TestStruct->nowGridMap[j][i] != 0)
            {
                for(int k = j - 1; k >= 0; k--)
                {
                    mKinect2TestStruct->nowGridMap[k][i] = mKinect2TestStruct->nowGridMap[j][i];
                }
                break;
            }
        }
    }

    memcpy(mKinect2TestStruct->lastGridMap, mKinect2TestStruct->nowGridMap, 400 * 400 * sizeof(float));

    memcpy(visData.gridMap, mKinect2TestStruct->nowGridMap, 400 * 400 * sizeof(float));
}

void KINECT2TEST::UpdateConMap()
{
    Update();

//    memset(mKinect2TestStruct->nowGridMap, 0, 400 * 400 * sizeof(float));

    int cGridNum[400][400] = {0};

//    for (size_t i = 0; i < mKinect2TestStruct->mPointCloud->points.size(); ++i)
//    {
//        if(mKinect2TestStruct->mPointCloud->points[i].x > -2 && mKinect2TestStruct->mPointCloud->points[i].x < 2 &&
//                mKinect2TestStruct->mPointCloud->points[i].z > 0.7 && mKinect2TestStruct->mPointCloud->points[i].z < 2)
//        {
//            int m = 0, n = 0;
//            n = floor(mKinect2TestStruct->mPointCloud->points[i].x / 0.01) + 200;
//            m = floor(mKinect2TestStruct->mPointCloud->points[i].z / 0.01) + 200;

//            //Mean
//            mKinect2TestStruct->nowGridMap[m][n] = (mKinect2TestStruct->nowGridMap[m][n] * cGridNum[m][n] + mKinect2TestStruct->mPointCloud->points[i].y) / (cGridNum[m][n] + 1);

//            cGridNum[m][n] = cGridNum[m][n] + 1;
//        }
//    }

    Eigen::Matrix4f posMatrix;
    posMatrix<< mKinect2TestStruct->robPose[0], mKinect2TestStruct->robPose[1], mKinect2TestStruct->robPose[2], mKinect2TestStruct->robPose[3],
            mKinect2TestStruct->robPose[4], mKinect2TestStruct->robPose[5], mKinect2TestStruct->robPose[6], mKinect2TestStruct->robPose[7],
            mKinect2TestStruct->robPose[8], mKinect2TestStruct->robPose[9], mKinect2TestStruct->robPose[10], mKinect2TestStruct->robPose[11],
            mKinect2TestStruct->robPose[12], mKinect2TestStruct->robPose[13], mKinect2TestStruct->robPose[14], mKinect2TestStruct->robPose[15];

    Eigen::Matrix4f invPosMatrix = posMatrix.inverse();

    for(int i = 0; i < 400; i++)
    {
        for(int j = 0; j < 400; j++)
        {
            if(mKinect2TestStruct->lastGridMap[i][j] != 0)
            {
                float tempPoint[3];
                tempPoint[0] = (j - 200) * 0.01 - 0.005;
                tempPoint[1] = mKinect2TestStruct->lastGridMap[i][j];
                tempPoint[2] = (i - 200) * 0.01 - 0.005;

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
                    mKinect2TestStruct->nowGridMap[m][n] = (mKinect2TestStruct->nowGridMap[m][n] * cGridNum[m][n] + tempTransPoint[1]) / (cGridNum[m][n] + 1);

                    cGridNum[m][n] = cGridNum[m][n] + 1;
                }

            }
        }
    }

    memcpy(mKinect2TestStruct->lastGridMap, mKinect2TestStruct->nowGridMap, 400 * 400 * sizeof(float));

    memcpy(visData.gridMap, mKinect2TestStruct->nowGridMap, 400 * 400 * sizeof(float));
}

void KINECT2TEST::SavePcd()
{
//    mKinect2TestStruct->listener->waitForNewFrame(mKinect2TestStruct->frames);
//    libfreenect2::Frame *depth = mKinect2TestStruct->frames[libfreenect2::Frame::Depth];

//    libfreenect2::Frame undistorted(512, 424, 4);

//    mKinect2TestStruct->registration->undistortDepth(depth, &undistorted);

    CloudPtr cloud(new Cloud(512, 424));
    pcl::PointXYZ *itp = &cloud->points[0];

//    for(int row = 0; row < 424; ++row)
//    {
//        for(int col = 0; col < 512; ++col, ++itp)
//        {
//            float x, y, z;
//            mKinect2TestStruct->registration->getPointXYZ(&undistorted, row, col, x, y, z);
//            itp->x = x;
//            itp->y = y;
//            itp->z = z;
//        }
//    }

    Cloud tempCloud = *cloud;
    tempCloud.width = 512*424;
    tempCloud.height = 1;
    std::stringstream out;
    out<< mKinect2TestStruct->frameNum;
    std::string fileName = "cloud" + out.str() +".pcd";
    pcl::io::savePCDFileASCII(fileName, tempCloud);
    mKinect2TestStruct->frameNum++;

//   mKinect2TestStruct->listener->release(mKinect2TestStruct->frames);
}

void KINECT2TEST::Update()
{
    memset(&visData, 0, sizeof(visData));

//    mKinect2TestStruct->listener->waitForNewFrame(mKinect2TestStruct->frames);
//    libfreenect2::Frame *depth = mKinect2TestStruct->frames[libfreenect2::Frame::Depth];

//    libfreenect2::Frame undistorted(512, 424, 4);

//    mKinect2TestStruct->registration->undistortDepth(depth, &undistorted);

//    visData.timeStamp = undistorted.timestamp;

    CloudPtr cloud(new Cloud(512, 424));
    pcl::PointXYZ *itp = &cloud->points[0];

//    for(int row = 0; row < 424; ++row)
//    {
//        for(int col = 0; col < 512; ++col, ++itp)
//        {
//            float x, y, z;
//            mKinect2TestStruct->registration->getPointXYZ(&undistorted, row, col, x, y, z);
//            itp->x = x;
//            itp->y = y;
//            itp->z = z;
//        }
//    }

    GenPointCoud(cloud, mKinect2TestStruct->mPointCloud);

 //   mKinect2TestStruct->listener->release(mKinect2TestStruct->frames);
}

void KINECT2TEST::KINECT2TEST_STRUCT::Initialize()
{
//    if(freenect2.enumerateDevices() == 0)
//    {
//        std::cout<<"no kinect2 connected!"<<std::endl;
//        exit(-1);
//    }
//    else
//    {
//        serial = freenect2.getDefaultDeviceSerialNumber();
//    }

    memset(nowGridMap, 0, 400 * 400 * sizeof(float));
    memset(lastGridMap, 0, 400 * 400 * sizeof(float));

    nowGridMap[399][20] = 1;

//    pipeline = new libfreenect2::CpuPacketPipeline();

//    dev = freenect2.openDevice(serial, pipeline);

//    int types = 0;
//    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
//    listener = new libfreenect2::SyncMultiFrameListener(types);

//    dev->setIrAndDepthFrameListener(listener);
//    dev->start();

//    registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
}

}

