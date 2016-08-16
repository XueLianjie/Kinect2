#include "Velodyne.h"
#include "pcl/io/vlp_grabber.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"

using namespace pcl;

namespace VelodyneSensor
{

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

void GenPointCoud(const CloudConstPtr &rawCloud, CloudPtr &adjCloud)
{
    cout<<"Poin: "<<rawCloud->points.front().x<<" "<<rawCloud->points.front().y<<" "<< rawCloud->points.front().z<<endl;

    Eigen::Matrix4f matrixSTS;
    matrixSTS << -1, 0, 0, 0,
            0, 0, 1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1;

    Eigen::Matrix4f matrixSTR;
    matrixSTR <<  0.9999, 0.0132, 0.0088, 0.0108,
            -0.0033, 0.7145, -0.6996, 0.2615,
            -0.0155, 0.6995, 0.7144, 0.5368,
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

class VELODYNE::VELODYNE_STRUCT
{
    friend class VELODYNE;
private:
    void Init();
    void Release();
    VLPGrabber grabber;
    boost::mutex cloud_mutex_;
    CloudConstPtr cloud_;
    boost::signals2::connection cloud_connection;
    void cloud_callback(const CloudConstPtr& cloud);
    int frameNum = 0;
};

VELODYNE::VELODYNE():mVelodyneStruct(new VELODYNE::VELODYNE_STRUCT)
{
    ;
}

VELODYNE::~VELODYNE()
{
    ;
}

void VELODYNE::Start()
{
    mVelodyneStruct->Init();
    cout<<"Device Open ! "<<endl;
    for (int i = 0; i < 3; i++)
    {
        Update();
    }
    SavePcd();
    cout<<"Clear !"<<endl;
}

void VELODYNE::Stop()
{
    mVelodyneStruct->Release();
    cout<<"Device Close ! "<<endl;
}

void VELODYNE::SavePcd()
{
    while(true)
    {
        CloudConstPtr rawCloud;

        if(mVelodyneStruct->cloud_mutex_.try_lock())
        {
            mVelodyneStruct->cloud_.swap(rawCloud);
            mVelodyneStruct->cloud_mutex_.unlock();
        }

        if(rawCloud)
        {
            Cloud tempCloud = *rawCloud;
            std::stringstream out;
            out<< mVelodyneStruct->frameNum;
            std::string fileName = "cloud" + out.str() +".pcd";
            pcl::io::savePCDFileASCII(fileName, tempCloud);
            mVelodyneStruct->frameNum++;
            break;
        }
    }

}

void VELODYNE::Update()
{
    while(true)
    {
        CloudConstPtr rawCloud;

        if(mVelodyneStruct->cloud_mutex_.try_lock())
        {
            mVelodyneStruct->cloud_.swap(rawCloud);
            mVelodyneStruct->cloud_mutex_.unlock();
        }

        if(rawCloud)
        {
            memset(&visData, 0, sizeof(visData));
            CloudPtr adjCloud(new Cloud);
            GenPointCoud(rawCloud, adjCloud);
            GenGridMap(adjCloud, visData);
            GenObstacleMap(visData);
            break;
        }
    }
}

void VELODYNE::VELODYNE_STRUCT::cloud_callback(const CloudConstPtr& cloud)
{
    boost::mutex::scoped_lock lock(cloud_mutex_);
    cloud_ = cloud;
}

void VELODYNE::VELODYNE_STRUCT::Init()
{
    boost::function <void (const CloudConstPtr&)> cloud_cb = boost::bind(& VELODYNE::VELODYNE_STRUCT::cloud_callback, this, _1);
    this->cloud_connection = this->grabber.registerCallback(cloud_cb);
    this->grabber.start();
}

void VELODYNE::VELODYNE_STRUCT::Release()
{
    this->grabber.stop();
    this->cloud_connection.disconnect();
}

}

