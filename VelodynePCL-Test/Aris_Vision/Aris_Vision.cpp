#include "Aris_Vision.h"
#include <vector>
#include <string>
#include <pcl/io/vlp_grabber.h>
#include <pcl/common/transforms.h>
#include "math.h"
#include <sstream>
#include <fstream>

using namespace pcl;

namespace aris
{

namespace sensor
{

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

static int frameNum = 0;

struct Point3D
{
    float X;
    float Y;
    float Z;
};

// ax + by + cz = d; a^2 + b^2 + c^2 = 1;
void CalPlane(vector<Point3D>& cPointSet, GridMap &cgridmap)
{
    int pointNum = cPointSet.size();
    MatrixXf pointSet(pointNum,3);
    Matrix3f A(3,3);
    A<<0, 0, 0, 0, 0, 0, 0, 0, 0;

    for(int i = 0; i < pointNum; i++)
    {
        pointSet(i,0) = cPointSet[i].X;
        pointSet(i,1) = cPointSet[i].Y;
        pointSet(i,2) = cPointSet[i].Z;
    }

    float xBar = pointSet.col(0).sum()/pointNum;
    float yBar = pointSet.col(1).sum()/pointNum;
    float zBar = pointSet.col(2).sum()/pointNum;

    for(int i = 0; i < pointNum; i++)
    {
        A(0, 0) += (pointSet(i,0) - xBar)*(pointSet(i,0) - xBar);
        A(0, 1) += (pointSet(i,0) - xBar)*(pointSet(i,1) - yBar);
        A(0, 2) += (pointSet(i,0) - xBar)*(pointSet(i,2) - zBar);
        A(1, 0) += (pointSet(i,1) - yBar)*(pointSet(i,0) - xBar);
        A(1, 1) += (pointSet(i,1) - yBar)*(pointSet(i,1) - yBar);
        A(1, 2) += (pointSet(i,1) - yBar)*(pointSet(i,2) - zBar);
        A(2, 0) += (pointSet(i,2) - zBar)*(pointSet(i,0) - xBar);
        A(2, 1) += (pointSet(i,2) - zBar)*(pointSet(i,1) - yBar);
        A(2, 2) += (pointSet(i,2) - zBar)*(pointSet(i,2) - zBar);
    }

    EigenSolver<MatrixXf> es(A);

    VectorXcf eigvals = es.eigenvalues();
    Vector3f eigvalues;
    eigvalues<<real(eigvals(0)), real(eigvals(1)), real(eigvals(2));

    MatrixXcf eigvect = es.eigenvectors();
    Matrix3f eigvectors;
    eigvectors <<real(eigvect(0,0)), real(eigvect(0,1)), real(eigvect(0,2)), real(eigvect(1,0)), real(eigvect(1,1)), real(eigvect(1,2)),
            real(eigvect(2,0)), real(eigvect(2,1)), real(eigvect(2,2));

    float minValue = eigvalues(0);
    int minNum = 0;

    for(int i = 1; i < 3; i++)
    {
        if(eigvalues(i) < minValue)
        {
            minValue = eigvalues(i);
            minNum = i;
        }
    }

    float planePara[4] = {0, 0, 0, 0};

    planePara[0] = eigvectors(0, minNum);
    planePara[1] = eigvectors(1, minNum);
    planePara[2] = eigvectors(2, minNum);

    planePara[3] = planePara[0]*xBar + planePara[1]*yBar + planePara[2]*zBar;

    if(planePara[0] < 0)
    {
        for(int i = 0; i < 4; i++)
        {
            cgridmap.planePara[i] = -planePara[i];
        }
    }
    else
    {
        for(int i = 0; i < 4; i++)
        {
            cgridmap.planePara[i] = planePara[i];
        }
    }

    float distance1 = 0;
    float distance2 = sqrt(cgridmap.planePara[0]*cgridmap.planePara[0] + cgridmap.planePara[1]*cgridmap.planePara[1] + cgridmap.planePara[2]*cgridmap.planePara[2]);

    for(int i = 0; i < pointNum; i++)
    {
        distance1 += fabs(cgridmap.planePara[0]*pointSet(i,0) + cgridmap.planePara[1]*pointSet(i,1) + cgridmap.planePara[2]*pointSet(i,2) - cgridmap.planePara[3]);
    }

    cgridmap.planeDegree = distance1/distance2/pointNum;
    cgridmap.normalVector = acos(cgridmap.planePara[1]/distance2)/3.1415926*180;
}

void GenPointCoud(const CloudConstPtr &rawCloud, CloudPtr &adjCloud)
{
    Eigen::Matrix4f matrixSTS;
    matrixSTS << -1, 0, 0, 0,
            0, -1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    Eigen::Matrix4f matrixSTR;
    matrixSTR <<  0.9995, 0.0134, -0.0273, 0.0224,
            -0.0304, 0.5120, -0.8584, 0.2026 + 0.038,
            0.0025, 0.8589, 0.5122, 0.5733,
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
            cdata.pGridMap[m][n].Y = (cdata.pGridMap[m][n].Y * cGridNum[m][n] + adjCloud->points[i].y) / (cGridNum[m][n] + 1);

            cdata.gridMap[m][n] = cdata.pGridMap[m][n].Y;

            cGridNum[m][n] = cGridNum[m][n] + 1;

            cdata.pGridMap[m][n].pointNum = cGridNum[m][n];
            cdata.pGridMap[m][n].X = (n - 200) * 0.01;
            cdata.pGridMap[m][n].Z = m * 0.01;
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

VELODYNE::VELODYNE()
{
    ;
}

VELODYNE::~VELODYNE()
{
    ;
}

void VELODYNE::updateData(VISION_DATA &data)
{
    VELODYNE_BASE::updateData(data);
    frameNum++;
}

class VELODYNE_BASE::VELODYNE_BASE_STRUCT
{
    friend class VELODYNE_BASE;
private:
    VLPGrabber grabber;
    boost::mutex cloud_mutex_;
    CloudConstPtr cloud_;
    boost::signals2::connection cloud_connection;
    void cloud_callback(const CloudConstPtr& cloud);
    void Init();
    void Stop();
};

VELODYNE_BASE::VELODYNE_BASE():mVelodyneStruct(new VELODYNE_BASE::VELODYNE_BASE_STRUCT)
{
    ;
}

void VELODYNE_BASE::VELODYNE_BASE_STRUCT::Init()
{
    boost::function <void (const CloudConstPtr&)> cloud_cb = boost::bind(&VELODYNE_BASE_STRUCT::cloud_callback, this, _1);
    this->cloud_connection = this->grabber.registerCallback(cloud_cb);
    this->grabber.start();
}

void VELODYNE_BASE::VELODYNE_BASE_STRUCT::Stop()
{
    this->grabber.stop();
    this->cloud_connection.disconnect();
}

void VELODYNE_BASE::VELODYNE_BASE_STRUCT::cloud_callback(const CloudConstPtr& cloud)
{
    boost::mutex::scoped_lock lock(cloud_mutex_);
    cloud_ = cloud;
}

void VELODYNE_BASE::init()
{
    mVelodyneStruct->Init();
}

VELODYNE_BASE::~VELODYNE_BASE()
{
    mVelodyneStruct->Stop();
    cout<<"Device Close!"<<endl;
}

void VELODYNE_BASE::release()
{
    mVelodyneStruct->Stop();
    cout<<"Device Close!"<<endl;
}

void VELODYNE_BASE::updateData(VISION_DATA &data)
{
    memset(&data, 0, sizeof(data));

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
            CloudPtr adjCloud;
            GenPointCoud(rawCloud, adjCloud);
            GenGridMap(adjCloud, data);
            GenObstacleMap(data);
            break;
        }
    }
}

}

}

