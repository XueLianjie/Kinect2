#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <map>
#include <string>
#include <eigen3/Eigen/Dense>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>
#include <Robot_Type_I.h>
#include "Vision_Gait0.h"
#include "Kinect2.h"
#include "Kinect2Test.h"

#include "rtdk.h"
#include "unistd.h"

using namespace aris::core;

double feetPosi[18] =
{ -0.3,  -0.9, -0.65,
  -0.45, -0.9,  0,
  -0.3,  -0.9,  0.65,
  0.3,  -0.9, -0.65,
  0.45, -0.9,  0,
  0.3,   -0.9,  0.65 };

Kinect2Sensor::KINECT2 kinect2;

// Kinect2TestSensor::KINECT2TEST kinect2;

atomic_bool isTerrainCaliRecorded(false);

enum CalibrationState
{
    None=0,
    Go=1,
    Processing=2,
    Back=3,
};

atomic_int calibrationState{CalibrationState::None};

atomic_bool isSending(false);
atomic_bool isStop(false);

aris::control::Pipe<int> visionCalibratePipe(true);

void TransM(double matrixIn[6], double matrixOut[6])
{
    double	alpha = matrixIn[0];
    double	beta = matrixIn[1];
    double	gama = matrixIn[2];

    Eigen::Matrix3f R_X;
    R_X << 1, 0, 0, 0, cos(gama), -sin(gama), 0, sin(gama), cos(gama);

    Eigen::Matrix3f R_Z;
    R_Z << cos(beta), -sin(beta), 0, sin(beta), cos(beta), 0, 0, 0, 1;

    Eigen::Matrix3f R_Y;
    R_Y << cos(alpha), 0, sin(alpha), 0, 1, 0, -sin(alpha), 0, cos(alpha);

    Eigen::Matrix3f Pose;
    Pose = R_Y * R_Z * R_X;

    double pMatrix[4][4] = { 0 };

    for (size_t i = 0; i < 3; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            pMatrix[i][j] = Pose(i, j);
        }
    }
    pMatrix[0][3] = matrixIn[3];
    pMatrix[1][3] = matrixIn[4];
    pMatrix[2][3] = matrixIn[5];
    pMatrix[3][3] = 1;

    aris::dynamic::s_pm2pe(*pMatrix, matrixOut, "313");
}


static auto visionCalibrateThread = std::thread([]()
{
    while(true)
    {
        int postureCount;
        visionCalibratePipe.recvInNrt(postureCount);

        kinect2.SavePcd();
        cout<<" map recorded"<<endl;
        isTerrainCaliRecorded=true;
    }
});

auto visionCalibrateParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    VISION_CALIBRATION_PARAM param;

    double a;

    std::ifstream file;
    std::string FileName = "/home/hex/Kinect2/Kinect2Calibration/Pose.txt";
    cout<<"file name:"<<FileName<<endl;

    file.open(FileName);
    if (!file) throw std::logic_error("calibration params file not exist");
    file>>a;

    int postureNum{ 0 };

    for (double tem; !file.eof(); file >> tem)  ++postureNum;
    if (postureNum % 6 != 0) throw std::logic_error("calibration params file invalid, because the num of numbers is not valid");
    postureNum /= 6;
    file.close();

    param.postureNum = postureNum;

    file.open(FileName);
    //file>>a;
    for (int i = 0; !file.eof(); file >> param.postureLib[i++]);
    file.close();

    cout<<"postureNum:"<<postureNum<<endl;

    msg_out.copyStruct(param);
}

auto visionCalibrate(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & cali_param)->int
{
    auto &robot=static_cast<Robots::RobotBase &>(model);
    auto &pSP=static_cast< const VISION_CALIBRATION_PARAM &>(cali_param);
    static aris::dynamic::FloatMarker beginMak{ robot.ground() };
    static int localCount=0;
    static int postureCount=0;
    static double targetPosture[6];
    double currentPeb[6];
    double s;

    if(pSP.count==0)
    {
        rt_printf("calibration gait begins\n");
    }

    localCount = localCount%pSP.gaitLength;

    switch(calibrationState)
    {
    case None:
        calibrationState=Go;
        break;
    case Go:
        if(localCount==0)
        {
            rt_printf("calibration posture %d\n",postureCount);
            memcpy(targetPosture,&pSP.postureLib[6*postureCount],sizeof(targetPosture));
            rt_printf("%lf %lf %lf %lf %lf %lf \n", targetPosture[0], targetPosture[1], targetPosture[2], targetPosture[3], targetPosture[4], targetPosture[5]);
        }

        s=PI*(localCount+1)/pSP.gaitLength;// (0,pi]
        //        currentPeb[3]=targetPosture[2]*M_PI/180.0*(1-cos(s))/2;
        //        currentPeb[4]=targetPosture[1]*M_PI/180.0*(1-cos(s))/2;
        //        currentPeb[5]=targetPosture[0]*M_PI/180.0*(1-cos(s))/2;
        //        currentPeb[0]=targetPosture[3]*(1-cos(s))/2;
        //        currentPeb[1]=targetPosture[4]*(1-cos(s))/2;
        //        currentPeb[2]=targetPosture[5]*(1-cos(s))/2;


        //        robot.SetPeb(currentPeb,beginMak,"132");

        currentPeb[2]=targetPosture[2]*M_PI/180.0*(1-cos(s))/2;
        currentPeb[1]=targetPosture[1]*M_PI/180.0*(1-cos(s))/2;
        currentPeb[0]=targetPosture[0]*M_PI/180.0*(1-cos(s))/2;
        currentPeb[3]=targetPosture[3]*(1-cos(s))/2;
        currentPeb[4]=targetPosture[4]*(1-cos(s))/2 * 0.8;
        currentPeb[5]=targetPosture[5]*(1-cos(s))/2;

        double bodyPose[6];
        TransM(currentPeb, bodyPose);

        robot.SetPeb(bodyPose,beginMak);

        robot.SetPee(feetPosi, beginMak);
        localCount+=1;

        if(pSP.gaitLength-localCount==0)
        {
            calibrationState=Processing;
            visionCalibratePipe.sendToNrt(postureCount);
            localCount = 0;
            rt_printf("begin capture !\n");
            rt_printf("raw: %lf %lf %lf \n", currentPeb[0], currentPeb[1], currentPeb[2]);
            rt_printf("target: %lf %lf %lf \n", bodyPose[3], bodyPose[4], bodyPose[5]);
        }
        break;
    case Processing:
        if(isTerrainCaliRecorded==true)
        {
            calibrationState=Back;
            isTerrainCaliRecorded=false;
            rt_printf("end capture !\n");
        }
        break;
    case Back:
        if(localCount==0)
        {
            rt_printf("calibration posture finished going back %d\n",postureCount);
        }
        s=PI*(localCount+1)/pSP.gaitLength;// (0,pi]
        //        currentPeb[3]=targetPosture[2]*M_PI/180.0*(1+cos(s))/2;
        //        currentPeb[4]=targetPosture[1]*M_PI/180.0*(1+cos(s))/2;
        //        currentPeb[5]=targetPosture[0]*M_PI/180.0*(1+cos(s))/2;
        //        currentPeb[0]=targetPosture[3]*(1+cos(s))/2;
        //        currentPeb[1]=targetPosture[4]*(1+cos(s))/2;
        //        currentPeb[2]=targetPosture[5]*(1+cos(s))/2;


        //        robot.SetPeb(currentPeb,beginMak,"132");

        currentPeb[2]=targetPosture[2]*M_PI/180.0*(1+cos(s))/2;
        currentPeb[1]=targetPosture[1]*M_PI/180.0*(1+cos(s))/2;
        currentPeb[0]=targetPosture[0]*M_PI/180.0*(1+cos(s))/2;
        currentPeb[3]=targetPosture[3]*(1+cos(s))/2;
        currentPeb[4]=targetPosture[4]*(1+cos(s))/2 * 0.8;
        currentPeb[5]=targetPosture[5]*(1+cos(s))/2;

        double bodyPose1[6];
        TransM(currentPeb, bodyPose1);

        robot.SetPeb(bodyPose1,beginMak);

        robot.SetPee(feetPosi, beginMak);
        localCount+=1;

        if(pSP.gaitLength-localCount==0)
        {
            postureCount+=1;
            calibrationState=None;
            localCount = 0;
            if(postureCount==pSP.postureNum)
                return 0;
        }
        break;
    default:
        break;
    }

    return 1;

}

VISION_WALK_PARAM visionWalkParam[7];

float robPose[7][16] = {0};

aris::control::Pipe<int> visionWalkPipe(true);

atomic_bool isMapRecorded(false);

atomic_int stepNum{0};

static auto visionWalkThread = std::thread([]()
{
    while(true)
    {
        int postureCount;
        visionWalkPipe.recvInNrt(postureCount);

        if(stepNum == 0)
        {
            kinect2.InitMap();
            kinect2.SaveMap();
            cout<<"map Init"<<endl;
        }
        else
        {
            kinect2.GetPose(robPose[stepNum - 1]);
            kinect2.UpdateConMap();
            kinect2.SaveMap();
            cout<<"map update"<<endl;
        }

        cout<<" map recorded"<<endl;
        isMapRecorded = true;
    }
});


auto visionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    visionWalkParam[0].movetype = flatmove;
    double avoidMove0[3] = {0, 0, 0.4};
    memcpy(visionWalkParam[0].movedata, avoidMove0, 3*sizeof(double));
    visionWalkParam[0].totalCount = 3000;

    robPose[0][0] = 1;
    robPose[0][5] = 1;
    robPose[0][10] = 1;
    robPose[0][11] = 0.2;
    robPose[0][15] = 1;

    visionWalkParam[1].movetype = turn;
    visionWalkParam[1].turndata = -30;
    visionWalkParam[1].totalCount = 3000;

    robPose[1][5] = 1;
    robPose[1][0] = cos(M_PI * (-15) / 180.0);
    robPose[1][2] = sin(M_PI * (-15) / 180.0);
    robPose[1][8] = -sin(M_PI * (-15) / 180.0);
    robPose[1][10] = cos(M_PI * (-15) / 180.0);
    robPose[1][15] = 1;

    visionWalkParam[2].movetype = flatmove;
    double avoidMove1[3] = {0, 0, 0.4};
    memcpy(visionWalkParam[2].movedata, avoidMove1, 3*sizeof(double));
    visionWalkParam[2].totalCount = 3000;

    robPose[2][0] = 1;
    robPose[2][5] = 1;
    robPose[2][10] = 1;
    robPose[2][11] = 0.2;
    robPose[2][15] = 1;

    visionWalkParam[3].movetype = turn;
    visionWalkParam[3].turndata = 60;
    visionWalkParam[3].totalCount = 5000;

    robPose[3][5] = 1;
    robPose[3][0] = cos(M_PI * (30) / 180.0);
    robPose[3][2] = sin(M_PI * (30) / 180.0);
    robPose[3][8] = -sin(M_PI * (30) / 180.0);
    robPose[3][10] = cos(M_PI * (30) / 180.0);
    robPose[3][15] = 1;

    visionWalkParam[4].movetype = flatmove;
    double avoidMove2[3] = {0, 0, 0.4};
    memcpy(visionWalkParam[4].movedata, avoidMove2, 3*sizeof(double));
    visionWalkParam[4].totalCount = 3000;

    robPose[4][0] = 1;
    robPose[4][5] = 1;
    robPose[4][10] = 1;
    robPose[4][11] = 0.2;
    robPose[4][15] = 1;

    visionWalkParam[5].movetype = turn;
    visionWalkParam[5].turndata = -30;
    visionWalkParam[5].totalCount = 3000;

    robPose[5][5] = 1;
    robPose[5][0] = cos(M_PI * (-15) / 180.0);
    robPose[5][2] = sin(M_PI * (-15) / 180.0);
    robPose[5][8] = -sin(M_PI * (-15) / 180.0);
    robPose[5][10] = cos(M_PI * (-15) / 180.0);
    robPose[5][15] = 1;

    visionWalkParam[6].movetype = flatmove;
    double avoidMove3[3] = {0, 0, 0.4};
    memcpy(visionWalkParam[6].movedata, avoidMove3, 3*sizeof(double));
    visionWalkParam[6].totalCount = 3000;

    robPose[6][0] = 1;
    robPose[6][5] = 1;
    robPose[6][10] = 1;
    robPose[6][11] = 0.2;
    robPose[6][15] = 1;

    aris::server::GaitParamBase param;
    msg_out.copyStruct(param);
}

auto visionWalk(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase & plan_param)->int
{
    static bool isFirstTime = true;

    if (isMapRecorded)
    {
        if(stepNum == 7)
        {
            return 0;
        }

        if(isFirstTime)
        {
            visionWalkParam[stepNum].count = 0;
            isFirstTime = false;
        }

        auto &robot = static_cast<Robots::RobotBase &>(model);

        int remainCount = RobotVisionWalk(robot, visionWalkParam[stepNum]);
        visionWalkParam[stepNum].count++;

        if(remainCount == 0 && isStop == true)
        {
            isStop = false;
            isFirstTime = true;
            return 0;
        }

        if(remainCount == 0 && isStop == false)
        {
            stepNum++;
            isFirstTime = true;
            isSending = false;
            isMapRecorded = false;
            return -1;
        }
    }
    else
    {
        if(isSending)
        {
            return -1;
        }
        else
        {
            visionWalkPipe.sendToNrt(6);
            isSending = true;
            return -1;
        }
    }
}

auto stopVisionWalkParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    isStop = true;
}

int main(int argc, char *argv[])
{
    kinect2.Start();

    std::string xml_address;

    if (argc <= 1)
    {
        std::cout << "you did not type in robot name, in this case ROBOT-III will start" << std::endl;
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/Kinect2/Robot_VIII.xml";
    }
    else if (std::string(argv[1]) == "III")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml";
        xml_address = "/home/hex/Desktop/RobotIII/resource/Robot_III.xml";
    }
    else if (std::string(argv[1]) == "VIII")
    {
        //xml_address = "/usr/Robots/resource/Robot_Type_I/Robot_VIII/Robot_VIII.xml";
        xml_address = "/home/hex/Kinect2/Robot_VIII.xml";
    }
    else
    {
        throw std::runtime_error("invalid robot name, please type in III or VIII");
    }

    auto &rs = aris::server::ControlServer::instance();

    rs.createModel<Robots::RobotTypeI>();
    rs.loadXml(xml_address.c_str());
    rs.addCmd("en", Robots::basicParse, nullptr);
    rs.addCmd("ds", Robots::basicParse, nullptr);
    rs.addCmd("hm", Robots::basicParse, nullptr);
    rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
    rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
    rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);
    rs.addCmd("ca", visionCalibrateParse, visionCalibrate);
    rs.addCmd("vwk", visionWalkParse, visionWalk);
    rs.addCmd("swk", stopVisionWalkParse, visionWalk);

    rs.open();

    rs.setOnExit([&]()
    {
        aris::core::XmlDocument xml_doc;
        xml_doc.LoadFile(xml_address.c_str());
        auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
        if (!model_xml_ele)
            throw std::runtime_error("can't find Model element in xml file");
        rs.model().saveXml(*model_xml_ele);

        aris::core::stopMsgLoop();
    });

    aris::core::runMsgLoop();

    return 0;
}
