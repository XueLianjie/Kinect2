#include "Vision_Control0.h"
#include <iostream>
#include <string.h>
#include <math.h>

using namespace std;

namespace VisionWalk
{

void visionAdjust(double *param_Adjust, bool *adjust_Finished)
{
    if(abs(TerrainAnalysis::leftedge_z[0] - TerrainAnalysis::rightedge_z[0]) > 5)
    {
        cout<<"TURN!"<<endl;
        double turn_ang = atan2((TerrainAnalysis::rightedge_z[0] - TerrainAnalysis::leftedge_z[0]), 70);

        //max turn
        if(fabs(turn_ang) > 35 * M_PI / 180)
        {
            turn_ang = turn_ang > 0 ? 35 * M_PI / 180 : -35 * M_PI / 180;
        }
        cout<<"LEFT EDGE Z: "<< TerrainAnalysis::leftedge_z[0] - 200<<endl;
        cout<<"RIGHT EDGE Z: "<< TerrainAnalysis::rightedge_z[0] - 200<<endl;
        turn_ang = turn_ang * 180 / M_PI;
        cout<<"NEEDS TURN: "<<turn_ang<<endl;

        //let robot turn turn_ang
        param_Adjust[3] = turn_ang;
    }
    else
    {
        //Move Closer
        if((TerrainAnalysis::leftedge_z[0] < 280) || (TerrainAnalysis::leftedge_z[0] > 285))
        {
            cout<<"MOVE FORWARD AND BACKWARD!"<<endl;
            double movez_data[3] = {0, 0, 0};
            movez_data[2] = (TerrainAnalysis::leftedge_z[0] - 282) * 0.01;

            //max walk
            if(TerrainAnalysis::leftedge_z[0] < 280)
            {
                movez_data[2] = movez_data[2] < -0.325 ? -0.325 : movez_data[2];
            }
            else
            {
                movez_data[2] = movez_data[2] > 0.325 ? 0.325 : movez_data[2];
            }
            cout<<"LEFT EDGE Z: "<<TerrainAnalysis::leftedge_z[0] - 200<<endl;
            cout<<"RIGHT EDGE Z: "<<TerrainAnalysis::rightedge_z[0] - 200<<endl;
            cout<<"MOVE ALONG Z "<<movez_data[2]<<endl;

            //let robot move movez_data
            param_Adjust[2] = movez_data[2];
        }
        else
        {
            *adjust_Finished = true;
        }
    }
}

void visionStepUp(double *nextfoot_pos)
{
    cout<<"Step Up!!!"<<endl;

    static int StepUp_Num = -1;

    static double StepUp_Foot_Height[5][6] =
    {
        {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05},
        {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05},
        {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05},
        {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05},
        {-1.05, -1.05, -1.05, -1.05, -1.05, -1.05},
    };

    StepUp_Num++;

    for(int m = 0; m < 4; m++)
    {
        if(fabs(TerrainAnalysis::CurrentHeight[m] + 0.85) < 0.035 )
        {
            TerrainAnalysis::CurrentHeight[m] = -0.85;
        }

        if(TerrainAnalysis::CurrentHeight[m] > -0.85 )
        {
            TerrainAnalysis::CurrentHeight[m] = -0.85;
        }

        if(TerrainAnalysis::CurrentHeight[m] < -1.05 )
        {
            TerrainAnalysis::CurrentHeight[m] = -1.05;
        }
    }
    StepUp_Foot_Height[(4+StepUp_Num)%5][0] = TerrainAnalysis::CurrentHeight[1];
    StepUp_Foot_Height[(2+StepUp_Num)%5][1] = TerrainAnalysis::CurrentHeight[0];
    StepUp_Foot_Height[(0+StepUp_Num)%5][2] = TerrainAnalysis::CurrentHeight[1];
    StepUp_Foot_Height[(4+StepUp_Num)%5][3] = TerrainAnalysis::CurrentHeight[3];
    StepUp_Foot_Height[(2+StepUp_Num)%5][4] = TerrainAnalysis::CurrentHeight[2];
    StepUp_Foot_Height[(0+StepUp_Num)%5][5] = TerrainAnalysis::CurrentHeight[3];

    cout<<"FOOT HEIGHT"<<endl<<StepUp_Foot_Height[StepUp_Num%5][0]<<endl<<StepUp_Foot_Height[StepUp_Num%5][1]<<endl<<StepUp_Foot_Height[StepUp_Num%5][2]
            <<endl<<StepUp_Foot_Height[StepUp_Num%5][3]<<endl<<StepUp_Foot_Height[StepUp_Num%5][4]<<endl<<StepUp_Foot_Height[StepUp_Num%5][5]<<endl;

    memcpy(nextfoot_pos, StepUp_Foot_Height[StepUp_Num%5], sizeof(StepUp_Foot_Height[StepUp_Num%5]));

    nextfoot_pos[6] = double(StepUp_Num);

    cout<<"Step UP Num "<<StepUp_Num<<endl;

    //Reset StepUp
    if(StepUp_Num == 4)
    {
        StepUp_Num = -1;
        for(int i = 0; i < 5; i++)
        {
            for(int j = 0; j < 6; j++)
            {
                StepUp_Foot_Height[i][j] = -1.05;
            }
        }
        cout<<"Step Up Finished"<<endl;
    }
}

void visionStepDown(double *next_foot_pos)
{
    cout<<"Step Down!!!"<<endl;

    static int StepDown_Num = -1;
    static double StepDown_Foot_Height[5][6] =
    {
        {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85},
        {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85},
        {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85},
        {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85},
        {-0.85, -0.85, -0.85, -0.85, -0.85, -0.85},
    };

    StepDown_Num++;

    for(int m = 0; m < 4; m++)
    {
        if(fabs(TerrainAnalysis::CurrentHeight[m] + 1.05) < 0.035 )
        {
            TerrainAnalysis::CurrentHeight[m] = -1.05;
        }

        if(TerrainAnalysis::CurrentHeight[m] < -1.05 )
        {
            TerrainAnalysis::CurrentHeight[m] = -1.05;
        }

        if(TerrainAnalysis::CurrentHeight[m] > -0.85 )
        {
            TerrainAnalysis::CurrentHeight[m] = -0.85;
        }
    }

    StepDown_Foot_Height[(4+StepDown_Num)%5][0] = TerrainAnalysis::CurrentHeight[1];
    StepDown_Foot_Height[(2+StepDown_Num)%5][1] = TerrainAnalysis::CurrentHeight[0];
    StepDown_Foot_Height[(0+StepDown_Num)%5][2] = TerrainAnalysis::CurrentHeight[1];
    StepDown_Foot_Height[(4+StepDown_Num)%5][3] = TerrainAnalysis::CurrentHeight[3];
    StepDown_Foot_Height[(2+StepDown_Num)%5][4] = TerrainAnalysis::CurrentHeight[2];
    StepDown_Foot_Height[(0+StepDown_Num)%5][5] = TerrainAnalysis::CurrentHeight[3];

    cout<<"FOOT HEIGHT"<<endl<<StepDown_Foot_Height[StepDown_Num%5][0]<<endl<<StepDown_Foot_Height[StepDown_Num%5][1]<<endl<<StepDown_Foot_Height[StepDown_Num%5][2]
            <<endl<<StepDown_Foot_Height[StepDown_Num%5][3]<<endl<<StepDown_Foot_Height[StepDown_Num%5][4]<<endl<<StepDown_Foot_Height[StepDown_Num%5][5]<<endl;

    memcpy(next_foot_pos, StepDown_Foot_Height[StepDown_Num%5], sizeof(StepDown_Foot_Height[StepDown_Num%5]));

    next_foot_pos[6] = double(StepDown_Num);

    cout<<"Step Down Num "<<StepDown_Num<<endl;

    //Reset StepDown
    if(StepDown_Num == 5)
    {
        cout<<"Step Down Finished "<<endl;
        StepDown_Num = -1;
        for(int i = 0; i < 5; i++)
        {
            for(int j = 0; j < 6; j++)
            {
                StepDown_Foot_Height[i][j] = -0.85;
            }
        }
    }
}

void visionStepOver(double *step_over_data)
{
    cout<<"Step Over!!!"<<endl;

    static double counter = 0.0;

    double stepover_data[4] = {0, 0, 0, 0};
    stepover_data[0] = counter;
    if(int(stepover_data[0]) % 2 == 0)
    {
        stepover_data[3] = 0.6;
    }
    else
    {
        stepover_data[3] = 0.05;
    }
    cout<<"Count "<<stepover_data[0]<<endl;
    cout<<"X "<<stepover_data[1]<<endl;
    cout<<"Y "<<stepover_data[2]<<endl;
    cout<<"Z "<<stepover_data[3]<<endl;

    counter++;

    memcpy(step_over_data,stepover_data,4*sizeof(double));

    if(stepover_data[0] == 4)
    {
        cout<<"Step Over Finished "<<endl;
        counter = 0;
    }
}
}
