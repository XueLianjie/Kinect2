#include "Vision_Terrain.h"
#include <math.h>
#include <string.h>

int TerrainAnalysis::leftedge_z[6] = {0, 0, 0, 0, 0, 0};
int TerrainAnalysis::rightedge_z[6] = {0, 0, 0, 0, 0, 0};

void TerrainAnalysis::TerrainAnalyze(const float oriGridMap[400][400])
{
    float GridMap[400][400];
    memcpy(GridMap, oriGridMap, 400 * 400 * sizeof(float));

    for (int p = 0; p < 6; p++)
    {
        leftedge_z[p] = 0;
        rightedge_z[p] = 0;
    }

    terrain.terrainType = FlatTerrain;

    //Judge Terrain
    for(int k = 275; k <= 355; k++)
    {
        // fill in nan data along z middle 200
        int r = 2;
        while(GridMap[k + 1][200] == 0 && r < 25)
        {
            GridMap[k + 1][200] = GridMap[k + r][200];
            r++;
        }

        //fill in nan data along z right
        int p = 2;
        while(GridMap[k + 1][200 - 35] == 0 && p < 25)
        {
            GridMap[k + 1][200 - 35] = GridMap[k + p][200 - 35];
            p++;
        }

        //fill in nan data along z left
        int q = 2;
        while(GridMap[k + 1][200 + 35] == 0 && q < 25)
        {
            GridMap[k + 1][200 + 35] = GridMap[k + q][200 + 35];
            q++;
        }
    }

    int i = 0;
    int j = 0;

    int positive[2][6] = {0};
    int negative[2][6] = {0};

    for(int k = 275; k <= 355; k++)
    {
        if(GridMap[k + 2][200] - GridMap[k][200] > 0.05)
        {
            positive[0][i] = 1;
            positive[1][i] = k;
            i++;
        }

        if(GridMap[k + 2][200] - GridMap[k][200] < -0.05)
        {
            negative[0][j] = 1;
            negative[1][j] = k;
            j++;
        }
    }

    if(positive[0][0] == 1 && negative[0][0] == 0)
    {
        terrain.terrainType = StepUpTerrain;
        terrain.terrainData[0] = (positive[1][0] - 265) / 100.0;
        terrain.terrainData[1] = GridMap[positive[1][0] + 7][200] - GridMap[positive[1][0]][200];
        terrain.terrainData[2] = 1;
    }
    if(positive[0][0] == 0 && negative[0][0] == 1)
    {
        terrain.terrainType = StepDownTerrain;
        terrain.terrainData[0] = (negative[1][0] - 265) / 100.0;
        terrain.terrainData[1] = -(GridMap[negative[1][0] + 7][200] - GridMap[negative[1][0]][200]);
        terrain.terrainData[2] = 1;
    }
    if(positive[0][0] == 1 && negative[0][0] == 1)
    {
        if(positive[1][0] < negative[1][0])
        {
            terrain.terrainType = DitchTerrain;

            float depth = 0;
            for(int i = negative[1][0] + 1; i < positive[1][0]; i++)
            {
                if(GridMap[i][200] < depth)
                {
                    depth = GridMap[i][200];
                }
            }

            terrain.terrainData[0] = (negative[1][0] - 265) / 100.0;
            terrain.terrainData[1] = -(depth - GridMap[negative[1][0]][200]);
            terrain.terrainData[2] = (positive[1][0] - negative[1][0]) / 100.0;
        }
        else
        {
            terrain.terrainType = BupTerrain;

            float height = -2;
            for(int i = positive[1][0] + 1; i < negative[1][0]; i++)
            {
                if(GridMap[i][200] > height)
                {
                    height = GridMap[i][200];
                }
            }
            terrain.terrainData[0] = (positive[1][0] - 265) / 100.0;
            terrain.terrainData[1] = height - GridMap[positive[1][0]][200];
            terrain.terrainData[2] = (negative[1][0] - positive[1][0]) / 100.0;
        }
    }
    if(positive[0][0] == 0 && negative[0][0] == 0)
    {
        terrain.terrainType = FlatTerrain;
    }

    if(terrain.terrainType != FlatTerrain)
    {
        //Find Edge
        int* rightz_pointer = rightedge_z;
        int* leftz_pointer = leftedge_z;

        //Find Edge Along Z
        for(int m = 275; m <= 355; m++)
        {
            if(fabs(GridMap[m + 2][165] - GridMap[m][165]) > 0.05)
            {
                *rightz_pointer = m + 1;
                rightz_pointer++;
            }

            if(fabs(GridMap[m + 2][235] - GridMap[m][235]) > 0.05)
            {
                *leftz_pointer = m + 1;
                leftz_pointer++;
            }
        }
    }

    cout<<leftedge_z[0] - 200<<" "<<rightedge_z[0] - 200<<endl;
}

void TerrainAnalysis::visionAdjust(double *param_Adjust, bool *adjust_Finished)
{
    if(abs(leftedge_z[0] - rightedge_z[0]) > 5)
    {
        cout<<"TURN!"<<endl;
        double turn_ang = atan2((rightedge_z[0] - leftedge_z[0]), 70);

        //max turn
        if(fabs(turn_ang) > 35 * M_PI / 180)
        {
            turn_ang = turn_ang > 0 ? 35 * M_PI / 180 : -35 * M_PI / 180;
        }
        cout<<"LEFT EDGE Z: "<< leftedge_z[0] - 200<<endl;
        cout<<"RIGHT EDGE Z: "<< rightedge_z[0] - 200<<endl;
        turn_ang = turn_ang * 180 / M_PI;
        cout<<"NEEDS TURN: "<<turn_ang<<endl;

        //let robot turn turn_ang
        param_Adjust[3] = turn_ang;
    }
    else
    {
        //Move Closer
        if((leftedge_z[0] < 280) || (leftedge_z[0] > 285))
        {
            cout<<"MOVE FORWARD AND BACKWARD!"<<endl;
            double movez_data[3] = {0, 0, 0};
            movez_data[2] = (leftedge_z[0] - 282) * 0.01;

            //max walk
            if(leftedge_z[0] < 280)
            {
                movez_data[2] = movez_data[2] < -0.325 ? -0.325 : movez_data[2];
            }
            else
            {
                movez_data[2] = movez_data[2] > 0.325 ? 0.325 : movez_data[2];
            }
            cout<<"LEFT EDGE Z: "<<leftedge_z[0] - 200<<endl;
            cout<<"RIGHT EDGE Z: "<<rightedge_z[0] - 200<<endl;
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
