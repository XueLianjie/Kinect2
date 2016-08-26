#include "Vision_Terrain0.h"
#include <math.h>
#include <string.h>

namespace VisionWalk
{

double TerrainAnalysis::CurrentHeight[4] = {0, 0, 0, 0};
int TerrainAnalysis::leftedge_z[6] = {0, 0, 0, 0, 0, 0};
int TerrainAnalysis::rightedge_z[6] = {0, 0, 0, 0, 0, 0};
int TerrainAnalysis::leftedge_x[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int TerrainAnalysis::rightedge_x[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int TerrainAnalysis::Terrain = FlatTerrain;

int frame_num = 0;

void TerrainAnalysis::TerrainAnalyze(const float oriGridMap[400][400])
{
    float GridMap[400][400];
    memcpy(GridMap, oriGridMap, 400*400*sizeof(float));
    bool positive[6] = {false, false, false, false, false, false};
    bool negative[6] = {false, false, false, false, false, false};

    for (int p = 0; p < 6; p++)
    {
        leftedge_z[p] = 0;
        rightedge_z[p] = 0;
    }

    for (int q = 0; q < 10; q++)
    {
        leftedge_x[q] = 0;
        rightedge_x[q] = 0;
    }

    Terrain = FlatTerrain;

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

    // Jude Terrain
    bool* positive_pointer = positive;
    bool* negative_pointer = negative;

    for(int k = 275; k <= 355; k++)
    {
        if(GridMap[k + 2][200] - GridMap[k][200] > 0.05)
        {
            *positive_pointer = true;
            positive_pointer++;
        }

        if(GridMap[k + 2][200] - GridMap[k][200] < -0.05)
        {
            *negative_pointer = true;
            negative_pointer++;
        }
    }

    if(positive[0] == true&& negative[0] == false)
    {
        Terrain = StepUpTerrain;
    }
    if(positive[0] == false&& negative[0] == true)
    {
        Terrain = StepDownTerrain;
    }
    if(positive[0] == true&& negative[0] == true)
    {
        Terrain = DitchTerrain;
    }
    if(positive[0] == false&& negative[0] == false)
    {
        Terrain = FlatTerrain;
    }

    if(Terrain != FlatTerrain)
    {
        //Find Edge

        int* rightz_pointer = rightedge_z;
        int* leftz_pointer = leftedge_z;
        int* rightx_pointer = rightedge_x;
        int* leftx_pointer = leftedge_x;

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
    //Judge Command

    CurrentHeight[0] = (GridMap[297][165] + GridMap[297][166] + GridMap[298][165] + GridMap[298][166])/4;
    CurrentHeight[1] = (GridMap[297][155] + GridMap[297][156] + GridMap[298][155] + GridMap[298][156])/4;
    CurrentHeight[2] = (GridMap[297][234] + GridMap[297][235] + GridMap[298][234] + GridMap[298][235])/4;
    CurrentHeight[3] = (GridMap[297][245] + GridMap[297][246] + GridMap[298][245] + GridMap[298][246])/4;

//    std::stringstream out;
//    out<<frame_num;
//    std::string filename = "GridMap" + out.str() + ".txt";
//    std::ofstream Gridmapfile(filename);
//    if (Gridmapfile.is_open())
//    {
//        for(int i = 0; i < 120; i++)
//        {
//            for(int j = 0; j < 120; j++)
//            {
//                Gridmapfile<<GridMap[i][j]<<" ";
//            }
//            Gridmapfile<<endl;
//        }
//    }
//    Gridmapfile<<leftedge_z[0]<<" "<<rightedge_z[0];

    frame_num++;
}

}
