#ifndef VISION_TERRAIN_H
#define VISION_TERRAIN_H

#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;

enum TerrainType
{
    UnknownTerrain = 19,
    StepUpTerrain = 20,
    StepDownTerrain = 21,
    DitchTerrain = 22,
    FlatTerrain = 23,
    ObstacleTerrain = 24,
    BupTerrain = 25,
};

struct Terrain
{
    TerrainType terrainType;
    // position height length
    float terrainData[3];
};

class TerrainAnalysis
{
public:
    TerrainAnalysis(){}
    ~TerrainAnalysis(){}
    static int leftedge_z[6];
    static int rightedge_z[6];
    void TerrainAnalyze(const float oriGridMap[400][400]);
    void visionAdjust(double *param_Adjust, bool *adjust_Finished);
    Terrain terrain;
};

#endif // VISION_TERRAIN_H
