#pragma once
#include <stdint.h>
#define N_RAYS_DS (60)

struct Particle_t
{
    int id;
    float x, y;
    float yaw;
    double weight;
    float rays[N_RAYS_DS];
};

struct Map_t
{
    int MAX_RANGE_PX;
    float opp_originX;
    float opp_originY;
    float map_resolution;
    float map_originX;
    float map_originY;
    int map_height;
    int map_width;
};

struct Cloud_t
{
    int maxRayIteration;
    float maxRange;
    float angleMin;
    float angleMax;
    float angleIncrement;
    float angleDownsample;
    int nAngle;
};

struct Odom_t
{
    double o_x;
    double o_y;
    double o_z;
    double o_w;
    double pose_x;
    double pose_y;
    double twist_x;
    int32_t sec;
    int32_t nsec; 
};

struct Lidar_t
{
    int32_t sec;
    int32_t nsec; 
    float ranges[1081];
};

//Unused
struct Sensor_pair_t
{
    struct Lidar_t lidar;
    struct Odom_t odom;
};