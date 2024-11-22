/* Fusion and cleanup of ray_marching.hpp
and particle_filter.hpp
*/

//#pragma once
#include "data_structures.h"
//#include "ray_marching.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "values.h"

#include "lidar_msg.h"
#include "odom_sync_msg.h"


#define WIDTH 485
#define HEIGHT 379
#define N_RAYS 60
#define N_PARTICLES 1081// 2000

#define FLOAT_PI 3.14159265359
#define MAX_PARTICLES 100

#define ANGLE_STEP (0.00435928675336234f)
// struct Particle_t
// {
//     int id;
//     float x, y;
//     float yaw;
//     double weight;
//     float rays[N_RAYS_DS];
// };

// struct Map_t
// {
//     int MAX_RANGE_PX;
//     float opp_originX;
//     float opp_originY;
//     float map_resolution;
//     float map_originX;
//     float map_originY;
//     int map_height;
//     int map_width;
// };

// struct Cloud_t
// {
//     int maxRayIteration;
//     float maxRange;
//     float angleMin;
//     float angleMax;
//     float angleIncrement;
//     float angleDownsample;
//     int nAngle;
// };


static double weights[100];
static int count = 0;
//static float x[N_PARTICLES];
//static float y[N_PARTICLES];
//static float yaw[N_PARTICLES];
static float rays[N_PARTICLES * N_RAYS];
static float rays_total[N_PARTICLES * N_RAYS];
static float rays_ds[N_PARTICLES * N_RAYS_DS];
static float distanceMap_d[WIDTH * HEIGHT];
static float orig_x;
static float orig_y;
static float map_resolution;
static int map_height;
static int map_width;
static float angleMin;
static float angleIncrement;
static float maxRange;
static int particles_d;
static int fd;
static int loadMapFPGA;
static int first;

//static int MAX_PARTICLES = 100;

//static float velocity;
//static float dts;
static float yaw_rate;
static float obs[60];
static float rays_angle[60];
static float current_pose[3];

static float ranges[N_PARTICLES];

void compute_weights(struct Particle_t* particles,
                         float* obs,
                         float* distMap,
                         float* sensor_model_table,
                         struct Map_t* map,
                         struct Cloud_t* cloud,
                         int table_width,
                         int n_particles);

void calculateRays(struct Particle_t* particles,
                       float* distMap,
                       struct Map_t* map,
                       struct Cloud_t* cloud,
                       int n_particles,
                        float* rays_angle);
  
void calculateWeights(struct Particle_t* particles,
                          float* obs,
                          float* sensor_model_table,
                          struct Map_t* map,
                          struct Cloud_t* cloud,
                          int table_width,
                          int n_particles);

// calculateWeightsOMP(struct Particle_t* particles,
//                              float* obs,
//                              float* sensor_model_table,
//                              struct Map_t map,
//                              struct Cloud_t cloud,
//                              int table_width,
//                              int n_particles);

static struct Map_t map;
static float* distanceMap;
// init
//std::vector<double> weights;
//std::default_random_engine generator;

// odomCb
static float odom_x;
static float odom_y;
static float odom_v;
static float oldDts = 0.0f;
static float oldYaw = 0.0f;
static float oldX = 0.0f;
static float oldY = 0.0f;
static float dts = 0.0f;
static float w = 0.0f;
static float x = 0.0f;
static float y = 0.0f;
static float velocity;
static double last_stamp;
static int first_odom = 1;
// motion model
static int firstMM;
static float std_w_mul;
static float std_yaw;
static float std_x_mul;
static float std_y_mul;
static float std_v_mul;

// sensor model table
static int table_width = 60;
static float *sensor_model_table;//[table_width*table_width];

// sensor model
static struct Cloud_t cloud;


// debug
//static int count = 0;
//std::ofstream myfile;
//RayMarching rayMarching;
//clock_t tStart, tEnd;
//double elapsed;

// utils
static int n_valid_particles_;
static int useFPGA;
static int useGPU;
//static bool wayToSort(Particle_t i, Particle_t j);
//std::vector<float> expected_pose;


//static int MAX_PARTICLES;
static struct Particle_t particles[MAX_PARTICLES];
//static struct Particle_t particles[100];

void motion_model(float velocity, float dts, float yaw_rate);
void sensor_model(float* obs, float* rays_angle);

void normalize();

void custom_const_setup();
void read_particles();
void read_natives_from_files();

