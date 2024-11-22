#include "kernels.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#define _USE_MATH_DEFINES
#include <math.h>

#define ITERATIONS 5000// 6885
#define N_RANGES 1081//2000

#define range_max 10.0
#define range_min 1.0

/*RANDOM GAUSSIAN DISTRIBUTION*/
float r4_normal_01 ( )
{
  float r1;
  float r2;
  const double r4_pi = 3.141592653589793;
  float distro_x;

  r1 = ( float ) drand48 ( );
  r2 = ( float ) drand48 ( );
  distro_x = sqrt ( - 2.0 * log ( r1 ) ) * cos ( 2.0 * r4_pi * r2 );

  return distro_x;
}
float r4_normal_ab ( float a, float b )
{
  float value;

  value = a + b * r4_normal_01 ( );

  return value;
}

void calculateRays(struct Particle_t *particles,
                                float *distMap,
                                struct Map_t *map,
                                struct Cloud_t *cloud,
                                int n_particles,
                                float *rays_angle)
{
    double angle, rayPoseX, rayPoseY, distance;
    for (int i = 0; i < n_particles; i++)
    {
        for (int j = 0; j < N_RAYS_DS; ++j)
        {
            float angle = (particles[i].yaw + cloud->angleMin) + rays_angle[j];
            rayPoseX = particles[i].x;
            rayPoseY = particles[i].y;
            float t = 0.0f;
            float out = cloud->maxRange;
            while (t < cloud->maxRayIteration)
            {
                int c = (int)((map->opp_originX - rayPoseX) / map->map_resolution);
                int r = (int)((map->opp_originY + rayPoseY) / map->map_resolution);

                if (c < 0 || c >= map->map_width || r < 0 || r > map->map_height)
                {
                    out = cloud->maxRange;
                    break;
                }

                distance = distMap[r * map->map_width + c];
                rayPoseX += distance * cos(angle);
                rayPoseY += distance * sin(angle);

                if (distance <= map->map_resolution)
                {
                    float xd = rayPoseX - particles[i].x;
                    float yd = rayPoseY - particles[i].y;
                    out = sqrtf(xd * xd + yd * yd);
                    break;
                }

                t += fmaxf(distance * 0.999f, 1.0);
            }

            particles[i].rays[j] = out;
        }
    }
}
/*======================================================================*/
/* Choose a weight for each particle from the sensor model              */
/* table according to the real and virtual rays                         */
void calculateWeights(struct Particle_t *particles,
                    float *obs,
                    float *sensor_model_table,
                    struct Map_t *map,
                    struct Cloud_t *cloud,
                    int table_width,
                    int n_particles)
/*======================================================================*/
{

    for (int i = 0; i < n_particles; i++)
    {
        double weight_temp = 1.0f;
        for (int j = 0; j < N_RAYS_DS; j++)
        { // TODO: substitute N_RAYS_DS with real number of rays

            float realRayPX = obs[j] / map->map_resolution;
           // realRayPX = std:min<float>(std::max<float>(realRayPX, 0.0), (map->MAX_RANGE_PX - 1.0));
            realRayPX = fminf(fmaxf(realRayPX, 0.0), (map->MAX_RANGE_PX - 1.0));


            float virtualRayPX = particles[i].rays[j] / map->map_resolution;

            // virtualRayPX =
            //     std::min<float>(std::max<float>(virtualRayPX, 0.0), (map->MAX_RANGE_PX - 1.0));
            virtualRayPX = fminf(fmaxf(virtualRayPX, 0.0), (map->MAX_RANGE_PX - 1.0));

            weight_temp *=
                (double)sensor_model_table[(int)virtualRayPX * table_width + (int)realRayPX];
            // weight_temp *=
            // (double)sensor_model_table[int(realRayPX)*table_width+int(virtualRayPX)];
        }

        particles[i].weight = weight_temp;
    }
}
/*============================================================================*/
/* Moves particles according to refined odometry data*/
void motion_model(float velocity, float dts, float yaw_rate)
/*============================================================================*/
{
    struct Particle_t valid_particles[MAX_PARTICLES];
    struct Particle_t tmp_particle;
    double motion_x, motion_y;
    float motion_yaw;
    int v_cnt = 0;

    for (int i = 0; i < MAX_PARTICLES; i++)
    {
        // Add measurements to each particle
        if (fabs(yaw_rate) < 0.001f)
        { // car going straight
            motion_x = particles[i].x + velocity * dts * cos(particles[i].yaw);
            motion_y = particles[i].y + velocity * dts * sin(particles[i].yaw);
            motion_yaw = particles[i].yaw;
        }
        else
        { // yaw rate is not equal to zero, car turning
            motion_x = particles[i].x +
                (velocity) / yaw_rate *
                    (sin(particles[i].yaw + yaw_rate * dts) - sin(particles[i].yaw));
            motion_y = particles[i].y +
                (velocity) / yaw_rate *
                    (cos(particles[i].yaw) - cos(particles[i].yaw + yaw_rate * dts));
            motion_yaw = particles[i].yaw + yaw_rate * dts;

            // remap to [-M_PI , M_PI]
            motion_yaw -= (int)((motion_yaw + FLOAT_PI) / (2.0f * FLOAT_PI)) * (2.0f * FLOAT_PI);
           // std::cout << yaw;
        }
        float std_x = std_x_mul + fabs(velocity * dts * std_v_mul);
        float std_y = std_y_mul;
        // RANDOM GAUSSIAN NOISE
        float x_noise = 0.0f;
        float y_noise = 0.0f;
        float yaw_noise = 0.0f;


        // std::normal_distribution<double> dist_x(x_noise, std_x);
        // std::normal_distribution<double> dist_y(y_noise, std_y);
        // std::normal_distribution<double> dist_yaw(yaw_noise,
        //                                          std_yaw + std::fabs(yaw_rate * dts * std_w_mul));

        float dist_x = r4_normal_ab(x_noise, std_x);
        float dist_y = r4_normal_ab(y_noise, std_y);
        float dist_yaw = r4_normal_ab(yaw_noise, std_yaw + fabs(yaw_rate * dts * std_w_mul));

        // x_noise = dist_x(generator);
        // y_noise = dist_y(generator);
        x_noise = dist_x;
        y_noise = dist_y;

        double tsin = sin(motion_yaw);
        double tcos = cos(motion_yaw);
        double rot_x = x_noise * tcos - y_noise * tsin;
        double rot_y = x_noise * tsin + y_noise * tcos;

        tmp_particle.x =motion_x + rot_x;
        tmp_particle.y =motion_y + rot_y;
        int r;
        int c = (int)((map.opp_originX - tmp_particle.x) / map.map_resolution);
        if (map.opp_originY >= 0)
        { // && abs(map.opp_originY) > abs(tmp_particle.y)){
            r = (int)((map.opp_originY + tmp_particle.y) / map.map_resolution);
        }
        else
        {
            int r = (int)((-map.opp_originY + tmp_particle.y) / map.map_resolution); // if is custom code to make sure r is positive. Case of y being negative and larger than
                                                                                     // opp_originY is still not fixed.
        }
        // float distance = distanceMap[r * map.map_width + c];
        //printf("MAP STRUCT:\nRANGE:%d\nop_X:%f\nop_Y: %f\nresolution: %f\nX: %f\nY: %f\nheight: %d\nwidth: %d\n",
        //map.MAX_RANGE_PX, map.opp_originX, map.opp_originY, map.map_resolution, map.map_originX, map.map_originY, map.map_height, map.map_width);
        //printf("d_map size: %d\nread at: %d",map.map_height * map.map_width, r*map.map_width + c);
        //printf("r=%d\nwidth=%d\nc=%d\n", r, map.map_width,c);
        //printf("r breakdown: r = [ op_origin_Y + particle Y ]/resolution: %d =(%f+%f)/%f",r,map.opp_originY,tmp_particle.y,map.map_resolution);
        if (distanceMap[r * map.map_width + c] >= map.map_resolution)
        {
            tmp_particle.yaw = motion_yaw + dist_yaw;

            valid_particles[v_cnt] = tmp_particle;
            v_cnt = v_cnt+1;
        }
    }
    n_valid_particles_ = v_cnt;
    *particles = *valid_particles;

    if ((!firstMM) && n_valid_particles_ < MAX_PARTICLES)
    {
        //    std::vector<Particle_t> new_particles = resample(MAX_PARTICLES - n_valid_particles_);
        //    particles.insert(particles.end(), new_particles.begin(), new_particles.end());
    }
    firstMM = 0;
}
/*====================================================================*/
/* Calculate Rays for each particle and Assign weights on the particles according to the rays*/
void sensor_model(float* obs, float* rays_angle)
{

    struct Particle_t *particles_d = &particles[0];
    calculateRays(
        particles_d, distanceMap, &map, &cloud, n_valid_particles_, &rays_angle[0]);

    calculateWeights(
        particles_d, &obs[0], &sensor_model_table[0], &map, &cloud, table_width, n_valid_particles_);

    for (int i = 0; i < n_valid_particles_; i++)
    {
        particles[i].weight = particles_d[i].weight;
        weights[i] = particles_d[i].weight;
    }
}

void normalize()
{
    double weightSum = 0;
    for (int i = 0; i < n_valid_particles_; i++)
    {
        weightSum += particles[i].weight;
    }

    for (int i = 0; i < n_valid_particles_; i++)
    {
        weights[i] /= weightSum;
        particles[i].weight /= weightSum;
    }
}

void custom_const_setup()
{
    /*Init Cloud*/
    cloud.maxRayIteration = 80;
    cloud.maxRange = 11.5;
    cloud.angleMin = -2.35619449615;
    cloud.angleMax = 2.35619449615;
    cloud.angleIncrement = 0.00436332309619;
    cloud.angleDownsample = 18;
    cloud.nAngle = (int)((fabs(cloud.angleMax) + fabs(cloud.angleMin)) / cloud.angleIncrement);
    /*Init Map*/
    // map.MAX_RANGE_PX = 200;
    // map.map_originX = 24.2592;
    // map.map_originY = 18.2029;
    // map.map_resolution = 0.05;
    // map.map_height = 299;
    // map.map_width = 409;
    // map.opp_originX = (map.map_width * map.map_resolution) + map.map_originX;
    // map.opp_originY = -map.map_originY;

    std_x_mul = 0.050;
    std_y_mul = 0.030;
    std_v_mul = 0.066;


    //    MAX_RANGE_METERS;

    //printf("cloud Initialized to:\n");
    //printf("maxRayIteration: %d\n",cloud.maxRayIteration);
    //printf("maxRange: %f\n", cloud.maxRange);
    //printf("angleMin: %f\n", cloud.angleMin);
    //printf("angleMax: %f\n", cloud.angleMax);
    //printf("angleIncrement : %f\n", cloud.angleIncrement);
    //printf("angleDownsample: %f\n", cloud.angleDownsample);
    //printf("nAngle: %d\n", cloud.nAngle);
    //printf("\n");
    //printf("Map Initialized to: \n");
    //printf("map.MAX_RANGE_PX: %d\n", map.MAX_RANGE_PX);
    //printf("map.map_originX: %f\n", map.map_originX);
    //printf("map.map_originY: %f\n", map.map_originY);
    //printf("map.map_resolution: %f\n", map.map_resolution);
    //printf("map.map_height: %d\n", map.map_height);
    //printf("map.map_width: %d\n", map.map_width);
    //printf("map.opp_originX: %f\n", map.opp_originY);
    //printf("map.opp_originY: %f\n", map.opp_originX);
}
void read_particles(){
    FILE *dat_particles;
    dat_particles = fopen("../particle_filter_proxy/dat_particles.txt","r");
    char line[1000];
    int p = 0;
    while (fgets(line, sizeof(line), dat_particles) != NULL && p<MAX_PARTICLES) {
        //printf("\nPARTICLE[%d]\n",p);
        sscanf(line, "%d %f %f %f %lf", &particles[p].id, &particles[p].x, &particles[p].y, &particles[p].yaw, &particles[p].weight);
        for (int i = 0; i < N_RAYS_DS; i++) {
            sscanf(line, "%f", &particles[p].rays[i]);
        }
        //printf("Integer: %d, Float 1: %f, Float 2: %f, Float 3: %f, Double Precision: %lf\n", 
        //        particles[p].id, particles[p].x, particles[p].y, particles[p].yaw, particles[p].weight);
        //printf("Floats: ");
        for (int i = 0; i < N_RAYS_DS; i++) {
            //printf("%f ", particles[p].rays[i]);
        }
        //printf("\n");
        p++;
    }
}
void read_distance_map(){
    int height = map.map_height;    //299
    int width = map.map_width;      //409
    char line[100000];
    float tmp;

    distanceMap = (float *)malloc(height * width * sizeof(float));

    FILE *dat_distance_map;
    dat_distance_map = fopen("../particle_filter_proxy/dat_distance_map.txt","r");

    int i=0;
    int j=0;
    while(fscanf(dat_distance_map, "%f", &tmp) != EOF && (i)<(width*height)){
        distanceMap[i] = tmp;
        //printf("\n%d: %f",i,distanceMap[i]);
        i++;       
    }
}
int read_map()
{
    FILE *dat_map = fopen("../particle_filter_proxy/dat_map.txt", "r");  // Open the file for reading
    if (dat_map == NULL) {
        printf("Failed to open file\n");
        return 1;
    }

    //struct Map_t map;

    // Use fscanf to read the data from the file into the struct
    fscanf(dat_map, "%d %f %f %f %f %f %d %d",
           &map.MAX_RANGE_PX, &map.opp_originX, &map.opp_originY,
           &map.map_resolution, &map.map_originX, &map.map_originY,
           &map.map_height, &map.map_width);

    // Close the file
    fclose(dat_map);

    // printf("MAX_RANGE_PX: %d\n", map.MAX_RANGE_PX);
    // printf("opp_originX: %f\n", map.opp_originX);
    // printf("opp_originY: %f\n", map.opp_originY);
    // printf("map_resolution: %f\n", map.map_resolution);
    // printf("map_originX: %f\n", map.map_originX);
    // printf("map_originY: %f\n", map.map_originY);
    // printf("map_height: %d\n", map.map_height);
    // printf("map_width: %d\n", map.map_width);

    return 0;
}
void read_sensor_model_table(){
    int table_width = map.MAX_RANGE_PX+1;
    float tmp;
    //float *sensor_model_table;
    sensor_model_table = (float *)malloc(table_width * table_width * sizeof(float));

    FILE *dat_sensor_model_table;
    dat_sensor_model_table = fopen("../particle_filter_proxy/dat_sensor_model_table.txt","r");

    int i=0;
    int j=0;
    while(fscanf(dat_sensor_model_table, "%f", &tmp) != EOF && (i)<(table_width*table_width)){
        sensor_model_table[i] = tmp;
       // printf("\n%d: %f",i,sensor_model_table[i]);
        i++;       
    }
}
/*========================================================================================*/
// struct Particle_t* resample(unsigned int n_particles, struct Particle_t *new_particles)
void resample(unsigned int n_particles, struct Particle_t *new_particles)
/*This function seems to be shuffling the particles */
/*========================================================================================*/
{

    // struct Particle_t *new_particles;
    // new_particles = (struct Particle_t*)malloc(sizeof(struct Particle_t)*n_particles);

    float U, r;
    float c = particles[0].weight;
    int i = 1;
    int j = 0;

    r = r4_normal_ab ( 0, 1.0f / (float)n_particles );
    for (int m = 0; m < n_particles; m++) {
        U = r + (float)(m - 1) * 1.0f / (float)n_particles; // (rand + index-1)/1000
        while (U > c) {
            i = (i + 1) % n_particles;
            c += particles[i].weight;
        }
        new_particles[j]=(particles[i]);
        j++;
    }
   // return new_particles;
}

float* expected_pose()
{

    float pose[3*n_valid_particles_];
    float w[n_valid_particles_];

    for (int i = 0; i < n_valid_particles_; i++) {
        pose[i + n_valid_particles_ * 0] = particles[i].x;
        pose[i + n_valid_particles_ * 1] = particles[i].y;
        pose[i + n_valid_particles_ * 2] = particles[i].yaw;
        w[i] = particles[i].weight;
    }

    float expected[3] = {0,0,0};
    for(int i = 0; i < n_valid_particles_; i++){
        expected[0] += pose[i + n_valid_particles_ * 0] * w[i];
        expected[1] += pose[i + n_valid_particles_ * 1] * w[i];
        expected[2] += pose[i + n_valid_particles_ * 2] * w[i];

    }
    current_pose[0] = expected[0];
    current_pose[1] = expected[1];
    current_pose[2] = expected[2];

    return current_pose;
}

void read_model_inputs(){

    int obs_size = 60;
    int rays_angle_size = 60;

    int i;
    // Open binary file in input mode
    FILE *dat_odom_v;
    FILE *dat_dts;
    FILE *dat_w;
    FILE *dat_scan;
    FILE *dat_rays_angles;
    
    /* Read odom_v*/
    dat_odom_v = fopen("../particle_filter_proxy/dat_odom_v.txt","r");
    if(fscanf(dat_odom_v, "%f", &velocity) != 1);
    //printf("\nvel: %f\n", velocity);
    fclose(dat_odom_v);

    /* Read dts*/
    dat_dts = fopen("../particle_filter_proxy/dat_dts.txt","r");
    if(fscanf(dat_dts, "%f", &dts) != 1);
    //printf("\ndts: %f\n", dts);
    fclose(dat_dts);

    /* Read yaw_rate*/
    dat_w = fopen("../particle_filter_proxy/dat_w.txt","r");
    if(fscanf(dat_w, "%f", &yaw_rate) != 1);
    //printf("\nyaw: %f\n", yaw_rate);
    fclose(dat_w);

    /*Read obs*/
    i=0;
    dat_scan= fopen("../particle_filter_proxy/dat_scan.txt","r");
    while (fscanf(dat_scan, "%f", &obs[i]) != EOF && i<60){
        i++;
    }
    for(i=0; i<60;i++){
        //printf("obs[%d]: %f\n",i,obs[i]);
    }
    fclose(dat_scan);   

    /*Read ray angles*/
    i=0;
    dat_rays_angles= fopen("../particle_filter_proxy/dat_rays_angles.txt","r");
    while (fscanf(dat_rays_angles, "%f", &rays_angle[i]) == 1 && i < 60) {
        i++;
    }
    for(i=0; i<60;i++){
        //printf("rays_angle[%d]: %f\n",i,rays_angle[i]);
    }
    fclose(dat_rays_angles);    
}



void read_natives_from_files()
{
    // int height, width;
    // height = map.map_height;
    // width = map.map_width;

    printf("Reading Constant Vars and Model Inputs");

    /*Read files*/
    read_model_inputs();
    read_map();
    read_particles();
    read_distance_map();
    read_sensor_model_table();
}
/*  The following 2 extraction functions should be changed to 
    be syncronized with each other. RN they draw from the dataset
    according to the iteration index
*/
struct Odom_t extract_odom_msg(int i)
{
    struct Odom_t odom;
    odom.sec = odom_sec[i];
    odom.nsec = odom_nsec[i];
    odom.o_x = o_x[i];
    odom.o_y = o_y[i];
    odom.o_z = o_z[i];
    odom.o_w = o_w[i];
    odom.pose_x = pose_x[i];
    odom.pose_y = 0;
    odom.twist_x = ang_z[i];
    return odom;
}
struct Lidar_t extract_lidar_msg(int i)
{
    struct Lidar_t lidar;
    lidar.sec = lidar_sec[i];
    lidar.nsec = lidar_nsec[i];
    // lidar.ranges = (float)malloc((N_RANGES-719)*sizeof(float));
    for(int ii=0; ii<N_RANGES; ii++) //N_RANGES = 2000 but actual is 1081
        lidar.ranges[ii] = lidar_ranges[i*(N_RANGES)+ii];
    return lidar;
}
/*ODOM CB Reads the Odometry Message*/
/*========================================================================================*/
void OdomCb(struct Odom_t msg)
/*========================================================================================*/
{
    double roll, pitch, yaw;


    // tf2::Quaternion q(msg->pose.pose.orientation.x,
    //                   msg->pose.pose.orientation.y,
    //                   msg->pose.pose.orientation.z,
    //                   msg->pose.pose.orientation.w);
    // float msg.o_x;
    // float msg.o_y;
    // float msg.o_z;
    // float msg.o_w;
    // float msg.pose_x;
    // float msg.pose_y;
    // float msg.twist_x;

    // R(q) = [ 1 - 2y^2 - 2z^2    2xy - 2zw        2xz + 2yw     ]
    //        [ 2xy + 2zw        1 - 2x^2 - 2z^2    2yz - 2xw     ]
    //        [ 2xz - 2yw        2yz + 2xw        1 - 2x^2 - 2y^2 ]

    /*Construct the columns of the rotation matrix*/
    float R[9];

    R[0] = 1- 2*pow(msg.o_y,2) - 2*pow(msg.o_z,2);
    R[1] = 2*msg.o_x * msg.o_y - 2*msg.o_z * msg.o_w;
    R[2] = 2*msg.o_x * msg.o_z + 2*msg.o_y * msg.o_w;
    
    R[3] = 2*msg.o_x * msg.o_y + 2*msg.o_z * msg.o_w;
    R[4] = 1 - 2*pow(msg.o_x,2)- 2*pow(msg.o_z,2);
    R[5] = 2*msg.o_y * msg.o_z - 2*msg.o_x * msg.o_w;
    
    R[6] = 2*msg.o_x * msg.o_z - 2*msg.o_y * msg.o_w;
    R[7] = 2*msg.o_y * msg.o_z + 2*msg.o_x * msg.o_w;
    R[8] = 1 - 2*pow(msg.o_x,2)- 2*pow(msg.o_y,2);

    // tf2::Matrix3x3 m(q);
    // m.getRPY(roll, pitch, yaw);


    pitch = atan2(-R[2*3+0], sqrt(R[0*3+0]*R[0*3+0] + R[1*3+0]*R[1*3+0]));

    // Check for gimbal lock
    if (fabs(pitch - M_PI/2) < 1e-6 || fabs(pitch + M_PI/2) < 1e-6) {
        yaw = 0;
        roll = atan2(R[1*3+2], R[0*3+2]);
    } else {
        // Extract roll and yaw angles
        roll = atan2(R[2*3+1]/cos(pitch), R[2*3+2]/cos(pitch));
        yaw = atan2(R[1*3+0]/cos(pitch), R[0*3+0]/cos(pitch));
    }

    odom_x = msg.pose_x;
    odom_y = msg.pose_y;
    odom_v = msg.twist_x;
    if (first_odom) {
        oldYaw = yaw;
        oldX = odom_x;
        oldY = odom_y;
        last_stamp = msg.sec + 1e-9 * msg.nsec;
        first_odom = 0;
        return;
    }

    dts = (msg.sec + 1e-9 * msg.nsec - last_stamp);
    float w = (yaw - oldYaw) / dts;
    float x = (odom_x - oldX);
    float y = (odom_y - oldY);
    velocity = (sqrt(pow(x, 2) + pow(y, 2))) / dts;

   // last_stamp = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    oldYaw = yaw;
    oldX = odom_x;
    oldY = odom_y;
    oldDts = dts;

    //odom_initialized = true;
    // update();
}
/*LIDAR CB is the function that reads the lidar sensor */
/*========================================================================================*/
void lidarCb(float * msg_ranges)//(const sensor_msgs::msg::LaserScan::SharedPtr msg)
/*========================================================================================*/
{
    // if (!map_initialized || !take_pose)
    //     return;
    // printf("MSG_RANGES[0]: %f\n",msg_ranges[0]);
    // sensor_msgs::msg::LaserScan downsampled_msg = *msg;
    // std::vector<float> ranges;
    // float * ranges;
    // void *new_ranges;
    // ranges = (float*)malloc(N_RANGES * sizeof(float));
    //  ranges = (float*)malloc(sizeof(float));
    int valid_rays = 0;
    for (int i = 0; i < N_RANGES; i++) {
        if (msg_ranges[i] < range_max && msg_ranges[i] > range_min)
            valid_rays++;
    }

    // nearest neighbor without multiplications or divisions
    int i = 0, j = 0, k=0, tmp = valid_rays;
    for (int i = 0; i < N_RANGES; ++i) {
        tmp += N_RAYS_DS - 1;
        if (tmp >= valid_rays) {
            tmp -= valid_rays;
            //skip invalid rays
            while (msg_ranges[i] > range_max || msg_ranges[i] < range_min) {
                ranges[k] = 0.0;
                k++;
                // printf("\ni:%d\nj:%d\nk:%d\n",i,j,k);
                // if(new_ranges = realloc(ranges, (k+16)*sizeof(float)))
                //     ranges = new_ranges;
                // else
                //     printf("OOM!\n");
                i++;
            }
            rays_angle[j] = i * ANGLE_STEP;
            obs[j] = msg_ranges[i];
            
            ranges[k] = obs[j];
            k++;
            j++;
        } else {
            ranges[k] = 0.0;
            k++;
        }
    }

    while (j < N_RAYS_DS) {
        obs[j++] = 0.0f;
    }
    //end = std::chrono::steady_clock::now();
    // std::cout << "downsample time: " <<
    // std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() << "us" <<
    // std::endl;

    // lidar_initialized = true;
   // update();

    //end = std::chrono::steady_clock::now();

    /* TAKE RANGES FROM NEW MESSAGE*/
    //////////////////////////////////////
    //downsampled_msg.ranges = ranges;////
    //////////////////////////////////////


    //lidar_pub->publish(downsampled_msg);
    // std::cout << "loop time: " <<
    // std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() << "us" <<
    // std::endl;
}

// void publish_tf(){
//     geometry_msgs::msg::TransformStamped ts;

//     ts.header.stamp = timestamp;
//     ts.header.frame_id = "lidar_link";
//     ts.child_frame_id = "map";

//     ts.transform.translation.x = poses[0];
//     ts.transform.translation.y = poses[1];
//     ts.transform.translation.z = 0.0;

//     tf2::Quaternion q = createQuaternionFromYaw(poses[2]);
//     ts.transform.rotation.x = q.x();
//     ts.transform.rotation.y = q.y();
//     ts.transform.rotation.z = q.z();
//     ts.transform.rotation.w = q.w();

//     tf2::Transform tf;
//     tf2::fromMsg(ts.transform, tf);
//     ts.transform = tf2::toMsg(tf.inverse());

//     tf_broadcaster->sendTransform(ts);
// }

int old_main()
{
    printf("Intialization:");
    struct timeval start, end;
    double elapsed_init, elapsed_motion, elapsed_sensor, elapsed_norm;

    gettimeofday(&start, NULL);   

    custom_const_setup();
    read_natives_from_files();

    gettimeofday(&end, NULL);   
    elapsed_init = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.0;

    printf("\nMotion Model:\n");
    gettimeofday(&start, NULL);   
    motion_model(velocity, dts, yaw_rate);
    gettimeofday(&end, NULL);
    elapsed_motion = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.0;   

    printf("\nSensor Model:\n");
    gettimeofday(&start, NULL);   
    sensor_model(obs, rays_angle);
    gettimeofday(&end, NULL);
    elapsed_sensor = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.0;  

    printf("\nNormalization:\n");
    gettimeofday(&start, NULL);   
    normalize();
    gettimeofday(&end, NULL);
    elapsed_norm = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.0;  

    printf("\nTIMINGS:\n");
    printf("Initialization  :%f ms\n", elapsed_init*1000);
    printf("Motion Model    :%f ms\n", elapsed_motion*1000);
    printf("Sensor Model    :%f ms\n", elapsed_sensor*1000);
    printf("Normalization   :%f ms\n", elapsed_norm*1000);
    printf("=========================\n");
    printf("Total           :%f ms\n", (elapsed_init + elapsed_motion + elapsed_norm + elapsed_sensor)*1000);
    printf("Processing      :%f ms\n", (elapsed_motion + elapsed_sensor + elapsed_norm)*1000);
}

int main()
{
    printf("Intialization:");

    struct Particle_t new_particles[MAX_PARTICLES];
    float* poses;
    int firstMCL = 1;
    struct timeval start, end;
    double elapsed_init, elapsed_preproc, elapsed_motion, elapsed_sensor, elapsed_norm;

    gettimeofday(&start, NULL);   

    /* INITIALIZATION */
    
    read_natives_from_files();
    
    gettimeofday(&end, NULL);   
    elapsed_init = (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.0;

    struct Odom_t odom;
    struct Lidar_t lidar;
    /* LOOP ITERATIONS */
    for (int i=0; i<ITERATIONS; i++){
        if (!firstMCL){
            // printf("resample:\n");
            // new_particles = resample(MAX_PARTICLES, new_particles);
            resample(MAX_PARTICLES, new_particles);
        }
        /*READ & PREPROC SENSOR SAMPLES*/
        
        // printf("\nExtract & Preprocess:\n");
        gettimeofday(&start, NULL);
        odom = extract_odom_msg(i);
        lidar = extract_lidar_msg(i);
        
        OdomCb(odom);
        lidarCb(lidar.ranges);
        gettimeofday(&end, NULL);
        elapsed_preproc += (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.0;   

        // printf("\nMotion Model:\n");
        gettimeofday(&start, NULL);   
        motion_model(velocity, dts, yaw_rate);
        gettimeofday(&end, NULL);
        elapsed_motion += (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.0;   

        // printf("\nSensor Model:\n");
        gettimeofday(&start, NULL);   
        sensor_model(obs, rays_angle);
        gettimeofday(&end, NULL);
        elapsed_sensor += (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.0;  

        // printf("\nNormalization:\n");
        gettimeofday(&start, NULL);   
        normalize();
        gettimeofday(&end, NULL);
        elapsed_norm += (end.tv_sec - start.tv_sec) + (end.tv_usec - start.tv_usec) / 1000000.0;  
    
        float* poses = expected_pose();
        firstMCL = 0;
    }
    printf("\nTIMINGS:\n");
    printf("Preproc         :%f ms\n", elapsed_preproc*1000);
    printf("Initialization  :%f ms\n", elapsed_init*1000);
    printf("Motion Model    :%f ms\n", elapsed_motion*1000);
    printf("Sensor Model    :%f ms\n", elapsed_sensor*1000);
    printf("Normalization   :%f ms\n", elapsed_norm*1000);
    printf("=========================\n");
    printf("Total           :%f ms\n", (elapsed_init + elapsed_motion + elapsed_norm + elapsed_sensor)*1000);
    printf("Processing      :%f ms\n", (elapsed_motion + elapsed_sensor + elapsed_norm)*1000);
}
