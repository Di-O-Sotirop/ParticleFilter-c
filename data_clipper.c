#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "values.h"
#include "lidar_msg.h"
#include "odom_msg.h"



int32_t time_diff(int i, int j){
    int32_t diff;
     if(lidar_sec[i] == odom_sec[j]){
        diff = abs(lidar_nsec[i]-odom_nsec[j]);
     }else{
        printf("!");
        if(lidar_sec[i] - odom_sec[j] > 0){ //lidar count is after odom 
            diff = abs(1000000000 + lidar_nsec[i] - odom_nsec[j]);
        } else {
            diff = abs(1000000000 + odom_nsec[i] - lidar_nsec[j]);
        }
        
     }
     return diff;
}

int automato(int prev,int cnt){ //checks if the process of syncronizing the 2 sensors follows the pattern 0-0-0-1 skips.
    if(prev < 3){
        if (cnt == 0){
            printf(".");
            return prev+1;
        }else{
            printf("x");
            return 0;
        }
    }else{
        if (cnt == 1)
            printf("v");
        else
            printf("x");
        return 0;
    }
}

void clipper()
{
    int odom_len = sizeof(odom_sec) / sizeof(odom_sec[0]);
    int lidar_len = sizeof(lidar_sec) / sizeof(lidar_sec[0]);
    int32_t current_diff, next_diff;
    int cnt=0;
    int state=0;
    int state_old=0;
    printf("Odom: %d\nLidar: %d\n", odom_len, lidar_len);
    //odom 8621 - lidar 6885
    int j=0;
    /*
    for(int i=0; i<lidar_len-1; i++){
        printf("%d\t: %d - %d :: %d - %d\n",i,lidar_sec[i+1]-lidar_sec[i], lidar_nsec[i+1]-lidar_nsec[i], odom_sec[i+1]-odom_sec[i], odom_nsec[i+1]-odom_nsec[i]);

    }
    */
    for (int i=0; i<lidar_len; i++){
        cnt = 0;
        current_diff = time_diff(i,j);
        next_diff = current_diff;
        // printf("%d,%d: %d\n", i,j,current_diff);
        while(current_diff >= next_diff){
            // current_diff = next_diff;
            j++;
            next_diff = time_diff(i,j);
            // printf("\t%d : %d\n",j, next_diff);
            cnt++;
        }
        current_diff = time_diff(i,j-1);
        // printf("\t>%d,%d: d=%d\n",i,j-1,current_diff);
        // if(cnt > 1)
        // printf("skipped: %d\n",cnt-1);
        state = automato(state, cnt-1);
        // state_old = state;
    }
}


/*==============WRITE FUNCTION=============================*/
/*  This function creates the header files that contain the
    syncronized data by parsing the full data several times.
    This is so that the data do not have to be saved in arrays
    before writing them. I could move the delimeter around in
    the output file to achieve this more efficiently but this
    is a one time case so no sweat.
 */
int write_int32_t_data(FILE *fp, int32_t *arr){
    int odom_len = sizeof(odom_sec) / sizeof(odom_sec[0]);
    int lidar_len = sizeof(lidar_sec) / sizeof(lidar_sec[0]);
    int32_t current_diff, next_diff;
    int cnt=0;
    printf("Odom: %d\nLidar: %d\n", odom_len, lidar_len);
    int j=0;
    for (int i=0; i<lidar_len; i++){
        cnt = 0;
        current_diff = time_diff(i,j);
        next_diff = current_diff;
        while(current_diff >= next_diff){
            j++;
            next_diff = time_diff(i,j);
            cnt++;
        }
        current_diff = time_diff(i,j-1);
        fprintf(fp, "%u,\n", arr[j-1]);
    }
}
int write_double_data(FILE *fp, double *arr){
    int odom_len = sizeof(odom_sec) / sizeof(odom_sec[0]);
    int lidar_len = sizeof(lidar_sec) / sizeof(lidar_sec[0]);
    int32_t current_diff, next_diff;
    int cnt=0;
    printf("Odom: %d\nLidar: %d\n", odom_len, lidar_len);
    int j=0;
    for (int i=0; i<lidar_len; i++){
        cnt = 0;
        current_diff = time_diff(i,j);
        next_diff = current_diff;
        while(current_diff >= next_diff){
            j++;
            next_diff = time_diff(i,j);
            cnt++;
        }
        current_diff = time_diff(i,j-1);
        fprintf(fp, "%lf,\n", arr[j-1]);
    }
}
int main(){
    FILE *fp;
    if ((fp = fopen("odom_sync_msg.h","w")) == NULL){
		printf("Error Opening %s stopping... \n", "odom_sync_msg.h");
		return -1;
	}
    
    printf("Writing odom_sec\n");
    fprintf(fp,"int32_t odom_sec[] = {\n");
    write_int32_t_data(fp, odom_sec);
    fprintf(fp,"};\n\n");
    printf("Done!\n\n");
    
    printf("Writing odom_nsec\n");    
    fprintf(fp,"int32_t odom_nsec[] = {\n");
    write_int32_t_data(fp, odom_nsec);
    fprintf(fp,"};\n\n");
    printf("Done!\n\n");

    printf("Writing o_x\n");    
    fprintf(fp,"double o_x [] = {\n");
    write_double_data(fp, o_x);
    fprintf(fp,"};\n\n");
    printf("Done!\n\n");
    
    printf("Writing o_y\n");    
    fprintf(fp,"double o_y [] = {\n");
    write_double_data(fp, o_y);
    fprintf(fp,"};\n\n");
    printf("Done!\n\n");
    
    printf("Writing o_z\n");
    fprintf(fp,"double o_z [] = {\n");
    write_double_data(fp, o_z);
    fprintf(fp,"};\n\n");
    printf("Done!\n\n");

    printf("Writing o_w\n");
    fprintf(fp,"double o_w [] = {\n");
    write_double_data(fp, o_w);
    fprintf(fp,"};\n\n");
    printf("Done!\n\n");

    printf("Writing pose_x\n");    
    fprintf(fp,"double pose_x [] = {\n");
    write_double_data(fp, o_w);
    fprintf(fp,"};\n\n");
    printf("Done!\n\n");

    printf("Writing ang_z\n");
    fprintf(fp,"double ang_z [] = {\n");
    write_double_data(fp, o_w);
    fprintf(fp,"};\n\n");
    printf("Done!\n\n");


    // clipper();
    return 0;
}