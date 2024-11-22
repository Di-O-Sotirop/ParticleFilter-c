#include <stdio.h>
#include <float.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#define LINE_LEN 8712//712
#define LIDAR_RAYS 1081
struct lidar_msg_t
{
    int32_t seq;
    int32_t sec;
    int32_t nsec;

    char stringid[16];
    
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    float ranges[LIDAR_RAYS];
    float intensities[LIDAR_RAYS];
};

/*==============================================================*/
float flt_unionconv(char*bytes){
/*==============================================================*/
    union{char b[4];float d;} uni;
    for (int i=4; i>0; i--)
        uni.b[i] = bytes[i];
    return uni.d;
}
/*==============================================================*/
/*Totally dumb implementation but it works. fors are because
 * using directly the pointer yields stack smashing             */
struct lidar_msg_t lidarline2struct(char *iline){
/*==============================================================*/

    //Declarations
    union{
        struct p_lidar_t {
            int32_t seq;
            int32_t sec;
            int32_t nsec;
            char    stringid[16];
        } p_lidar;
        char bytes[12+16];
    } uni;
    
    for(int i=0;i<(12+16);i++)
        uni.bytes[i] = iline[i];
    char line[4];

    struct lidar_msg_t lidar;
    printf("header done\n");
    //Assignmnets

    lidar.seq = uni.p_lidar.seq;
    lidar.sec = uni.p_lidar.sec;
    lidar.nsec = uni.p_lidar.nsec;
    for(int i=0;i<16;i++)
        lidar.stringid[i] = uni.p_lidar.stringid[i];
    printf("string assigned\n");
    for (int i=0;i<4;i++) line[i] = iline[28+i];
    printf("angle min loaded\n"); for(int i=0; i<4;i++) printf("line[%d]:%hu\n",i,line[i]);
    flt_unionconv(line);
    printf("uniconv test complete\n");
    lidar.angle_min=flt_unionconv(line);
    printf("angle min converted and assigned\n");
    for (int i=0;i<4;i++) line[i] = iline[32+i];
    lidar.angle_max = flt_unionconv(line);
    printf("angle max converted and assigned\n");
    for (int i=0;i<4;i++) line[i] = iline[36+i];
    lidar.angle_increment = flt_unionconv(line);

    for (int i=0;i<4;i++) line[i] = iline[40+i];
    lidar.time_increment = flt_unionconv(line);
    
    for (int i=0;i<4;i++) line[i] = iline[44+i];
    lidar.scan_time = flt_unionconv(line);
    
    for (int i=0;i<4;i++) line[i] = iline[48+i];
    lidar.range_min = flt_unionconv(line);

    for (int i=0;i<4;i++) line[i] = iline[52+i];
    lidar.range_max = flt_unionconv(line);
    printf("range max converted and assigned\n");
    //skip tag 4 bytes
    for(int i=0;i<(1081);i++){
        for (int ii=0;ii<4;ii++) line[ii] = iline[60+i*4+ii];
        lidar.ranges[i] = flt_unionconv(line);
    }
    printf("ranges converted and assigned\n");
    //skip tag 4 bytes
    for(int i=0;i<(1081);i++){
        for (int ii=0;ii<4;ii++) line[ii] = iline[4388+i*4+ii];
        lidar.intensities[i] = flt_unionconv(line);
    }
    printf("Conv Finished, Returning\n");
    return lidar;
}
/*==============================================================*/
void print_lidar_struct(struct lidar_msg_t lidar_struct){
/*==============================================================*/
    printf("seq = %u\n sec = %u\n nsec = %u\n",
    lidar_struct.seq,
    lidar_struct.sec,
    lidar_struct.nsec);
    printf("stringid[16]=");
    for (int i=0; i<16;i++)
        printf("%c", lidar_struct.stringid[i]);
    printf("\nangle_min=%f\nangle_max=%f\nangle_incr=%f\ntime_incr=%lf\nscan_time=%f\nrange_min=%f\nrange_max=%f\n",
    lidar_struct.angle_min,
    lidar_struct.angle_max,
    lidar_struct.angle_increment,
    lidar_struct.time_increment,
    lidar_struct.scan_time,
    lidar_struct.range_min,
    lidar_struct.range_max);
    printf("ranges=\n");
    for (int i=0; i< LIDAR_RAYS;i++){
        printf("%f ", lidar_struct.ranges[i]);
        printf("\n");
    }
    printf("intensities=\n");
    for (int i=0; i< LIDAR_RAYS;i++){
        printf("%f ", lidar_struct.intensities[i]);
        printf("\n");
    }
}
/*==============================================================*/
int write_struct(FILE *fp, struct lidar_msg_t lidar_struct){
/*==============================================================*/
    fprintf(fp, "%u,%u,%u",
    lidar_struct.seq,
    lidar_struct.sec,
    lidar_struct.nsec);
    for (int i=0; i<16;i++)
        fprintf(fp, "%c", lidar_struct.stringid[i]);
    fprintf(fp,"%f,%f,%f,%f,%f,%f,%f,",
    lidar_struct.angle_min,
    lidar_struct.angle_max,
    lidar_struct.angle_increment,
    lidar_struct.time_increment,
    lidar_struct.scan_time,
    lidar_struct.range_min,
    lidar_struct.range_max);
    for (int i=0; i<LIDAR_RAYS;i++){
        fprintf(fp,"%f,", lidar_struct.ranges[i]);
    }
    for (int i=0; i<LIDAR_RAYS;i++){
        fprintf(fp,"%f,", lidar_struct.intensities[i]);
    }
}

/*==============================================================*/
int lidar_csv_to_struct(char *ifilename, char *ofilename){
/*==============================================================*/

	FILE *ifp, *ofp;
	char ch;
	int nlines = 0;
    unsigned short int byte;
    int j=0;
    int i=0;
    char char_buffer[3]; //3 decimals
    char line_buffer[LINE_LEN];
    union Lidar {
        struct lidar_msg_t lidar_struct;
        char lidar_bin[LINE_LEN];
    } lidar;
    struct lidar_msg_t msg;

    /*  Opening Files   */
	if ((ifp = fopen(ifilename,"r")) == NULL){
		printf("Error reading %s stopping... \n", ifilename);
		return -1;
	}

	if ((ofp = fopen(ofilename,"w")) == NULL){
		printf("Error Opening %s stopping... \n", ofilename);
		return -1;
	}

    /*  Process     */
	while((ch=fgetc(ifp))!=EOF) {
        if(ch==','){ 
            byte = 0;
            for(int k=i; k>=0;k--){//reverse
                byte += ((unsigned short int)char_buffer[k-1])*pow(10,i-k);
                printf(".");
            }
            i=0;
            lidar.lidar_bin[j] = (char)byte;
            line_buffer[j]= (char)byte;
            j++;
        }else if(ch=='\n'){
            nlines++;
            // if (nlines >= 8)
            //     break;
            printf("\n");
            printf("Numbers read: %d\n",j);
            j=0;
            printf("Printing struct: \n");
            // print_odom_struct(lidar.lidar_struct);
            msg = lidarline2struct(line_buffer); 
            write_struct(ofp, msg);
            // for(int k=0; k<LINE_LEN; k++){
            //     fprintf(ofp, "%c,", odom.odom_bin[k]);
            // }
            fprintf(ofp, "\n");
        }else{
            char_buffer[i] = ch-48;
            //printf("%d+",(int)ch-48);
            i++;
        }
	}
	fclose(ifp);
    printf("Lines Read: %d\n",nlines);
}

/*==============================================================*/
int main(){
/*==============================================================*/
    //bytes2double();
      lidar_csv_to_struct("laserScan_expo.csv", "laser_file.txt");

    double d;
    char bytes[8];
    bytes[7] = 0x3F;
    bytes[6] = 0xD1;
    bytes[5] = 0x9B;
    bytes[4] = 0x94;
    bytes[3] = 0xC0;
    bytes[2] = 0x00;
    bytes[1] = 0x00;
    bytes[0] = 0x00;
    d = flt_unionconv(bytes);
    // printf("double: %lf\n", d);
    // union {char bytes[8]; double d;} uni;
    // uni.d = d;
    // printf("re-double: %lf\n", unionconv(uni.bytes));

}