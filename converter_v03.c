#include <stdio.h>
#include <float.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#define LINE_LEN 716
struct odom_msg_t
{
    int32_t seq;
    int32_t sec;
    int32_t nsec;

    char stringid[24];
    
    double pos_x;
    double pos_y;
    double pos_z;

    double ori_x;
    double ori_y;
    double ori_z;
    double ori_w;

    double covariance[36];

    double lin_x;
    double lin_y;
    double lin_z;

    double ang_x;
    double ang_y;
    double ang_z;

    double covariance_twist[36];
    
};

/*==============================================================*/
double unionconv(char*bytes){
/*==============================================================*/
    union{char b[8];double d;} uni;
    for (int i=8; i>0; i--)
        uni.b[i] = bytes[i];
    return uni.d;
}
/*==============================================================*/
struct odom_msg_t line2struct_old(char *iline){
/*==============================================================*/
    // char line[716];
    // for (int i=0;i<716;i++)
    //     line[i] = iline[i];
    char line[8];
    struct odom_msg_t odom;
    memcpy(&odom.seq, &line, sizeof(int32_t));
    memcpy(&odom.sec, &line+4, sizeof(int32_t));
    memcpy(&odom.nsec, &line+8, sizeof(int32_t));

    for(int i=0;i<24;i++)
        odom.stringid[i] = line[i+12];
    for (int i=0;i<8;i++) line[i] = iline[36+i];
    odom.pos_x=unionconv(line);
    // odom.pos_x = unionconv(&iline[36]);
    //odom.pos_x = unionconv(line+36);
    // odom.pos_y = unionconv(line+44);
    // odom.pos_z = unionconv(line+52);

    // odom.ori_x = unionconv(line+60);
    // odom.ori_y = unionconv(line+68);
    // odom.ori_z = unionconv(line+76);
    // odom.ori_w = unionconv(line+84);

    // for(int i=0;i<36;i++)
    //     odom.covariance[i] = unionconv(line+92+i*8);

    // odom.lin_x = unionconv(line+380);
    // odom.lin_y = unionconv(line+388);
    // odom.lin_z = unionconv(line+396);

    // odom.ang_x = unionconv(line+404);
    // odom.ang_y = unionconv(line+412);
    // odom.ang_z = unionconv(line+420);
    // for(int i=0;i<36;i++)
    //     odom.covariance_twist[i]=unionconv(line+428+i*8);
    return odom;
}
/*==============================================================*/
/*Totally dumb implementation but it works. fors are because
 * using directly the pointer yields stack smashing             */
struct odom_msg_t line2struct(char *iline){
/*==============================================================*/
    // char line[716];
    // for (int i=0;i<716;i++)
    //     line[i] = iline[i];
    union{
        struct p_odom_t {
            int32_t seq;
            int32_t sec;
            int32_t nsec;
            char    stringid[24];
        } p_odom;
        char bytes[12+24];
    } uni;
    for(int i=0;i<36;i++)
        uni.bytes[i] = iline[i];
    char line[8];

    struct odom_msg_t odom;
    // memcpy(&odom.seq, &line, sizeof(int32_t));
    // memcpy(&odom.sec, &line+4, sizeof(int32_t));
    // memcpy(&odom.nsec, &line+8, sizeof(int32_t));

    // for(int i=0;i<24;i++)
    //     odom.stringid[i] = line[i+12];
    odom.seq = uni.p_odom.seq;
    odom.sec = uni.p_odom.sec;
    odom.nsec = uni.p_odom.nsec;
    for(int i=0;i<24;i++)
        odom.stringid[i] = uni.p_odom.stringid[i];

    for (int i=0;i<8;i++) line[i] = iline[36+i];
    odom.pos_x=unionconv(line);
    
    for (int i=0;i<8;i++) line[i] = iline[44+i];
    odom.pos_y = unionconv(line);

    for (int i=0;i<8;i++) line[i] = iline[52+i];
    odom.pos_z = unionconv(line);


    for (int i=0;i<8;i++) line[i] = iline[60+i];
    odom.ori_x = unionconv(line);
    
    for (int i=0;i<8;i++) line[i] = iline[68+i];
    odom.ori_y = unionconv(line);
    
    for (int i=0;i<8;i++) line[i] = iline[76+i];
    odom.ori_z = unionconv(line);

    for (int i=0;i<8;i++) line[i] = iline[84+i];
    odom.ori_w = unionconv(line);

    for(int i=0;i<36;i++){
        for (int ii=0;ii<8;ii++) line[ii] = iline[92+i*8+ii];
        odom.covariance[i] = unionconv(line);
    }
    for (int i=0;i<8;i++) line[i] = iline[380+i];
    odom.lin_x = unionconv(line);

    for (int i=0;i<8;i++) line[i] = iline[388+i];
    odom.lin_y = unionconv(line);

    for (int i=0;i<8;i++) line[i] = iline[396+i];
    odom.lin_z = unionconv(line);

    for (int i=0;i<8;i++) line[i] = iline[404+i];
    odom.ang_x = unionconv(line);
    
    for (int i=0;i<8;i++) line[i] = iline[412+i];
    odom.ang_y = unionconv(line);
    
    for (int i=0;i<8;i++) line[i] = iline[420+i];
    odom.ang_z = unionconv(line);
    
    for(int i=0;i<36;i++){
        for (int ii=0;ii<8;ii++) 
            line[ii] = iline[428+i*8+ii];
        odom.covariance_twist[i]=unionconv(line);
    }
    return odom;
}
/*==============================================================*/
void print_odom_struct(struct odom_msg_t odom_struct){
/*==============================================================*/
    printf("seq = %u\n sec = %u\n nsec = %u\n",
    odom_struct.seq,
    odom_struct.sec,
    odom_struct.nsec);
    printf("stringid[24]=");
    for (int i=0; i<24;i++)
        printf("%c", odom_struct.stringid[i]);
    printf("\npos_x=%lf\npos_y=%lf\npos_z=%lf\nori_x=%lf\nori_y=%lf\nori_z=%lf\nori_w=%lf\n",
    odom_struct.pos_x,
    odom_struct.pos_y,
    odom_struct.pos_z,
    odom_struct.ori_x,
    odom_struct.ori_y,
    odom_struct.ori_z,
    odom_struct.ori_w);
    printf("covariance=\n");
    for (int i=0; i<6;i++){
        for(int ii=0; ii<6;ii++){
            printf("%f ", odom_struct.covariance[i*6+ii]);
        }
        printf("\n");
    }
    printf("\nlin_x=%lf\n,lin_y=%lf\n,lin_z=%lf\n,ang_x=%lf\nang_y=%lf\n,ang_z=%lf\n",
    odom_struct.lin_x,
    odom_struct.lin_y,
    odom_struct.lin_z,
    odom_struct.ang_x,
    odom_struct.ang_y,
    odom_struct.ang_z);
    printf("covariance_twist=\n");
    for (int i=0; i<6;i++){
        for(int ii=0; ii<6;ii++){
            printf("%f ", odom_struct.covariance_twist[i*6+ii]);
        }
        printf("\n");
    }
}
/*==============================================================*/
int write_struct(FILE *fp, struct odom_msg_t odom_struct){
/*==============================================================*/
    fprintf(fp, "%u,%u,%u",
    odom_struct.seq,
    odom_struct.sec,
    odom_struct.nsec);
    for (int i=0; i<24;i++)
        fprintf(fp, "%c", odom_struct.stringid[i]);
    fprintf(fp,"%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
    odom_struct.pos_x,
    odom_struct.pos_y,
    odom_struct.pos_z,
    odom_struct.ori_x,
    odom_struct.ori_y,
    odom_struct.ori_z,
    odom_struct.ori_w);
    for (int i=0; i<6;i++){
        for(int ii=0; ii<6;ii++){
            fprintf(fp,"%lf,", odom_struct.covariance[i*6+ii]);
        }
    }
    fprintf(fp,"%lf,%lf,%lf,%lf,%lf,%lf,",
    odom_struct.lin_x,
    odom_struct.lin_y,
    odom_struct.lin_z,
    odom_struct.ang_x,
    odom_struct.ang_y,
    odom_struct.ang_z);
    for (int i=0; i<6;i++){
        for(int ii=0; ii<6;ii++){
            fprintf(fp,"%lf,", odom_struct.covariance_twist[i*6+ii]);
        }
    }
}

/*==============================================================*/
int csv_to_struct(char *ifilename, char *ofilename){
/*==============================================================*/

	FILE *ifp, *ofp;
	char ch;
	int nlines = 0;
    unsigned short int byte;
    int j=0;
    int i=0;
    char char_buffer[3]; //3 decimals
    char line_buffer[LINE_LEN];
    union Odom {
        struct odom_msg_t odom_struct;
        char odom_bin[LINE_LEN];
    } odom;
    struct odom_msg_t msg;

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
            odom.odom_bin[j] = (char)byte;
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
            print_odom_struct(odom.odom_struct);
            msg = line2struct(line_buffer); 
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
      csv_to_struct("file.dta", "file.txt");

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
    d = unionconv(bytes);
    // printf("double: %lf\n", d);
    // union {char bytes[8]; double d;} uni;
    // uni.d = d;
    // printf("re-double: %lf\n", unionconv(uni.bytes));

}