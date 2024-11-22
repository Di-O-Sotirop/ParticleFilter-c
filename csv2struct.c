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
/*  Open a file, iterate each char.
    Everytime you find a comma, 
    combine the past characters into a char
*/
/*==============================================================*/
int read_convert(char *ifilename, char *ofilename){
/*==============================================================*/

	FILE *ifp, *ofp;
	char ch;
	int nlines = 0;
    int byte;
    int j=0;
    int i=0;
    char char_buffer[3]; //3 decimals
    char line_buffer[LINE_LEN];
    union Odom {
        struct odom_msg_t odom_struct;
        char odom_bin[LINE_LEN];
    } odom;

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
                byte += ((int)char_buffer[k-1])*pow(10,i-k);
                printf(".");
            }
            i=0;
            line_buffer[j] = (char)byte;
            j++;
            printf("Byte[%d][%d] :%c:%d\n",nlines, j, byte, (char)byte);
            //Save byte to output file
            fprintf(ofp, "%c", (char)byte);
        }else{
            char_buffer[i] = ch-48;
            printf("%d+",(int)ch-48);
            i++;
        }        
		if(ch=='\n'){
            nlines++;
            if (nlines >= 8)
                break;
            printf("\n");
            fprintf(ofp, "%c", (char)byte);
            printf("Numbers read: %d\n",j);
            j=0;
            for(int k=0; k<LINE_LEN; k++){
               // fprintf(ofp, "%c", (char)byte);
            }
        }
	}
	fclose(ifp);
    printf("Lines Read: %d\n",nlines);
}
/*==============================================================*/
int write_struct(FILE *fp, struct odom_msg_t odom_struct){
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
            j++;
            //printf("Byte[%d][%d] :%c:%d\n",nlines, j, byte, (char)byte);
            //Save byte to output file
           // fprintf(ofp, "%c", (char)byte);
        }else{
            char_buffer[i] = ch-48;
            //printf("%d+",(int)ch-48);
            i++;
        }        
		if(ch=='\n'){
            nlines++;
            // if (nlines >= 8)
            //     break;
            printf("\n");
            printf("Numbers read: %d\n",j);
            j=0;
            printf("Printing struct: \n");
            print_odom_struct(odom.odom_struct);
            write_struct(ofp, odom.odom_struct);
            // for(int k=0; k<LINE_LEN; k++){
            //     fprintf(ofp, "%c,", odom.odom_bin[k]);
            // }
            fprintf(ofp, "\n");
        }
	}
	fclose(ifp);
    printf("Lines Read: %d\n",nlines);
}

/*==============================================================*/
struct odom_msg_t bin2odom(char *filename){
/*==============================================================*/    
    FILE *ifp ;
	char ch;
	int nlines = 0;
    int i = 0;
    char byte_line[LINE_LEN];

    union Odom {
        struct odom_msg_t odom_struct;
        char odom_bin[LINE_LEN];
    } odom;

	if ((ifp = fopen(filename,"r")) == NULL){
		printf("Error reading %s stopping... \n", filename);
	}

	while((ch=fgetc(ifp))!=EOF) {

		if(ch=='\n'){
            nlines++;
            printf("\n");
            return odom.odom_struct;
        }
        odom.odom_bin[i] = ch;
        i++;
    }
	fclose(ifp);
    printf("Lines Read: %d\n",nlines);
}



/*==============================================================*/
int main(){
/*==============================================================*/
    
    struct odom_msg_t odom_struct;

    csv_to_struct("file.dta", "file.txt");
    //odom_struct = bin2odom("file.bin");
    printf("seq = %d\n, sec = %d\n, nsec = %d\n, stringid[24]=%s\n,pos_x=%lf\n,pos_y=%lf\n,pos_z=%lf\n,ori_x=%lf\n,ori_y=%lf\n,ori_z=%lf\n,ori_w=%lf\n,covariance[36]==%lf\n,lin_x=%lf\n,lin_y=%lf\n,lin_z=%lf\n,ang_x=%lf\nang_y=%lf\n,ang_z=%lf\ncovariance_twist[36]=%lf\n",
    odom_struct.seq,
    odom_struct.sec,
    odom_struct.nsec,
    odom_struct.stringid,
    odom_struct.pos_x,
    odom_struct.pos_y,
    odom_struct.pos_z,
    odom_struct.ori_x,
    odom_struct.ori_y,
    odom_struct.ori_z,
    odom_struct.ori_w,
    odom_struct.covariance[0],
    odom_struct.lin_x,
    odom_struct.lin_y,
    odom_struct.lin_z,
    odom_struct.ang_x,
    odom_struct.ang_y,
    odom_struct.ang_z,
    odom_struct.covariance_twist[0]
    );

}