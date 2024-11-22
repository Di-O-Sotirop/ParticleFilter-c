#include <stdio.h>
#include <float.h>
#include <stdint.h>
#include <math.h>

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

double bin2f64(int* arr)
{
    /*  1-sign
        11-expo
        52-data(+1 hidden bit as a 1)
    */
   // short MSdata = (a2 << 4);
    short MSdata = (arr[1] % 16);
    double res = 0;
    for (int i=2; i<8; i++)
    {
        printf("%d:%f\n",i,res);
        double temp = 0;
        printf("temp: %f + %d = %f\n",temp, arr[i], temp+arr[i]);
        temp += arr[i];
        printf("temp %f * %d = ",temp,(1 << ((7-i)*8)));
        temp = temp * (1 << ((7-i)*8)); 
        res += temp;
        printf(" %f\n", temp);
    }
    return 0.0;
}
/*  Open a file, iterate each char.
    Everytime you find a comma, 
    combine the past characters into a char
*/
int read_convert(char *ifilename, char *ofilename){
	FILE *ifp, *ofp;
	char ch;
	int nlines = 0;
    int byte;
    int j=0;
    int i=0;
    char char_buffer[3]; //3 decimals
    char line_buffer[716];

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
    //        if (nlines >= 8)
      //          break;
            printf("\n");
            fprintf(ofp, "%c", ch);
            printf("Numbers read: %d\n",j);
            j=0;
        }
	}
	fclose(ifp);
    printf("Lines Read: %d\n",nlines);
	//return nlines;
}
//   while (fgets(line, sizeof(line), dat_particles) != NULL && p<MAX_PARTICLES) {
//         //printf("\nPARTICLE[%d]\n",p);
//         sscanf(line, "%d %f %f %f %lf", &particles[p].id, &particles[p].x, &particles[p].y, &particles[p].yaw, &particles[p].weight);
//         for (int i = 0; i < N_RAYS_DS; i++) {
//             sscanf(line, "%f", &particles[p].rays[i]);
//         }
int read_convert_original(char *filename){
	FILE *ifp ;
	char ch;
	int nlines = 0;

	if ((ifp = fopen(filename,"r")) == NULL){
		printf("Error reading %s stopping... \n", filename);
		return -1;
	}

	while((ch=fgetc(ifp))!=EOF) {
		if(ch=='\n'){
            nlines++;
            printf("\n");
            printf("!");
        }
        if (nlines > 16)
            break;
        printf("%c",ch);
	}
	fclose(ifp);
    printf("Lines Read: %d\n",nlines);
	//return nlines;
}
  
float bin2flt(char *received_data){
    union {
    char chars[4];
    float f;
    } u;

    for (int i = 0; i < 4; i++)
    u.chars[3-i] = received_data[i];
    float f1 = u.f; 

    return f1;
}


int main(){
    int arr[8] = {255,255,255,255,255,255,255,255};
    //bin2f64(arr);
    read_convert("file.dta", "file.bin");

    char str[] = {217,103,112,234};
    printf("FLOAT: %f\n ",bin2flt(str));


}