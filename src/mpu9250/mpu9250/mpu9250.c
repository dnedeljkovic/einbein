////////////////////////////////////////////////////////////////////////////
//
//  This file is part of linux-mpu9150
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <stdio.h>
#include <string.h>

#include "../glue/linux_glue.h"
#include "../eMPL/inv_mpu.h"
#include "../eMPL/inv_mpu_dmp_motion_driver.h"
#include "mpu9250.h"

static int data_ready();
static int data_fusion(mpudata_t *mpu);
static unsigned short inv_row_2_scale(const signed char *row);
static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);
void calibrate_data(mpudata_t *mpu);


#define AKM_DATA_READY      (0x01)
#define AKM_DATA_OVERRUN    (0x02)
#define AKM_OVERFLOW        (0x80)
#define AKM_DATA_ERROR      (0x40)

int first = 1;
short rawGyro[3];
short rawAccel[3];
short rawMag[3];
double accel[3];
double gyro[3];
double mag[3];
double virtMag[3] = {1,0,0};
double accNorm = 0;
double magNorm = 0;
double rEstNorm = 0;
double Axz = 0;
double Ayz = 0;
double AxzOld = 0;
double AyzOld = 0; 
double rAcc[3] = {0,0,0};
double rMag[3] = {0,0,0};
double rEst[3] = {0,0,0};
double rEstOld[3] = {0,0,0};
double rGyro[3] = {0,0,0};
double angDisp[3] = {0,0,0};
double angDispGyro[3] = {0,0,0};
double angDispGyroOld[3] = {0,0,0};
double angDispAcc[3] = {0,0,0};
double angDispMag[3] = {0,0,0};
double orthErr = 0;
double xOrth[3] = {0,0,0};
double yOrth[3] = {0,0,0};
double zOrth[3] = {0,0,0};
double xNorm[3] = {0,0,0};
double yNorm[3] = {0,0,0};
double zNorm[3] = {0,0,0};
double euler[3] = {0,0,0};
double dcm[9] = {0,0,0,0,0,0,0,0,0};
double timeOld = 0;
double sg = 1;
double sm = 1;
double sa = 0.1;
int wGyro = 10; //can be chosen from 5 to 20;

#define ACCSENS  (32768/2)
#define GYROSENS (32768/2000)
#define MAGSENS  (32768/2)


int cRow = 0;
char data[512000];




int mpu9250_init(int i2c_bus, int sample_rate)
{
	signed char gyro_orientation[9] = { 1, 0, 0,
                                        0, 1, 0,
                                        0, 0, 1 };

	if (i2c_bus < MIN_I2C_BUS || i2c_bus > MAX_I2C_BUS) {
		printf("Invalid I2C bus %d\n", i2c_bus);
		return -1;
	}

	if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE) {
		printf("Invalid sample rate %d\n", sample_rate);
		return -1;
	}

	linux_set_i2c_bus(i2c_bus);

	printf("\nInitializing IMU .");
	fflush(stdout);

//*******Work around in mpu_init werden sensoren gelöscht, müssen aber vorher != 0 sein!!
	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)) {
		printf("\nmpu_set_sensors() failed\n");
		return -1;
	}
//**************

	if (mpu_init(NULL)) {
		printf("\nmpu_init() failed\n");
		return -1;
	}

	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)) {
		printf("\nmpu_set_sensors() failed\n");
		return -1;
	}

// 	if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
// 		printf("\nmpu_configure_fifo() failed\n");
// 		return -1;
// 	}

	if (mpu_set_sample_rate(sample_rate)) {
		printf("\nmpu_set_sample_rate() failed\n");
		return -1;
	}

// 	if (dmp_load_motion_driver_firmware()) {
// 		printf("\ndmp_load_motion_driver_firmware() failed\n");
// 		return -1;
// 	}
// 
// 	if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {
// 		printf("\ndmp_set_orientation() failed\n");
// 		return -1;
// 	}
// 
// 	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL
// 						| DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {
// 		printf("\ndmp_enable_feature() failed\n");
// 		return -1;
// 	}
//  
// 	if (dmp_set_fifo_rate(sample_rate)) {
// 		printf("\ndmp_set_fifo_rate() failed\n");
// 		return -1;
// 	}
// 
// 	if (mpu_set_dmp_state(1)) {
// 		printf("\nmpu_set_dmp_state(1) failed\n");
// 		return -1;
// 	}

	printf(" done\n\n");

	return 0;
}

void mpu9250_exit()
{
	// turn off the DMP on exit 
	if (mpu_set_dmp_state(0))
		printf("mpu_set_dmp_state(0) failed\n");

	// TODO: Should turn off the sensors too
}

int mpu9250_read_dmp(mpudata_t *mpu)
{
  //printf("read dmp\n");
	short sensors;
	unsigned char more;

	if (!data_ready()){
	  //printf("dmp data not ready: %d\n",data_ready());
	  //linux_delay_ms(1);
	  return -1;
	}
	

	if (dmp_read_fifo(mpu->rawGyro, mpu->rawAccel, mpu->rawQuat, &mpu->dmpTimestamp, &sensors, &more) < 0) {
		printf("dmp_read_fifo() failed\n");
		return -1;
	}

	while (more) {
		// Fell behind, reading again
		if (dmp_read_fifo(mpu->rawGyro, mpu->rawAccel, mpu->rawQuat, &mpu->dmpTimestamp, &sensors, &more) < 0) {
			printf("dmp_read_fifo() failed\n");
			return -1;
		}
	}

	return 0;
}


int mpu9250_read(mpudata_t *mpu)
{
  //printf("read dmp\n");
	if (mpu9250_read_dmp(mpu) != 0){
	 // printf("read dmp failed\n");
	  return -1;
	}  

	calibrate_data(mpu);

	return data_fusion(mpu);
}

int data_ready()
{
	short status;

	if (mpu_get_int_status(&status) < 0) {
		printf("mpu_get_int_status() failed\n");
		return 0;
	}

	return (status == (MPU_INT_STATUS_DATA_READY/* | MPU_INT_STATUS_DMP | MPU_INT_STATUS_DMP_0*/));
}

void calibrate_data(mpudata_t *mpu)
{
  mpu->calibratedAccel[VEC3_X] = mpu->rawAccel[VEC3_X];
  mpu->calibratedAccel[VEC3_Y] = mpu->rawAccel[VEC3_Y];
  mpu->calibratedAccel[VEC3_Z] = mpu->rawAccel[VEC3_Z];

}



int data_fusion(mpudata_t *mpu)
{
	quaternion_t dmpQuat;
	vector3d_t dmpEuler;
	quaternion_t unfusedQuat;
	
	dmpQuat[QUAT_W] = (float)mpu->rawQuat[QUAT_W];
	dmpQuat[QUAT_X] = (float)mpu->rawQuat[QUAT_X];
	dmpQuat[QUAT_Y] = (float)mpu->rawQuat[QUAT_Y];
	dmpQuat[QUAT_Z] = (float)mpu->rawQuat[QUAT_Z];

	quaternionNormalize(dmpQuat);

	quaternionToEuler(dmpQuat, dmpEuler);

	mpu->fusedEuler[VEC3_X] = dmpEuler[VEC3_X];
	mpu->fusedEuler[VEC3_Y] = -dmpEuler[VEC3_Y];
	mpu->fusedEuler[VEC3_Z] = -dmpEuler[VEC3_Z]; //0;

	return 0;
}


/* Derivation of the acceleration to get velocity and position
 */ 
void derivate_accel(mpudata_t *mpu){
  double ax = (float)mpu->calibratedAccel[VEC3_X]/16384;//*9.81/16384;
  double ay = (float)mpu->calibratedAccel[VEC3_Y]/16384;//*9.81/16384;
  double az = (float)mpu->calibratedAccel[VEC3_Z]/16384;//*9.81/16384;
  double deltaTms = mpu->dmpTimestamp - timeOld;
  double deltaT = deltaTms/1000;
  double vx, vy, vz;
  double px, py, pz;
  quaternion_t dmpQuat;
  matrix3d_t rotMatrix;
  
  dmpQuat[QUAT_W] = (float)mpu->rawQuat[QUAT_W];
  dmpQuat[QUAT_X] = (float)mpu->rawQuat[QUAT_X];
  dmpQuat[QUAT_Y] = (float)mpu->rawQuat[QUAT_Y];
  dmpQuat[QUAT_Z] = (float)mpu->rawQuat[QUAT_Z];
  
  quaternionNormalize(dmpQuat);
    
  quaternionToRotMatrix(dmpQuat,rotMatrix);
  
  //%printf("Rotations Matrix\n");
 //printf("%f\t%f\t%f\n%f\t%f\t%f\n%f\t%f\t%f\n",rotMatrix[0][0],rotMatrix[0][1],rotMatrix[0][2],rotMatrix[1][0],rotMatrix[1][1],rotMatrix[1][2],rotMatrix[2][0],rotMatrix[2][1],rotMatrix[2][2]);
  
  // if first ..... einfügen !!!!!!!!!!!!!!!
  
  vx = ax * deltaT;
  vy = ay * deltaT;
  vz = az * deltaT;
  px = 0.5 * ax * deltaT*deltaT;
  py = 0.5 * ay * deltaT*deltaT;  
  pz = 0.5 * az * deltaT*deltaT;


  
  //printf("Position: X %f   Y %f   Z %f\n",px,py,pz);
  printf("%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f\n",ax,ay,az,vx,vy,vz,px,py,pz,deltaT, dmpQuat[QUAT_W],dmpQuat[QUAT_X],dmpQuat[QUAT_Y],dmpQuat[QUAT_Z]);
  //printf("%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f\n",ax,ay,az,vx,vy,vz,px,py,pz,deltaT, dmpQuat[QUAT_W],dmpQuat[QUAT_X],dmpQuat[QUAT_Y],dmpQuat[QUAT_Z],rotMatrix[0][0],rotMatrix[0][1],rotMatrix[0][2],rotMatrix[1][0],rotMatrix[1][1],rotMatrix[1][2],rotMatrix[2][0],rotMatrix[2][1],rotMatrix[2][2]);
  //printf("%f;%f;%f;%f\n",dmpQuat[QUAT_W],dmpQuat[QUAT_X],dmpQuat[QUAT_Y],dmpQuat[QUAT_Z]);
  
  timeOld = mpu->dmpTimestamp;
  
  
}





/* Position estimation is an example of the algorithmus 
 * from http://www.starlino.com/imu_guide.html
 */
void estimate_position(/*mpudata_t *mpu, unsigned long loop_delay,*/ double time){
  int i;
  
  double timedelay = (time-timeOld);
  
  
  unsigned char data_read[6];
  
  //read accel
  i2c_read(0x69, 0x3B, 6, data_read);
//   printf("XH: %d\n",(int)data_read[0]);
//   printf("XL: %d\n",(int)data_read[1]);
//   printf("YH: %d\n",(int)data_read[2]);
//   printf("YL: %d\n",(int)data_read[3]);
//   printf("ZH: %d\n",(int)data_read[4]);
//   printf("ZL: %d\n",(int)data_read[5]);
  
  rawAccel[VEC3_X] = ((short)data_read[0] << 8) | data_read[1];
  rawAccel[VEC3_Y] = ((short)data_read[2] << 8) | data_read[3];
  rawAccel[VEC3_Z] = ((short)data_read[4] << 8) | data_read[5];
  
//   printf("%d  %d  %d \t ", rawAccel[VEC3_X], rawAccel[VEC3_Y], rawAccel[VEC3_Z]);
  //printf("%d  %d  %d  %f\n", accel[VEC3_X], accel[VEC3_Y], accel[VEC3_Z], time);
  
  //read gyro
  i2c_read(0x69, 0x43, 6, data_read);
  
  rawGyro[VEC3_X] = ((short)data_read[0] << 8) | data_read[1];
  rawGyro[VEC3_Y] = ((short)data_read[2] << 8) | data_read[3];
  rawGyro[VEC3_Z] = ((short)data_read[4] << 8) | data_read[5];
  
//   printf("%d  %d  %d  \t", rawGyro[VEC3_X], rawGyro[VEC3_Y], rawGyro[VEC3_Z]);
  //printf("%d  %d  %d  %f\n\n", gyro[VEC3_X], gyro[VEC3_Y], gyro[VEC3_Z], time);
  
  
  
  for(i=VEC3_X;i<(VEC3_Z+1);i++){
    accel[i] = (float)rawAccel[i] / (ACCSENS);
    gyro[i] = (float)rawGyro[i] / (GYROSENS);
    angDispGyro[i] = gyro[i] *DEGREE_TO_RAD * timedelay; // *DEGREE_TO_RAD
    
  }
//   printf("%f  %f  %f \t %f\n", accel[VEC3_X], accel[VEC3_Y], accel[VEC3_Z], time);
  
  // normalize acceleration
  accNorm = sqrt(accel[VEC3_X]*accel[VEC3_X] + accel[VEC3_Y]*accel[VEC3_Y] + accel[VEC3_Z]*accel[VEC3_Z] );
  for(i=VEC3_X;i<(VEC3_Z+1);i++){
    rAcc[i] = accel[i] / accNorm;
  }
  
  if(first){
    printf("first estimation\n");
    for(i=VEC3_X;i<(VEC3_Z+1);i++){
      rEst[i] = rAcc[i];
      rEstOld[i] = rEst[i];
      angDispGyroOld[i] = angDispGyro[i];
    }
    first = 0;
  }else{
    // estimation
    Axz = AxzOld + (-angDispGyro[VEC3_X] - angDispGyroOld[VEC3_X])/2 ;
    Ayz = AyzOld + (-angDispGyro[VEC3_Y] - angDispGyroOld[VEC3_Y])/2 ;
    rGyro[VEC3_X] = sinf(Axz) / sqrt(1 + cosf(Axz)*cos(Axz) * tan(Ayz)*tan(Ayz));
    rGyro[VEC3_Y] = sinf(Ayz) / sqrt(1 + cosf(Ayz)*cos(Ayz) * tan(Axz)*tan(Axz));
    
   
    if(rEstOld[VEC3_Z]>0){
      rGyro[VEC3_Z] = sqrt(1 - rGyro[VEC3_X]*rGyro[VEC3_X] - rGyro[VEC3_Y]*rGyro[VEC3_Y]);
    }else if(rEstOld[VEC3_Z]==0){
      rGyro[VEC3_Z] = 0;
    }else{
      rGyro[VEC3_Z] = -sqrt(1 - rGyro[VEC3_X]*rGyro[VEC3_X] - rGyro[VEC3_Y]*rGyro[VEC3_Y]);
    }
    
    for(i=VEC3_X;i<(VEC3_Z+1);i++){
      rEst[i] = ((rAcc[i] + rGyro[i] * wGyro)/(1 + wGyro));
    }
//     printf("rAcc: %f  %f  %f\t rGyro: %f  %f  %f\t Axz: %f  Ayz: %f\t angDisp: %f %f  %f %f\t dt: %f\n",rAcc[0],rAcc[1],rAcc[2],rGyro[0],rGyro[1],rGyro[2],Axz,Ayz,angDisp[0],angDisp[1],angDispOld[0],angDispOld[1],timedelay);
     
    rEstNorm = sqrt(rEst[VEC3_X]*rEst[VEC3_X] + rEst[VEC3_Y]*rEst[VEC3_Y] + rEst[VEC3_Z]*rEst[VEC3_Z]);
    for(i=VEC3_X;i<(VEC3_Z+1);i++){
      rEst[i] = rEst[i] / rEstNorm;
      rEstOld[i] = rEst[i];
      angDispGyroOld[i] = angDispGyro[i];
    }

    
    AxzOld = atan2f(rEstOld[VEC3_X],rEstOld[VEC3_Z]);
    AyzOld = atan2f(rEstOld[VEC3_Y],rEstOld[VEC3_Z]);
    
    euler[VEC3_X] = asinf(rEst[VEC3_X]);
    euler[VEC3_Y] = asinf(rEst[VEC3_Y]);
    euler[VEC3_Z] = asinf(rEst[VEC3_Z]);

    timeOld = time;
    
    //printf("rEst: X %f    Y %f    Z %f\n",REst[VEC3_X], REst[VEC3_Y], REst[VEC3_Z]);
    
    printf("%f;%f;%f;\t%f;%f;%f;\t%f;%f;%f;\t%f;%f;%f;\t%f;%f;%f;\t%f\n",rEst[VEC3_X], rEst[VEC3_Y], rEst[VEC3_Z], euler[VEC3_X], euler[VEC3_Y], euler[VEC3_Z], gyro[VEC3_X], gyro[VEC3_Y], gyro[VEC3_Z], accel[VEC3_X], accel[VEC3_Y], accel[VEC3_Z], rAcc[VEC3_X], rAcc[VEC3_Y], rAcc[VEC3_Z], timedelay);
  
  
//   // normalize acceleration
//   RAccel = sqrt(mpu->calibratedAccel[VEC3_X]*mpu->calibratedAccel[VEC3_X] + 
// 		    mpu->calibratedAccel[VEC3_Y]*mpu->calibratedAccel[VEC3_Y] +
// 		    mpu->calibratedAccel[VEC3_Z]*mpu->calibratedAccel[VEC3_Z] );
//   normAccel[VEC3_X] = mpu->calibratedAccel[VEC3_X] / RAccel;
//   normAccel[VEC3_Y] = mpu->calibratedAccel[VEC3_Y] / RAccel;
//   normAccel[VEC3_Z] = mpu->calibratedAccel[VEC3_Z] / RAccel;
//   
//   if(first){
//     printf("first estimation\n");
//     for(i=VEC3_X;i<(VEC3_Z+1);i++){
//       REst[i] = //normAccel[i];
//       REstOld[i] = REst[i];
//     }
//     first = 0;
//   }else{
//     // estimation
//     AxzOld = atan2f(REstOld[VEC3_X],REstOld[VEC3_Z]);
//     AyzOld = atan2f(REstOld[VEC3_Y],REstOld[VEC3_Z]);
//     Axz = AxzOld + (-mpu->rawGyro[VEC3_X] - rawGyroOld[VEC3_X])*DEGREE_TO_RAD/2 * timedelay;
//     Ayz = AyzOld + (-mpu->rawGyro[VEC3_Y] - rawGyroOld[VEC3_Y])*DEGREE_TO_RAD/2 * timedelay;
//     rGyro[VEC3_X] = sinf(Axz) / sqrt(1 + cosf(Axz)*cos(Axz) * tan(Ayz)*tan(Ayz));
//     rGyro[VEC3_Y] = sinf(Ayz) / sqrt(1 + cosf(Ayz)*cos(Ayz) * tan(Axz)*tan(Axz));
//    
//     if(REstOld[VEC3_Z]>=0){
//       rGyro[VEC3_Z] = sqrt(1 - rGyro[VEC3_X]*rGyro[VEC3_X] - rGyro[VEC3_Y]*rGyro[VEC3_Y]);
//     }else{
//       rGyro[VEC3_Z] = -sqrt(1 - rGyro[VEC3_X]*rGyro[VEC3_X] - rGyro[VEC3_Y]*rGyro[VEC3_Y]);
//     }
//     
//     RREst = sqrt(REst[VEC3_X]*REst[VEC3_X] + REst[VEC3_Y]*REst[VEC3_Y] + REst[VEC3_Z]*REst[VEC3_Z]);
//     for(i=VEC3_X;i<(VEC3_Z+1);i++){
//       REst[i] = ((normAccel[i] + rGyro[i] * wGyro)/(1 + wGyro)) / RREst;
//       REstOld[i] = REst[i];
//     }    
//     timeOld = time;
//     
//     //printf("rEst: X %f    Y %f    Z %f\n",REst[VEC3_X], REst[VEC3_Y], REst[VEC3_Z]);
//     
//     printf("%f;%f;%f;%f;%f;%f;%f\n",REst[VEC3_X], REst[VEC3_Y], REst[VEC3_Z], normAccel[VEC3_X], normAccel[VEC3_Y], normAccel[VEC3_Z], timedelay);
     
  }
  
}

int estimate_position2(double time){
  int i;
  
  double timedelay = (time-timeOld);
  
  
  unsigned char data_read[6];
  
  //read accel
  i2c_read(0x69, 0x3B, 6, data_read); 
  rawAccel[VEC3_X] = ((short)data_read[0] << 8) | data_read[1];
  rawAccel[VEC3_Y] = ((short)data_read[2] << 8) | data_read[3];
  rawAccel[VEC3_Z] = ((short)data_read[4] << 8) | data_read[5];
  //read gyro
  i2c_read(0x69, 0x43, 6, data_read);
  rawGyro[VEC3_X] = ((short)data_read[0] << 8) | data_read[1];
  rawGyro[VEC3_Y] = ((short)data_read[2] << 8) | data_read[3];
  rawGyro[VEC3_Z] = ((short)data_read[4] << 8) | data_read[5];
  
  //read mag
//   i2c_read(0x69, 0x49, 8, data_read);
//  // i2c_read(0x0C, 0x03, 6, data_read);
//   
//   /* AK8963 doesn't have the data read error bit. */
//   if (!(data_read[0] & AKM_DATA_READY) || (data_read[0] & AKM_DATA_OVERRUN))
//       return -2;
//   if (data_read[7] & AKM_OVERFLOW)
//       return -3;
// 
//   rawMag[VEC3_X] = ((short)data_read[2] << 8) | data_read[1];
//   rawMag[VEC3_Y] = ((short)data_read[4] << 8) | data_read[3];
//   rawMag[VEC3_Z] = ((short)data_read[6] << 8) | data_read[5];
  
  
//   printf("%d;%d;%d;%f\n", rawMag[VEC3_X], rawMag[VEC3_Y], rawMag[VEC3_Z], timedelay);
  
 
  
  
  for(i=VEC3_X;i<(VEC3_Z+1);i++){
    accel[i] = (float)rawAccel[i] / (ACCSENS);
    gyro[i] = (float)rawGyro[i] / (GYROSENS);
    //mag[i] = (float)rawMag[i] / (MAGSENS);
    mag[i] = virtMag[i];
    angDispGyro[i] = gyro[i] *DEGREE_TO_RAD * timedelay; // *DEGREE_TO_RAD
    
  }
  
  // normalize acceleration
  accNorm = sqrt(accel[VEC3_X]*accel[VEC3_X] + accel[VEC3_Y]*accel[VEC3_Y] + accel[VEC3_Z]*accel[VEC3_Z] );
  magNorm = sqrt(mag[VEC3_X]*mag[VEC3_X] + mag[VEC3_Y]*mag[VEC3_Y] + mag[VEC3_Z]*mag[VEC3_Z]);
  for(i=VEC3_X;i<(VEC3_Z+1);i++){
    rAcc[i] = accel[i] / accNorm;
    rMag[i] = mag[i] / magNorm;
  }
  
  
  if(first){
    printf("first estimation\n");
    //first row
    dcm[0] = rMag[0];
    dcm[1] = rMag[1];
    dcm[2] = rMag[2];
    //second row
    dcm[3] = rAcc[1] * rMag[2] - rAcc[2] * rMag[1];
    dcm[4] = rAcc[2] * rMag[0] - rAcc[0] * rMag[2];
    dcm[5] = rAcc[0] * rMag[1] - rAcc[1] * rMag[0];
    //third row
    dcm[6] = rAcc[0];
    dcm[7] = rAcc[1];
    dcm[8] = rAcc[2];
    
    first = 0;
  }else{
    angDispAcc[0] = (dcm[7] * (rAcc[2] - dcm[8])) - (dcm[8] * (rAcc[1] - dcm[7]));
    angDispAcc[1] = (dcm[8] * (rAcc[0] - dcm[6])) - (dcm[6] * (rAcc[2] - dcm[8]));
    angDispAcc[2] = (dcm[6] * (rAcc[1] - dcm[7])) - (dcm[7] * (rAcc[0] - dcm[6]));
    
    angDispMag[0] = (dcm[1] * (rMag[2] - dcm[2])) - (dcm[2] * (rMag[1] - dcm[1]));
    angDispMag[1] = (dcm[2] * (rMag[0] - dcm[0])) - (dcm[0] * (rMag[2] - dcm[2]));
    angDispMag[2] = (dcm[0] * (rMag[1] - dcm[1])) - (dcm[1] * (rMag[0] - dcm[0]));
    
    for(i=VEC3_X;i<(VEC3_Z+1);i++){
      angDisp[i] = (sa * angDispAcc[i] + sg * angDispGyro[i] + sm * angDispMag[i]) / (sa + sg + sm);
    }
    
    dcm[6] = dcm[6] + angDisp[1]*dcm[8] - angDisp[2]*dcm[7];
    dcm[7] = dcm[7] + angDisp[2]*dcm[6] - angDisp[0]*dcm[8];
    dcm[8] = dcm[8] + angDisp[0]*dcm[7] - angDisp[1]*dcm[6];
    
//     dcm[3] = dcm[3] + angDisp[1]*dcm[5] - angDisp[2]*dcm[4];
//     dcm[4] = dcm[4] + angDisp[2]*dcm[3] - angDisp[0]*dcm[5];
//     dcm[5] = dcm[5] + angDisp[0]*dcm[4] - angDisp[1]*dcm[3];
//     
//     dcm[0] = dcm[4]*dcm[8] - dcm[5]*dcm[7];
//     dcm[1] = dcm[5]*dcm[6] - dcm[3]*dcm[8];
//     dcm[2] = dcm[3]*dcm[7] - dcm[4]*dcm[6];
    
    dcm[0] = dcm[0] + angDisp[1]*dcm[2] - angDisp[2]*dcm[1];
    dcm[1] = dcm[1] + angDisp[2]*dcm[0] - angDisp[0]*dcm[2];
    dcm[2] = dcm[2] + angDisp[0]*dcm[1] - angDisp[1]*dcm[0];
    
    dcm[3] = dcm[7]*dcm[2] - dcm[8]*dcm[1];
    dcm[4] = dcm[8]*dcm[0] - dcm[6]*dcm[2];
    dcm[5] = dcm[6]*dcm[1] - dcm[7]*dcm[0];
    
    
    
//     orthErr = dcm[3]*dcm[7] - dcm[6]*dcm[4] + dcm[6]*dcm[1] - dcm[0]*dcm[7] + dcm[0]*dcm[4] - dcm[3]*dcm[0];
//     for(i=0;i<3;i++){
//       xOrth[i] = dcm[i*3] - orthErr/2*dcm[i*3+1];
//       yOrth[i] = dcm[i*3+1] - orthErr/2*dcm[i*3];
//     }
    
    orthErr = dcm[0]*dcm[1] - dcm[3]*dcm[4] + dcm[6]*dcm[7];
    for(i=0;i<3;i++){
      xOrth[i] = dcm[i*3] - orthErr/2 * dcm[i*3+1];
      yOrth[i] = dcm[i*3+1] - orthErr/2 * dcm[i*3];
    }
    
    zOrth[0] = xOrth[1]*yOrth[2] - xOrth[2]*yOrth[1];
    zOrth[1] = xOrth[2]*yOrth[0] - xOrth[0]*yOrth[2];
    zOrth[2] = xOrth[0]*yOrth[1] - xOrth[1]*yOrth[0];
    
    for(i=0;i<3;i++){
      xNorm[i] = 0.5 * (3 - (xOrth[0]*xOrth[0] + xOrth[1]*xOrth[1] + xOrth[2]*xOrth[2])) * xOrth[i];
      yNorm[i] = 0.5 * (3 - (yOrth[0]*yOrth[0] + yOrth[1]*yOrth[1] + yOrth[2]*yOrth[2])) * yOrth[i];
      zNorm[i] = 0.5 * (3 - (zOrth[0]*zOrth[0] + zOrth[1]*zOrth[1] + zOrth[2]*zOrth[2])) * zOrth[i];
    }
    
//     dcm[0] = xNorm[0];
//     dcm[1] = xNorm[1];
//     dcm[2] = xNorm[2];
//     dcm[3] = yNorm[0];
//     dcm[4] = yNorm[1];
//     dcm[5] = yNorm[2];
//     dcm[6] = zNorm[0];
//     dcm[7] = zNorm[1];
//     dcm[8] = zNorm[2];

    dcm[0] = xNorm[0];
    dcm[3] = xNorm[1];
    dcm[6] = xNorm[2];
    dcm[1] = yNorm[0];
    dcm[4] = yNorm[1];
    dcm[7] = yNorm[2];
    dcm[2] = zNorm[0];
    dcm[5] = zNorm[1];
    dcm[8] = zNorm[2];
    
    euler[VEC3_Y] = -asinf(dcm[6]);
    euler[VEC3_X] = asinf(dcm[7]) / cosf(euler[VEC3_Y]);
    euler[VEC3_Z] = asinf(dcm[3]) / cosf(euler[VEC3_Y]);
  }
  
  virtMag[VEC3_X] = cosf(euler[VEC3_Y]);
  virtMag[VEC3_Y] = 0;
  virtMag[VEC3_Z] = -sinf(euler[VEC3_Y]);
  
  
  printf("%f;%f;%f;\t%f;%f;%f;\t%f;%f;%f;\t%f;%f;%f;\t%f\n", euler[VEC3_X], euler[VEC3_Y], euler[VEC3_Z], gyro[VEC3_X], gyro[VEC3_Y], gyro[VEC3_Z], accel[VEC3_X], accel[VEC3_Y], accel[VEC3_Z], mag[VEC3_X], mag[VEC3_Y], mag[VEC3_Z], timedelay);
  
  
  timeOld = time;
  
  return 0;
}





/* These next two functions convert the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from InvenSense's MPL.
 */
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}
