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
static void calibrate_data(mpudata_t *mpu);
static int data_fusion(mpudata_t *mpu);
static unsigned short inv_row_2_scale(const signed char *row);
static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);


int use_accel_cal;
caldata_t accel_cal_data;

int first = 1;
float RAccel, RREst;
float Axz, Ayz, AxzOld, AyzOld;
float normAccel[3];
float REst[3], REstOld[3];
float rGyro[3], rawGyroOld[3];
double timeOld = 0.0;
int wGyro = 10; //can be chosen from 5 to 20;

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
	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL /*| INV_XYZ_COMPASS*/)) {
		printf("\nmpu_set_sensors() failed\n");
		return -1;
	}
//**************

	if (mpu_init(NULL)) {
		printf("\nmpu_init() failed\n");
		return -1;
	}

	if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL /*| INV_XYZ_COMPASS*/)) {
		printf("\nmpu_set_sensors() failed\n");
		return -1;
	}

	if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
		printf("\nmpu_configure_fifo() failed\n");
		return -1;
	}

	if (mpu_set_sample_rate(sample_rate)) {
		printf("\nmpu_set_sample_rate() failed\n");
		return -1;
	}

	if (dmp_load_motion_driver_firmware()) {
		printf("\ndmp_load_motion_driver_firmware() failed\n");
		return -1;
	}

	if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {
		printf("\ndmp_set_orientation() failed\n");
		return -1;
	}

	if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL
						| DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {
		printf("\ndmp_enable_feature() failed\n");
		return -1;
	}
 
	if (dmp_set_fifo_rate(sample_rate)) {
		printf("\ndmp_set_fifo_rate() failed\n");
		return -1;
	}

	if (mpu_set_dmp_state(1)) {
		printf("\nmpu_set_dmp_state(1) failed\n");
		return -1;
	}

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
	short sensors;
	unsigned char more;

	if (!data_ready())
		return -1;

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
	if (mpu9250_read_dmp(mpu) != 0)
		return -1;

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

	return (status == (MPU_INT_STATUS_DATA_READY | MPU_INT_STATUS_DMP | MPU_INT_STATUS_DMP_0));
}

void calibrate_data(mpudata_t *mpu)
{
  if (use_accel_cal) {
    mpu->calibratedAccel[VEC3_X] = -(short)(((long)mpu->rawAccel[VEC3_X] * (long)ACCEL_SENSOR_RANGE)
		      / (long)accel_cal_data.range[VEC3_X]);

    mpu->calibratedAccel[VEC3_Y] = (short)(((long)mpu->rawAccel[VEC3_Y] * (long)ACCEL_SENSOR_RANGE)
		      / (long)accel_cal_data.range[VEC3_Y]);

    mpu->calibratedAccel[VEC3_Z] = (short)(((long)mpu->rawAccel[VEC3_Z] * (long)ACCEL_SENSOR_RANGE)
		      / (long)accel_cal_data.range[VEC3_Z]);
  }
  else {
    mpu->calibratedAccel[VEC3_X] = -mpu->rawAccel[VEC3_X];
    mpu->calibratedAccel[VEC3_Y] = mpu->rawAccel[VEC3_Y];
    mpu->calibratedAccel[VEC3_Z] = mpu->rawAccel[VEC3_Z];
  }
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
  
  dmpQuat[QUAT_W] = (float)mpu->rawQuat[QUAT_W];
  dmpQuat[QUAT_X] = (float)mpu->rawQuat[QUAT_X];
  dmpQuat[QUAT_Y] = (float)mpu->rawQuat[QUAT_Y];
  dmpQuat[QUAT_Z] = (float)mpu->rawQuat[QUAT_Z];
  
  quaternionNormalize(dmpQuat);
  
  // if first ..... einfügen !!!!!!!!!!!!!!!
  
  vx = ax * deltaT;
  vy = ay * deltaT;
  vz = az * deltaT;
  px = 0.5 * ax * deltaT*deltaT;
  py = 0.5 * ay * deltaT*deltaT;  
  pz = 0.5 * az * deltaT*deltaT;


  
  //printf("Position: X %f   Y %f   Z %f\n",px,py,pz);
  printf("%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f;%f\n",ax,ay,az,vx,vy,vz,px,py,pz,deltaT, dmpQuat[QUAT_W],dmpQuat[QUAT_X],dmpQuat[QUAT_Y],dmpQuat[QUAT_Z]);
  //printf("%f;%f;%f;%f\n",dmpQuat[QUAT_W],dmpQuat[QUAT_X],dmpQuat[QUAT_Y],dmpQuat[QUAT_Z]);
  
  timeOld = mpu->dmpTimestamp;
  
  
}




/* Position estimation is an example of the algorithmus 
 * from http://www.starlino.com/imu_guide.html
 */
void estimate_position(mpudata_t *mpu, unsigned long loop_delay, double time){
 /* float RAccel, RREst;
  float Axz, Ayz, AxzOld, AyzOld;
  float normAccel[3];
  float REst[3], REstOld[3];
  float rGyro[3], rawGyroOld[3];
  double timeOld;
  int wGyro = 5; //can be chosen from 5 to 20;*/
  int i;
  
  double timedelay = (time-timeOld);
  
  // normalize acceleration
  RAccel = sqrt(mpu->calibratedAccel[VEC3_X]*mpu->calibratedAccel[VEC3_X] + 
		    mpu->calibratedAccel[VEC3_Y]*mpu->calibratedAccel[VEC3_Y] +
		    mpu->calibratedAccel[VEC3_Z]*mpu->calibratedAccel[VEC3_Z] );
  normAccel[VEC3_X] = mpu->calibratedAccel[VEC3_X] / RAccel;
  normAccel[VEC3_Y] = mpu->calibratedAccel[VEC3_Y] / RAccel;
  normAccel[VEC3_Z] = mpu->calibratedAccel[VEC3_Z] / RAccel;
  
  if(first){
    printf("first estimation\n");
    for(i=VEC3_X;i<(VEC3_Z+1);i++){
      REst[i] = normAccel[i];
      REstOld[i] = REst[i];
    }
    first = 0;
  }else{
    // estimation
    AxzOld = atan2f(REstOld[VEC3_X],REstOld[VEC3_Z]);
    AyzOld = atan2f(REstOld[VEC3_Y],REstOld[VEC3_Z]);
    Axz = AxzOld + (-mpu->rawGyro[VEC3_X] - rawGyroOld[VEC3_X])*DEGREE_TO_RAD/2 * timedelay;
    Ayz = AyzOld + (-mpu->rawGyro[VEC3_Y] - rawGyroOld[VEC3_Y])*DEGREE_TO_RAD/2 * timedelay;
    rGyro[VEC3_X] = sinf(Axz) / sqrt(1 + cosf(Axz)*cos(Axz) * tan(Ayz)*tan(Ayz));
    rGyro[VEC3_Y] = sinf(Ayz) / sqrt(1 + cosf(Ayz)*cos(Ayz) * tan(Axz)*tan(Axz));
   
    if(REstOld[VEC3_Z]>=0){
      rGyro[VEC3_Z] = sqrt(1 - rGyro[VEC3_X]*rGyro[VEC3_X] - rGyro[VEC3_Y]*rGyro[VEC3_Y]);
    }else{
      rGyro[VEC3_Z] = -sqrt(1 - rGyro[VEC3_X]*rGyro[VEC3_X] - rGyro[VEC3_Y]*rGyro[VEC3_Y]);
    }
    
    RREst = sqrt(REst[VEC3_X]*REst[VEC3_X] + REst[VEC3_Y]*REst[VEC3_Y] + REst[VEC3_Z]*REst[VEC3_Z]);
    for(i=VEC3_X;i<(VEC3_Z+1);i++){
      REst[i] = ((normAccel[i] + rGyro[i] * wGyro)/(1 + wGyro)) / RREst;
      REstOld[i] = REst[i];
    }    
    timeOld = time;
    
    //printf("rEst: X %f    Y %f    Z %f\n",REst[VEC3_X], REst[VEC3_Y], REst[VEC3_Z]);
    
    printf("%d;%d;%d;%f;%f;%f;%f;%f;%f;%d;%d;%d;%f\n", mpu->rawGyro[VEC3_X], mpu->rawGyro[VEC3_Y], mpu->rawGyro[VEC3_Z], REst[VEC3_X], REst[VEC3_Y], REst[VEC3_Z], normAccel[VEC3_X], normAccel[VEC3_Y], normAccel[VEC3_Z], mpu->calibratedAccel[VEC3_X], mpu->calibratedAccel[VEC3_Y], mpu->calibratedAccel[VEC3_Z],timedelay);
     
  }
  
}


void mpu9250_set_accel_cal(caldata_t *cal)
{
  int i;
  long bias[3];

  if (!cal) {
    use_accel_cal = 0;
    return;
  }

  memcpy(&accel_cal_data, cal, sizeof(caldata_t));

  for (i = 0; i < 3; i++) {
    if (accel_cal_data.range[i] < 1)
      accel_cal_data.range[i] = 1;
    else if (accel_cal_data.range[i] > ACCEL_SENSOR_RANGE)
      accel_cal_data.range[i] = ACCEL_SENSOR_RANGE;

    bias[i] = -accel_cal_data.offset[i];
  }
  

  //DIESEN TEIL LOESCHEN!!!!!!!!!!!
  int debug_on = 1;
  if (debug_on) {
    printf("\naccel cal (range : offset)\n");

    for (i = 0; i < 3; i++)
      printf("%d : %d\n", accel_cal_data.range[i], accel_cal_data.offset[i]);
  }//bis hier!!!!!!!!
  
  

  mpu_set_accel_bias(bias);

  use_accel_cal = 1;
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
