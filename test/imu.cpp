#include <iostream>
#include <unistd.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/core/System.hpp>


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>

extern "C"{
  #include "../src/mpu9250/mpu9250/mpu9250.h"
  #include "../src/mpu9250/glue/linux_glue.h"
}


long msg, data[9];
int8_t accuracy;
unsigned long timestamp;
float float_data[3] = {0};
typedef unsigned long inv_time_t;

#define BUF_SIZE        (256)
#define PACKET_LENGTH   (23)

#define PACKET_DEBUG    (1)
#define PACKET_QUAT     (2)
#define PACKET_DATA     (3)

#define INV_NEW_DATA 64

//define ACC offset in g
#define ACC_X_OFFSET 0.035
#define ACC_Y_OFFSET 0.011
#define ACC_Z_OFFSET 0.014

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::hal;


struct eMPL_output_s {
    long quat[4];
    int quat_accuracy;
    int gyro_status;
    int accel_status;
    int compass_status;
    int nine_axis_status;
    inv_time_t nine_axis_timestamp;
};

void read_loop(unsigned int sample_rate, Logger<LogWriter> *log);
void print_fused_euler_angles(mpudata_t *mpu, Logger<LogWriter> *log);
void print_calibrated_accel(mpudata_t *mpu, Logger<LogWriter> *log);
void register_sig_handler();
void sigint_handler(int sig);
int set_cal();

int done = 0;



int main(int argc, char *argv[]){
    
  char *accel_cal_file = NULL;

  StreamLogWriter w(std::cout);
  Logger<LogWriter>::setDefaultWriter(&w);
  w.show(~0); // show all messages
  Logger<LogWriter> log;

  // Start
  log.info() << "Application einbein started...";

  // Initialize hardware
  log.info() << "Initializing hardware";
  HAL& hal = HAL::instance();


  int i2c_bus = DEFAULT_I2C_BUS;
  int sample_rate = DEFAULT_SAMPLE_RATE_HZ;

  register_sig_handler();

  if (mpu9250_init(i2c_bus, sample_rate))
    exit(1);

  //set_cal();
  
  //set_acc_offset(ACC_X_OFFSET,ACC_Y_OFFSET,ACC_Z_OFFSET);
  
  
  /*
  unsigned char data_read[0];
  
  i2c_read(0x69, 0x19, 1, data_read);
  printf("ACC Config 2: %d\n",(int)data_read[0]);
  
  i2c_read(0x69, 0x19, 1, data_read);
  printf("mpu frequence: %d\n",(int)data_read[0]);
  */
  
  
  
  /*int wait = 20;
  while(wait > 0){
    printf("measurement starts in %d\n",wait);
    sleep(1);
    wait--;
  }*/

  read_loop(sample_rate, &log);
  

  mpu9250_exit();

  log.trace() << "ended";
  exit (1);
}

void read_loop(unsigned int sample_rate, Logger<LogWriter> *log)
{ 
  unsigned long loop_delay;
    mpudata_t mpu;

    memset(&mpu, 0, sizeof(mpudata_t));

    if (sample_rate == 0)
	    return;

    loop_delay = (1000 / sample_rate) - 2;

    log->trace() << "Entering read loop (ctrl-c to exit)";

    linux_delay_ms(loop_delay);
    
    double time;

    while (!done) {
      
//       if(mpu9250_read_reg(&mpu) == 0){
// 	estimate_position(/*&mpu, loop_delay, System::getTime()*/);
//       }
      
//       time = System::getTime();
      
//       estimate_position(System::getTime());
//       estimate_position2(System::getTime());
//       estimate_pos_madgwick(System::getTime());
      estimate_pos_mahony(System::getTime());
      
//       printf("time do calculate: %f\n",System::getTime()-time);
      
// 	if (mpu9250_read(&mpu) == 0) {
		//print_fused_euler_angles(&mpu, log);
		//print_calibrated_accel(&mpu, log);
// 		derivate_accel(&mpu);
		
	   //printf("timestamp: %ld\n",mpu.dmpTimestamp-timeOld);
// 	}
// 	timeOld = mpu.dmpTimestamp;
// 	linux_delay_ms(loop_delay);
// 	linux_delay_ms(1);
    }
}

void print_fused_euler_angles(mpudata_t *mpu, Logger<LogWriter> *log)
{
   log->trace() << "X: " << mpu->fusedEuler[VEC3_X] * RAD_TO_DEGREE 
	       << " Y: " << mpu->fusedEuler[VEC3_Y] * RAD_TO_DEGREE
	       << " Z: " << mpu->fusedEuler[VEC3_Z] * RAD_TO_DEGREE;
}

void print_calibrated_accel(mpudata_t *mpu, Logger<LogWriter> *log)
{
    log->trace() << "X: " << mpu->calibratedAccel[VEC3_X] 
	       << " Y: " << mpu->calibratedAccel[VEC3_Y]
	       << " Z: " << mpu->calibratedAccel[VEC3_Z];
}







/* **************************************************************************
 * 
 * ***************************  set_cal löschen  ***************************
 * 
 * **************************************************************************
 */
int set_cal()
{
  int i;
  FILE *f;
  char buff[32];
  long val[6];
  caldata_t cal; 
    
  f = fopen("./accelcal.txt", "r");

  if (!f) {
    printf("Default accelcal.txt not found\n");
    return 0;
  }
   
  memset(buff, 0, sizeof(buff));
	
  for (i = 0; i < 6; i++) {
    if (!fgets(buff, 20, f)) {
      printf("Not enough lines in calibration file\n");
      break;
    }

    val[i] = atoi(buff);

    if (val[i] == 0) {
      printf("Invalid cal value: %s\n", buff);
      break;
    }
  }
  
  fclose(f);

  if (i != 6) 
    return -1;

  cal.offset[0] = (short)((val[0] + val[1]) / 2);
  cal.offset[1] = (short)((val[2] + val[3]) / 2);
  cal.offset[2] = (short)((val[4] + val[5]) / 2);

  cal.range[0] = (short)(val[1] - cal.offset[0]);
  cal.range[1] = (short)(val[3] - cal.offset[1]);
  cal.range[2] = (short)(val[5] - cal.offset[2]);

  

  return 0;
  
}


void register_sig_handler()
{
  Logger<LogWriter> log;
  
  struct sigaction sia;

    bzero(&sia, sizeof sia);
    sia.sa_handler = sigint_handler;

    if (sigaction(SIGINT, &sia, NULL) < 0) {
	  log.error() << "sigaction(SIGINT)";
	  exit(1);
    } 
}

void sigint_handler(int sig)
{
    done = 1;
}