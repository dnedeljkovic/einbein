#include <iostream>
#include <unistd.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <getopt.h>
#include <errno.h>

#include "../src/mpu9250/mpu9250/mpu9250.h"
#include "../src/mpu9250/glue/linux_glue.h"


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

struct eMPL_output_s {
    long quat[4];
    int quat_accuracy;
    int gyro_status;
    int accel_status;
    int compass_status;
    int nine_axis_status;
    inv_time_t nine_axis_timestamp;
};


void read_loop(unsigned int sample_rate);
void print_fused_euler_angles(mpudata_t *mpu);
void register_sig_handler();
void sigint_handler(int sig);

int done;

using namespace eeros;
using namespace eeros::logger;
using namespace eeros::hal;


int main(int argc, char *argv[]){
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

    read_loop(sample_rate);

    mpu9250_exit();

    log.trace() << "ended";
    exit (1);
}

void read_loop(unsigned int sample_rate)
{
    unsigned long loop_delay;
    mpudata_t mpu;

    memset(&mpu, 0, sizeof(mpudata_t));

    if (sample_rate == 0)
	    return;

    loop_delay = (1000 / sample_rate) - 2;

    log.trace() << "\nEntering read loop (ctrl-c to exit)\n\n";

    linux_delay_ms(loop_delay);


    while (!done) {
	if (mpu9250_read(&mpu) == 0) {
		print_fused_euler_angles(&mpu);
		//print_calibrated_accel(&mpu);
	}

	linux_delay_ms(loop_delay);
    }
}

void print_fused_euler_angles(mpudata_t *mpu)
{
   /* log.trace() << "\rX: " << mpu->fusedEuler[VEC3_X] * RAD_TO_DEGREE 
	       << " Y: " << mpu->fusedEuler[VEC3_Y] * RAD_TO_DEGREE
	       << " Z: " << mpu->fusedEuler[VEC3_Z] * RAD_TO_DEGREE;*/
}

void print_calibrated_accel(mpudata_t *mpu)
{
    /*log.trace() << "\rX: " << mpu->calibratedAccel[VEC3_X] 
	       << " Y: " << mpu->calibratedAccel[VEC3_Y]
	       << " Z: " << mpu->calibratedAccel[VEC3_Z];*/
}

void register_sig_handler()
{
    struct sigaction sia;

    bzero(&sia, sizeof sia);
    sia.sa_handler = sigint_handler;

    if (sigaction(SIGINT, &sia, NULL) < 0) {
	   // log.error() << "sigaction(SIGINT)";
	    exit(1);
    } 
}

void sigint_handler(int sig)
{
    done = 1;
}