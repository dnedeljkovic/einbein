#include <iostream>
#include <unistd.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/Logger.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/hal/FlinkDevice.hpp>
#include <eeros/hal/FlinkDigIn.hpp>
#include <eeros/hal/FlinkDigOut.hpp>
#include <eeros/hal/FlinkFqd.hpp>
#include <eeros/hal/FlinkAnalogOut.hpp>
#include <eeros/hal/FlinkPwm.hpp>


#define FPGA_DEVICE "/dev/flink0"

// unique-id of flink subdevices
#define FQD_ID	2
#define DAC_ID	3
#define GPIO_ID	4
#define IMU_ID	5
#define PWM_ID	6

using namespace eeros;
//using namespace eeros::control;
using namespace eeros::logger;
using namespace eeros::hal;
//using namespace eeros::safety;

/*static volatile bool running = true;
void sig_handler(int signum){
    running = false;
}*/

int main(int argc, char *argv[]){
//    signal(SIGINT, sig_handler);

    StreamLogWriter w(std::cout);
    Logger<LogWriter>::setDefaultWriter(&w);
    Logger<LogWriter> log;
    
    // Start
    log.info() << "Application einbein started...";

    // Initialize hardware
    log.info() << "Initializing hardware";
    HAL& hal = HAL::instance();
    
    log.trace() << "  Creating device structure...";
    FlinkDevice onBoard (FPGA_DEVICE);
    
    /*
     * GPIO
     */
    // Safety Stop
/*    FlinkDigIn safetyStop("safetyStop", &onBoard, GPIO_ID, 33);
    // Motor 1
    FlinkDigOut mot1IN1("mot1IN1", &onBoard, GPIO_ID, 0);
    FlinkDigOut mot1IN2("mot1IN2", &onBoard, GPIO_ID, 1);
    FlinkDigOut mot1IN3("mot1IN3", &onBoard, GPIO_ID, 2);
    FlinkDigOut mot1IN5("mot1IN5", &onBoard, GPIO_ID, 3);
    FlinkDigOut mot1OUT1("mot1OUT1", &onBoard, GPIO_ID, 5);
    FlinkDigOut mot1OUT2("mot1OUT2", &onBoard, GPIO_ID, 7);
    FlinkDigIn mot1INRET("mot1INRET", &onBoard, GPIO_ID, 4);		//GND
    FlinkDigIn mot1OUTRET1("mot1OUTRET1", &onBoard, GPIO_ID, 6);
    FlinkDigIn mot1OUTRET2("mot1OUTRET2", &onBoard, GPIO_ID, 8);
    FlinkDigIn mot1EncINDEX("mot1EncINDEX", &onBoard, GPIO_ID, 27);	//??
    FlinkDigIn mot1EncSUPRET("mot1EncSUPRET", &onBoard, GPIO_ID, 28);	//GND
    // Motor 2
    FlinkDigOut mot2IN1("mot2IN1", &onBoard, GPIO_ID, 9);
    FlinkDigOut mot2IN2("mot2IN2", &onBoard, GPIO_ID, 10);
    FlinkDigOut mot2IN3("mot2IN3", &onBoard, GPIO_ID, 11);
    FlinkDigOut mot2IN5("mot2IN5", &onBoard, GPIO_ID, 12);
    FlinkDigOut mot2OUT1("mot2OUT1", &onBoard, GPIO_ID, 14);
    FlinkDigOut mot2OUT2("mot2OUT2", &onBoard, GPIO_ID, 16);
    FlinkDigIn mot2INRET("mot2INRET", &onBoard, GPIO_ID, 13);		//GND
    FlinkDigIn mot2OUTRET1("mot2OUTRET1", &onBoard, GPIO_ID, 15);
    FlinkDigIn mot2OUTRET2("mot2OUTRET2", &onBoard, GPIO_ID, 17);
    FlinkDigIn mot2EncINDEX("mot2EncINDEX", &onBoard, GPIO_ID, 29);	//??
    FlinkDigIn mot2EncSUPRET("mot2EncSUPRET", &onBoard, GPIO_ID, 30);	//GND
    // Motor 3
    FlinkDigOut mot3IN1("mot3IN1", &onBoard, GPIO_ID, 18);
    FlinkDigOut mot3IN2("mot3IN2", &onBoard, GPIO_ID, 19);
    FlinkDigOut mot3IN3("mot3IN3", &onBoard, GPIO_ID, 20);
    FlinkDigOut mot3IN5("mot3IN5", &onBoard, GPIO_ID, 21);
    FlinkDigOut mot3OUT1("mot3OUT1", &onBoard, GPIO_ID, 23);
    FlinkDigOut mot3OUT2("mot3OUT2", &onBoard, GPIO_ID, 25);
    FlinkDigIn mot3INRET("mot3INRET", &onBoard, GPIO_ID, 22);		//GND
    FlinkDigIn mot3OUTRET1("mot3OUTRET1", &onBoard, GPIO_ID, 24);
    FlinkDigIn mot3OUTRET2("mot3OUTRET2", &onBoard, GPIO_ID, 26);
    FlinkDigIn mot3EncINDEX("mot3EncINDEX", &onBoard, GPIO_ID, 31);	//??
    FlinkDigIn mot3EncSUPRET("mot3EncSUPRET", &onBoard, GPIO_ID, 32);	//GND
    // LEDs
    FlinkDigOut LEDOnTheSpot("LEDOnTheSpot", &onBoard, GPIO_ID, 34);
    FlinkDigOut LEDManual("LEDManual", &onBoard, GPIO_ID, 35);
    FlinkDigOut LEDAutonomous("LEDAutonomous", &onBoard, GPIO_ID, 36);
    FlinkDigOut LEDRes1("LEDRes1", &onBoard, GPIO_ID, 37);
    FlinkDigOut LEDRes2("LEDRes2", &onBoard, GPIO_ID, 38);
    FlinkDigOut LEDRes3("LEDRes3", &onBoard, GPIO_ID, 39);
    FlinkDigOut LEDRes4("LEDRes4", &onBoard, GPIO_ID, 40);
    FlinkDigOut LEDRes5("LEDRes5", &onBoard, GPIO_ID, 41);
    // Switches
    FlinkDigIn SwitchStart("SwitchStart", &onBoard, GPIO_ID, 42);
    FlinkDigIn SwitchStop("SwitchStop", &onBoard, GPIO_ID, 43);
    FlinkDigIn SwitchRes1("SwitchRes1", &onBoard, GPIO_ID, 44);
    FlinkDigIn SwitchRes2("SwitchRes2", &onBoard, GPIO_ID, 45);
    FlinkDigIn SwitchRes3("SwitchRes3", &onBoard, GPIO_ID, 46);
    FlinkDigIn SwitchMode1("SwitchMode1", &onBoard, GPIO_ID, 47);
    FlinkDigIn SwitchMode2("SwitchMode2", &onBoard, GPIO_ID, 48);
    FlinkDigIn SwitchMode3("SwitchMode3", &onBoard, GPIO_ID, 49);
    FlinkDigIn SwitchMode4("SwitchMode4", &onBoard, GPIO_ID, 50);
    FlinkDigIn SwitchMode5("SwitchMode5", &onBoard, GPIO_ID, 51);
    FlinkDigIn SwitchMode6("SwitchMode6", &onBoard, GPIO_ID, 52);
*/    
    // FPGA Digital IO
    // TODO gpio 53 to 65
    
    /*
     * FQD
     */
    // Motor 1
/*    FlinkFqd mot1Enc("mot1Enc", &onBoard, FQD_ID, 0); //TODO scale, offset, initValue
    // Motor 2
    FlinkFqd mot2Enc("mot2Enc", &onBoard, FQD_ID, 1); //TODO scale, offset, initValue
    // Motor 3
    FlinkFqd mot3Enc("mot3Enc", &onBoard, FQD_ID, 2); //TODO scale, offset, initValue
*/    
    /*
     * DAC
     */
    // Motor 1
/*    FlinkAnalogOut mot1DAC("mot1DAC", &onBoard, DAC_ID, 0); //TODO scale, offset
    // Motor 2
    FlinkAnalogOut mot2DAC("mot2DAC", &onBoard, DAC_ID, 1); //TODO scale, offset
    // Motor 2
    FlinkAnalogOut mot3DAC("mot3DAC", &onBoard, DAC_ID, 2); //TODO scale, offset
*/    
    /*
     * PWM
     */
    // LiDAR 
//    FlinkPwm lidarPWM("lidarPWM", &onBoard, PWM_ID, 0); //not used (TODO scale, offset)
    
    /*
     * IMU
     */
    // TODO set SPI and I2C for the MPU9250
    
    
    /*
     * Add to HAL
     */
//    hal.addPeripheralInput(&safetyStop);
    //TODO hier weiter machen!!!!!
    
      
    log.trace() << "ended";
    exit (1);
}