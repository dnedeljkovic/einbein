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
#include "../include/einbein/constants.hpp"


#define FPGA_DEVICE "/dev/flink0"

// unique-id of flink subdevices
#define FQD_ID	1
#define DAC_ID	0
#define GPIO_ID	3
//#define IMU_ID	5
//#define PWM_ID	6


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
    
    log.trace() << "  Creating device structure...";
    FlinkDevice onBoard (FPGA_DEVICE);
    
    
    FlinkDigIn TasterStop("TasterStop", &onBoard, GPIO_ID, 0, true);
    FlinkDigIn TasterStart("TasterStart", &onBoard, GPIO_ID, 1, true);
    FlinkDigOut LEDRes5("LEDRes5", &onBoard, GPIO_ID, 2, true);
    FlinkDigOut Mot3EncSupret("Mot3EncSupret", &onBoard, GPIO_ID, 3);
    FlinkDigOut Mot3IO1("Mot3Inhibit", &onBoard, GPIO_ID, 4);
    FlinkDigIn TasterRes1 ("TasterRes1", &onBoard, GPIO_ID, 5, true);
    FlinkDigIn TasterRes2 ("TasterRes2", &onBoard, GPIO_ID, 6, true);
    FlinkDigOut LEDRes4("LEDRes4", &onBoard, GPIO_ID, 7, true);
    FlinkDigOut LEDRes3("LEDRes3", &onBoard, GPIO_ID, 8, true);
    FlinkDigOut Mot3Inret("Mot3Inret", &onBoard, GPIO_ID, 9);
    
    FlinkAnalogOut resDAC("resDAC", &onBoard, DAC_ID, 3, dac3Scale, dac3Offset);
    
//    FlinkFqd mot3Enc("mot3Enc", &onBoard, FQD_ID, 0, fqd0scale, fqd0offset, fqd0delta);
    
    
    hal.addPeripheralInput(&TasterStop);
    hal.addPeripheralInput(&TasterStart);
    hal.addPeripheralOutput(&LEDRes5);
    hal.addPeripheralOutput(&Mot3EncSupret);
    hal.addPeripheralOutput(&Mot3IO1);
    hal.addPeripheralInput(&TasterRes1);
    hal.addPeripheralInput(&TasterRes2);
    hal.addPeripheralOutput(&LEDRes4);
    hal.addPeripheralOutput(&LEDRes3);
    hal.addPeripheralOutput(&Mot3Inret);
    
    hal.addPeripheralOutput(&resDAC);
    
//    hal.addPeripheralInput(&mot3Enc);
    
    
    bool toggle = false;
    bool inhibit = false;
    double volts = 0.0;
    
    Mot3IO1.set(inhibit);
    LEDRes3.set(inhibit);
    LEDRes5.set(toggle);
 
    Mot3EncSupret.set(false);	//set output to GND
    Mot3Inret.set(false);	//set output to GND
    resDAC.set(volts);		//set DAC to 0V
        
    while(!TasterRes2.get()){
  
      if(TasterStart.get()){
	toggle = !toggle;
	/*volts += 0.5;
	if(volts > 10.0) volts = -10;*/
	volts = 0.4;
	log.info() << "volts " << volts;
	
	inhibit = true;
	
	usleep(20000);
      } 
      
      if(TasterStop.get()){
	inhibit = false;
      }
      
      if(TasterRes1.get()){
	volts = -0.4;
	log.info() << "volts " << volts;
      }
      
      resDAC.set(volts);
      Mot3IO1.set(inhibit);
      LEDRes3.set(inhibit);
      LEDRes5.set(toggle);
      
    }

    resDAC.set(0);
    LEDRes5.set(false);
    Mot3IO1.set(false);
    LEDRes3.set(false);
      
    log.trace() << "ended";
    exit (1);
}