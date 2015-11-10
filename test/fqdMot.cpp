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
#define FQD_ID	0
//#define DAC_ID	3
//#define GPIO_ID	2
//#define IMU_ID	5
//#define PWM_ID	6

using namespace eeros;

using namespace eeros::logger;
using namespace eeros::hal;


int main(int argc, char *argv[]){
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
    
    

    FlinkDigIn TasterRes3("TasterRes3", &onBoard, GPIO_ID, 0);
    FlinkDigIn TasterStart("TasterStart", &onBoard, GPIO_ID, 1);
    FlinkDigOut LEDRes5("LEDRes5", &onBoard, GPIO_ID, 2);
    
    hal.addPeripheralInput(&TasterRes3);
    hal.addPeripheralInput(&TasterStart);
    hal.addPeripheralOutput(&LEDRes5);
    
    bool toggle = true;
        
    while(TasterRes3.get()){
  
      if(!TasterStart.get()){
	toggle = !toggle;
	sleep(1);
      }
      
      LEDRes5.set(toggle);
    }
      
    log.trace() << "ended";
    exit (1);
}