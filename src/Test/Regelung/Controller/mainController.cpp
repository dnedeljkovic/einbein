#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include "einbein/Regelung/Controller/CSController.hpp"
#include <eeros/logger/SysLogWriter.hpp>



using namespace eeros;
using namespace eeros::hal;
using namespace eeros::control;
using namespace eeros::safety;
using namespace eeros::logger;
using namespace einbein;

// enable to stop the program with ctrl c
volatile bool running = true;

// handler on press on ctrl c that will set running to false
void signalHandler(int signum) {
	running = false;

}
	
	
int main() {
	//set the interrupt handler
	signal(SIGINT, signalHandler);
	
	//start the loggers
	StreamLogWriter w(std::cout);
	SysLogWriter s("delta");
	w.show(0);
	s.show();
	
	Logger<LogWriter>::setDefaultWriter(&w);	
	Logger<LogWriter> log('M');
	
		
	log.trace() << "Application sismplesystem started...";
	
	// create control system
	CSController csController(1);
	
	// initialize hardware
	csController.start();
	
	printf("Init done\n");

	
	while (running) {
		std::cout << "\n------------------------------------------------------------------------------ "  << std::endl; 
		
		
		//xf_end_0
 		std::cout << "xf_end_0		: " << csController.controller.getOut_xf_end_0().getSignal().getValue() << "  [m]" << std::endl;

		//yf_end_0
 		std::cout << "yf_end_0 		: " << csController.controller.getOut_yf_end_0().getSignal().getValue() << "  [m]" << std::endl;
		
		//getOut_F_alpha1_y
 		std::cout << "getOut_F_alpha1_y	: " << csController.controller.getOut_F_alpha1_y().getSignal().getValue() << "  [m]" << std::endl;
		
		//getOut_F_beta1_x
 		std::cout << "getOut_F_beta1_x	: " << csController.controller.getOut_F_beta1_x().getSignal().getValue() << "  [m]" << std::endl;
		
		//getOut_F_h
 		std::cout << "getOut_F_h		: " << csController.controller.getOut_F_h().getSignal().getValue() << "  [m]" << std::endl;
		
		
		sleep(1);
	}
	
	

	csController.stop();
	csController.join();
	
	
	
	return 0;	
}
