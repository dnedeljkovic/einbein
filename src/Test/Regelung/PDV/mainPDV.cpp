#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include "einbein/Regelung/PDV/CSPDV.hpp"
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
	CSPDV csPDV(1);
	
	// initialize hardware
	csPDV.start();
	
	printf("Init done\n");

	
	while (running) {
		std::cout << "\n------------------------------------------------------------------------------ "  << std::endl; 
		
		
		//F_0
 		std::cout << "F_0		: " << csPDV.pDV.getOut_F_0().getSignal().getValue() << "  [m]" << std::endl;

		
		
		sleep(1);
	}
	
	

	csPDV.stop();
	csPDV.join();
	
	
	
	return 0;	
}
