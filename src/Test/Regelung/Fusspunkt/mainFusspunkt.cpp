#include <iostream>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>

#include <eeros/hal/HAL.hpp>
#include <eeros/safety/SafetySystem.hpp>
#include <eeros/logger/StreamLogWriter.hpp>
#include <eeros/logger/SysLogWriter.hpp>


#include <einbein/Regelung/Fusspunkt/constantFusspunkt.hpp>
#include <einbein/templates_function.hpp>


using namespace eeros;
using namespace eeros::hal;
//using namespace eeros::control;
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
	//TODO lw 0.001 durch dt ersetzten und testen
	//ControlSystem ControlSys(0.01);
	
	// initialize hardware
	//ControlSys.start();
	
	printf("Init done\n");
	double test_s = rP1i_1_Mi(0);
	std::cout << test_s <<  std::endl;
	
 

	
	while (running) {
/*
		std::cout << "------------------------------------------------------------------------------ "  << std::endl; 
			
		
		//Beschleunigung
 		std::cout << "Beschleunigung 		: " << ControlSys.trajektorie.getOut_ddx_d().getSignal().getValue() << "  [m/s²]" << std::endl;

		//Geschwindigkeit
 		std::cout << "Geschwindigkeit 	: " << ControlSys.trajektorie.getOut_dx_d().getSignal().getValue() << "  [m/s]" << std::endl;
		
		//Position
 		std::cout << "Position		: " << ControlSys.trajektorie.getOut_x_d().getSignal().getValue() << "  [m]" << std::endl;
*/
		sleep(0.1);
	}
	
	
	
	
	//ControlSys.stop();
	//ControlSys.join();
	
	
	
	return 0;
	
}
