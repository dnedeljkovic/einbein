#include "einbein/Regelung/Trajektorie/ControlSystem.hpp"
#include "einbein/Regelung/Trajektorie/Trajektorie.hpp"

using namespace einbein;


ControlSystem::ControlSystem(double ts) :
  
    timedomain("Main time domain", ts, true) {
      
    Ts.setValue(ts);
    x_start.setValue(-0.32);
    x_end.setValue(1.234);
    T_sprung.setValue(2.265);

    //connect
    trajektorie.getIn_Ts().connect(Ts.getOut());
    trajektorie.getIn_T_sprung().connect((T_sprung.getOut()));
    trajektorie.getIn_xf_start().connect(x_start.getOut());
    trajektorie.getIn_xf_end().connect(x_end.getOut());
    
    
    
    //Add Block to run-Methode
    timedomain.addBlock(&Ts);
    timedomain.addBlock(&x_start);
    timedomain.addBlock(&x_end);
    timedomain.addBlock(&T_sprung);
    timedomain.addBlock(&trajektorie);
    
    printf("\nAdd Block to run-Methode\n");
  }
  
void ControlSystem::start(){
    timedomain.start();
    printf("start ControlSystem\n");
}  


void ControlSystem::stop(){
    timedomain.stop();
}
 

void ControlSystem::join(){
    timedomain.join();
} 
 
 
 