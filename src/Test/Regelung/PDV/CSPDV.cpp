
#include "einbein/Regelung/PDV/CSPDV.hpp"
#include "einbein/Regelung/PDV/PDV.hpp"


using namespace einbein;
using namespace eeros::math;
using namespace eeros::control;


CSPDV::CSPDV(double ts) :

    //init Block
    pDV(3.141592653589793e+02, 6.157521601035994e+02, 0.8, 1),		//double kp_Vk, double kv_Vk, double m_Vk, double Ts
  
    timedomain("Main time domain", ts, true) {
      
    xIst_0.setValue(0.1);
    xSoll_0.setValue(0.15);
     
      
    //connect
    pDV.getIn_xSoll_0().connect(xSoll_0.getOut());
    pDV.getIn_xIst_0().connect(xIst_0.getOut());
   
    
    //Add Block to run-Methode
    timedomain.addBlock(&xIst_0);
    timedomain.addBlock(&xSoll_0);
    timedomain.addBlock(&pDV);

    printf("\nAdd Block to run-Methode\n");
   }
  
void CSPDV::start(){
    timedomain.start();
    printf("start CSPDV\n");
}  


void CSPDV::stop(){
    timedomain.stop();
}
 

void CSPDV::join(){
    timedomain.join();
} 
 