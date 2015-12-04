
#include "einbein/Regelung/Base2Tool/CSVorKin.hpp"


using namespace einbein;


CSVorKin::CSVorKin(double ts) :
  
    timedomain("Main time domain", ts, true) {
    //set Input  
    alpha1.setValue(0);
    beta1.setValue(0);
    gamma1.setValue(0);
    enc1.setValue(0.1);
    enc2.setValue(0.05);
    enc3.setValue(0);
    FVec_Fuss.setValue({-3,1,0});


    
    //connect
    vorKin.getIn_alpha1().connect(alpha1.getOut());
    vorKin.getIn_beta1().connect(beta1.getOut());
    vorKin.getIn_gamma1().connect(gamma1.getOut());
      
    vorKin.getIn_enc1().connect(enc1.getOut());
    vorKin.getIn_enc2().connect(enc2.getOut());
    vorKin.getIn_enc3().connect(enc3.getOut());
    
    vorKin.getIn_F_Fuss_vec().connect(FVec_Fuss.getOut());

   
    
    //Add Block to run-Methode
    timedomain.addBlock(&alpha1);
    timedomain.addBlock(&beta1);
    timedomain.addBlock(&gamma1);
    timedomain.addBlock(&enc1);
    timedomain.addBlock(&enc2);
    timedomain.addBlock(&enc3);
    timedomain.addBlock(&FVec_Fuss);
    timedomain.addBlock(&vorKin);
    
    printf("\nAdd Block to run-Methode\n");
  }
  
void CSVorKin::start(){
    timedomain.start();
    printf("start CSVorKin\n");
}  


void CSVorKin::stop(){
    timedomain.stop();
}
 

void CSVorKin::join(){
    timedomain.join();
} 
 