
#include "einbein/Regelung/Controller/CSController.hpp"



using namespace einbein;
using namespace eeros::math;
using namespace eeros::control;


CSController::CSController(double ts) :
  
    timedomain("Main time domain", ts, true) {
    
     
      
    //Zustand.setValue(zustand);
    Ts.setValue(ts);
    
/*    //Konstante VarZustand
    VarZustand(0).setValue(0);			//rz_Platte_max
    VarZustand(1).setValue(0);			//rx_Platte_Landung   
    VarZustand(2).setValue(0);			//ry_Platte_Landung
    VarZustand(3).setValue(0);			//vx_Platte_Absprung     
    VarZustand(4).setValue(0);			//vy_Platte_Absprung     
    
    VarZustand(8).setValue(0);			//rx_FussPlatte     
    VarZustand(9).setValue(0);			//ry_FussPlatte     
    VarZustand(10).setValue(0);			//rz_FussPlatte     
    
    //Konstante DataImu
    Imu(4).setValue(0);			//alpha1_ist      
    Imu(5).setValue(0);			//beta1_ist      
    Imu(9).setValue(0);			//omega_x1_ist      
    Imu(10).setValue(0);			//omega_y1_ist*/      

    //Konstante zuweisen      /0	/1	/2	/3	/4	/5	/6	/7	/8	/9	/10
    var = Matrix<11,1,double>({0,	2,	0.5, 	-0.589,	0.256,	0,	0,	0, 	0, 	0, 	0.5});
    VarZustand.setValue(var);
	
//			/0	/1	/2	/3	/4	/5	/6
//			/7	/8	/9	/10	/11	/12	/13    
    Imu = Matrix<14,1>({0,	0,	0, 	2,	1,	0,	0,
			0,	0,	-0.25, 	3,	0,	0,	0 });

    DataImu.setValue(Imu);	
    
        
    //connect
    controller.getIn_Zustand().connect(Zustand.getOut());
    controller.getIn_VarZustand().connect(VarZustand.getOut());
    controller.getIn_DataImu().connect(DataImu.getOut());
    
    //Add Block to run-Methode
    timedomain.addBlock(&Zustand);
    timedomain.addBlock(&VarZustand);
    timedomain.addBlock(&DataImu);
    timedomain.addBlock(&controller);

    printf("\nAdd Block to run-Methode\n");
   }
  
void CSController::start(){
    timedomain.start();
    printf("start CSController\n");
}  


void CSController::stop(){
    timedomain.stop();
}
 

void CSController::join(){
    timedomain.join();
} 
 