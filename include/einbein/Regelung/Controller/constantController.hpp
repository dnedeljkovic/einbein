#ifndef EINBEIN_CONSTANTCONTROLLER_HPP
#define EINBEIN_CONSTANTCONTROLLER_HPP


//includes
#include <eeros/math/Matrix.hpp>
using namespace eeros::math;


namespace einbein{
  
  /*///------------------------------------------------------------------------------
   * 	Konstante Controller
   *///------------------------------------------------------------------------------
  
  
    //static constant ist für alle erzeugten Objekten gleich 
    //Platenregelung (beta1 --> x-Richtung = Drehung um y)
    static const double beta1_soll 		= 0;
    static const double omega_y1_soll 		= 0;    
    static const double k_beta1 		= 10; 	
    static const double k_omega_y1 		= 50; 
    //Plattenregelung (alpha1 --> y-Richtung = Drehung um x)
    static const double alpha1_soll 		= 0; 
    static const double omega_x1_soll 		= 0;    
    static const double k_alpha1 		= 10; 	
    static const double k_omega_x1 		= 50; 
 
    
//Positionsregelung
    static const double T_Boden 		= 0.6;		//Periodenzeit Feder-Dämpfer-Masse System Fuss 
    
    //x-Richtung
    static const double x_Platte_soll 		= 0.5;         	//Sollvorgabe [m/s] //TODO Sollvorgabe über Terminal vorgeben
    static const double Kp_vx 			= 3; 		//Geschwindigkeitsregler
    static const double Kp_x 			= 0.05; 	//Positionsregler
    static const double vx_Platte_max 		= 0.5;       	//maximale erlaubte Geschwindigkeit [m/s^2]
    

    //y-Richtung
    static const double y_Platte_soll 		= 0.8;         	//Sollvorgabe [m/s] //TODO Sollvorgabe über Terminal vorgeben
    static const double Kp_vy 			= 3; 		//Geschwindigkeitsregler
    static const double Kp_y 			= 0.05; 	//Positionsregler
    static const double vy_Platte_max 		= 0.5;       	//maximale erlaubte Geschwindigkeit [m/s^2]


 
}
#endif // CONSTANTSFUSSPUNKT_HPP



