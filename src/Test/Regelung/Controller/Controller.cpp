#include <cmath>
#include <einbein/Regelung/Base2Tool/constantFusspunkt.hpp>
#include <einbein/Regelung/Controller/Controller.hpp>
#include <einbein/templates_function.hpp>


//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;


//Konstruktor
Controller::Controller(){
    xf_end_0 	= 0;
    yf_end_0 	= 0;
    F_alpha1_y 	= 0;
    F_beta1_x 	= 0;
    F_h 	= 0;
    
    //Anfangszustand des Ausgang setzen
    out_xf_end_0.getSignal().setValue(xf_end_0);
    out_yf_end_0.getSignal().setValue(yf_end_0);
    out_F_beta1_x.getSignal().setValue(F_beta1_x); 
    out_F_alpha1_y.getSignal().setValue(F_alpha1_y); 
    out_F_h.getSignal().setValue(F_h);   
  
  
  
    //für Testzwecke
    Zustand = 0;
    point = 0;
};

//Destruktor
Controller::~Controller(){};



void Controller::run(){
//-----------------------------  set Input-------------------------------------------------
      //Zustand 	= in_Zustand.getSignal().getValue();
	
	//für Testzwecke --> erhöhung des Zustandes
	if (point >= 1){Zustand++; point = 0; printf("Zustand %d/n", Zustand);}
	else point ++;
      
      Ts 		= in_Ts.getSignal().getValue();
      VarZustand	= in_VarZustand.getSignal().getValue(); 
      DataImu	= in_DataImu.getSignal().getValue();   
      
      //Variablen zuweisen
      rz_Platte_max       = VarZustand(0);
      rx_Platte_Landung   = VarZustand(1);
      vx_Platte_Absprung  = VarZustand(3);

      ry_Platte_Landung   = VarZustand(2);
      vy_Platte_Absprung  = VarZustand(4); 

      rx_FussPlatte       = VarZustand(8);
      ry_FussPlatte       = VarZustand(9);
      rz_FussPlatte       = VarZustand(10);
      
      //Variablen Sensor zuweisen
      alpha1_ist	      = DataImu(3);
      beta1_ist	      = DataImu(4);
      omega_x1_ist	      = DataImu(9);
      omega_y1_ist	      = DataImu(10);
    
    //--------------------------- Zustandsmaschine -------------------------------------------
      
      switch (Zustand){
	
	case 0: //Fehler
	    F_h = 0;
	    xf_end_0   = 0;				yf_end_0   = 0;
	    F_beta1_x  = 0;        			F_beta1_z  = 0;
	    F_alpha1_y = 0;       			F_alpha1_z = 0;  
	    printf("error Controller.cpp: Zustand 0 wird angegeben\n");
	    
	    break;//case 0
	    
	    
	case 1: //Sinkflug
	    F_h = 0;
	    xf_end_0   = xf_end_intern;		yf_end_0   = yf_end_intern;
	    F_beta1_x  = 0;        			F_beta1_z  = 0;
	    F_alpha1_y = 0;       			F_alpha1_z = 0;
	    
	    //TODO PID-Regler für Höhenregelung einfügen. Siehe Simulink, Block Controller
		    
	    break;//case 1
	    
	    
	case 2:  //Landung
	    F_h = 0;
	    xf_end_0   = 0;				yf_end_0   = 0;
	    F_beta1_x  = 0;        			F_beta1_z  = 0;
	    F_alpha1_y = 0;       			F_alpha1_z = 0;
	    
	    break;//case 2
	    

	case 3: //Einziehen
	    F_h = 25;
	    xf_end_0 =  0;			yf_end_0 =  0;
	    

	    //Plattenregelung (beta1 --> x-Richtung = Drehung um y)
	    T_beta1 = k_beta1 *(beta1_soll-beta1_ist) + k_omega_y1 * (omega_y1_soll-omega_y1_ist);
	    F_beta1_x = -T_beta1/rz_FussPlatte;
	  
	    //Plattenregelung (alpha1 --> y-Richtung = Drehung um x)
	    T_alpha1 = k_alpha1 *(alpha1_soll-alpha1_ist) + k_omega_x1 * (omega_x1_soll-omega_x1_ist);
	    F_alpha1_y = T_alpha1/rz_FussPlatte;
	    
	    break;//case 3
	    
	    
	  case 4: //Scheitelpunkt am Boden
	    F_h = 0;
	    xf_end_0 =  0;			yf_end_0 =  0;
	    
	    //Plattenregelung (beta1 --> x-Richtung = Drehung um y)
	    T_beta1 = k_beta1 *(beta1_soll-beta1_ist) + k_omega_y1 * (omega_y1_soll-omega_y1_ist);
	    F_beta1_x = -T_beta1/rz_FussPlatte;
	  
	    //Plattenregelung (alpha1 --> y-Richtung = Drehung um x)
	    T_alpha1 = k_alpha1 *(alpha1_soll-alpha1_ist) + k_omega_x1 * (omega_x1_soll-omega_x1_ist);
	    F_alpha1_y = T_alpha1/rz_FussPlatte;
	      
	    break;//case 4
	    
	    
	    
	  case 5: //Schub
	    F_h = -25;
	    xf_end_0 =  0;			yf_end_0 =  0;
		  
	    //Plattenregelung (beta1 --> x-Richtung = Drehung um y)
	    T_beta1 = k_beta1 *(beta1_soll-beta1_ist) + k_omega_y1 * (omega_y1_soll-omega_y1_ist);
	    F_beta1_x = -T_beta1/rz_FussPlatte;
	  
	    //Plattenregelung (alpha1 --> y-Richtung = Drehung um x)
	    T_alpha1 = k_alpha1 *(alpha1_soll-alpha1_ist) + k_omega_x1 * (omega_x1_soll-omega_x1_ist);
	    F_alpha1_y = T_alpha1/rz_FussPlatte;
	    
	    break;//case 5
	    
	    
	    
	  case 6: //Absprung
	    F_h = 0;
	    xf_end_0   = 0;				yf_end_0   = 0;
	    F_beta1_x  = 0;        			F_beta1_z  = 0;
	    F_alpha1_y = 0;       			F_alpha1_z = 0;      
	    posx_calc = 0;         //nächste Position des Fusspunktes kann berechnet werden
	    posy_calc = 0;         //nächste Position des Fusspunktes kann berechnet werden
	    
	    break;//case 6
	    
	    
	    
	  case 7: //Steigflug
	    F_h = 0;
	    F_beta1_x  = 0;        			F_beta1_z  = 0;
	    F_alpha1_y = 0;       			F_alpha1_z = 0;
	    
	  //Positionsregelung x-Richtung
	    if(posx_calc == 0){/*
	      Positionsregelung: 
	      Positionsregelung berechnet eine
	      Sollgeschwindigkeit,welche der Geschwindigkeitregelung übergeben 
	      wird*/
	      vx_Platte_soll = Kp_vx * (x_Platte_soll - rx_Platte_Landung);
	    
	      //maximale Geschwindigkeit
	      if (vx_Platte_soll >= vx_Platte_max){
		  vx_Platte_soll = vx_Platte_max;
	      }//end if
	      if (vx_Platte_soll <= -vx_Platte_max){
		  vx_Platte_soll = -vx_Platte_max;
	      }// end if


	      //Geschwindigkeitsregelung
	      xf_end_intern = vx_Platte_Absprung * T_Boden/2 + Kp_x*(vx_Platte_Absprung - vx_Platte_soll);
		
	      posx_calc = 1;
	    }//end if posx_calc

	    
	    //Positionsregelung y-Richtung 
	    if(posy_calc == 0){/*
	      Positionsregelung: 
	      Positionsregelung berechnet eine
	      Sollgeschwindigkeit,welche der Geschwindigkeitregelung übergeben 
	      wird*/
	      vy_Platte_soll = Kp_vy * (y_Platte_soll - ry_Platte_Landung);
	      
	      //maximale Geschwindigkeit
	      if (vy_Platte_soll >= vy_Platte_max){
		  vy_Platte_soll = vy_Platte_max;
	      }//end if
	      if (vy_Platte_soll <= -vy_Platte_max){
		  vy_Platte_soll = -vy_Platte_max;
	      }// end if
      
	      // Geschwindigkeitsregelung
	      yf_end_intern = vy_Platte_Absprung * 0.3 + Kp_y*(vy_Platte_Absprung - vy_Platte_soll);
			
	    posx_calc = 1;
	    }//end if posx_calc
      
      
	    xf_end_0 =  xf_end_intern;
	    yf_end_0 =  yf_end_intern;
	    
	    break;//case 7
	    
	      
	  case 8: //Scheitelpunkt Luft
	    F_h = 0;
	    xf_end_0   = xf_end_intern;		yf_end_0   =  yf_end_intern;
	    F_beta1_x  = 0;        			F_beta1_z  = 0;
	    F_alpha1_y = 0;       			F_alpha1_z = 0;  

	    break;//case 8
	    
	  default:
	    F_h = 0;
	    xf_end_0   = 0;				yf_end_0   = 0;
	    F_beta1_x  = 0;        			F_beta1_z  = 0;
	    F_alpha1_y = 0;       			F_alpha1_z = 0;
	    printf("error Controller.cpp: Zustand default wird angegeben\n");
	    
	    
	    
	  }//end case
	

	
	
	
    //-----------------------------  set Output ----------------------------------------------- 

      out_xf_end_0.getSignal().setValue(xf_end_0);
      out_yf_end_0.getSignal().setValue(yf_end_0);
      out_F_beta1_x.getSignal().setValue(F_beta1_x); 
      out_F_alpha1_y.getSignal().setValue(F_alpha1_y); 
      out_F_h.getSignal().setValue(F_h);   
      
      //set Timestamp
      out_xf_end_0.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());
      out_yf_end_0.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());
      out_F_beta1_x.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());
      out_F_alpha1_y.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());
      out_F_h.getSignal().setTimestamp(in_Zustand.getSignal().getTimestamp());

      //set Name
      out_xf_end_0.getSignal().setName("Sollwert Fusspositoin x-Richtung [m]");
      out_yf_end_0.getSignal().setName("Sollwert Fusspositoin y-Richtung [m]");
      out_F_beta1_x.getSignal().setName("Kraft in x-Richtung für Körperhalteregelung [N]");
      out_F_alpha1_y.getSignal().setName("Kraft in y-Richtung für Körperhalteregelung [N]");
      out_F_h.getSignal().setName("Kraft in z-Richtung für den Sprung [N]");

    
}//end run






  


