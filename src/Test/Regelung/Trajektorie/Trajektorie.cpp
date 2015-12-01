#include <einbein/Regelung/Trajektorie//Trajektorie.hpp>
#include <valarray>
#include <iostream>


/*
 Trajektoriegenerator: Es wird die Beschleunigung, Geschwindigkeit und Position bestimmt.
 
 Ts:		Abtastzeit [sec]
 T_sprung:	Dauer bis Sollwert erricht werden soll [sec]
 x_end:		Sollwert [..]
 x_start:	Istwert [..]
 
 -Die Beschleunigung wird durch die Vorgabe der Zeit ermittelt.
 -Die Flugzeit wird mit einem Faktor verkürzt (Sicherheitsfaktor)
 -Die Beschleunigungszeit kann mit einem Faktor beeinflusst werden.
 
 */


using namespace einbein;


Trajektorie::Trajektorie() {
  //Ts.zero();
};


Trajektorie::~Trajektorie(){}


void Trajektorie::run(){
  
  Ts 		= in_Ts.getSignal().getValue();
  T_sprung 	= in_T_sprung.getSignal().getValue();
  xf_start 	= in_xf_start.getSignal().getValue();
  xf_end	= in_xf_end.getSignal().getValue();
  
  delta_xf 	= xf_end-xf_start;
  
  //Zurücksetzen und Parameter neu berechnen, wenn sich Vorgabe ändert (Anfangszustand)
  if(xf_end != xf_end_1){
    ddx_d_1 = 0;
    dx_d_1  = 0;
    x_d_1   = xf_start;
   
    //Berechnen der Sprungdauer --> vielfaches der Abtastzeit Ts
    //zusätzlich wird ein Sicherheitsfaktor eingefügt. So erreicht die Trajektorie vor der
    //Vorgegebenen Zeit s_soll.
    t_Luft    =  (round(T_sprung*0.75*1/Ts))*Ts;
    delta_x   =  delta_xf; 
    
    // Berechnen von gewünschter Beschleunigung bei vorgegebner Beschleunigungszeit
    t_b   	= (round(0.5*t_Luft*1/Ts)-1)*Ts;
    dd_x_calc 	= delta_x/(t_b*(t_Luft-t_b));
    
    // Vorzeichen muss bestimmt werden, da beim Berechnen von t_b das Vorzeichen verloren geht
    vorzeichen_ddx = signum(delta_x); 
    
    //Berechnen von Beschleunigungszeiten
    // t_b    = Beschleunigungszeit (gerundet)
    // t_Luft = Flusgzeit (gerundet)
    t1  = 0;
    t2  = t_b;
    t3  = t_Luft-t_b;
    t4  = t_Luft;
    
    
    punkte_pos  = round((t2-t1)/Ts);
    punkte_null = round((t3-t2)/Ts);
    if (punkte_null < 0){punkte_null = 0;}
    //punkte_neg  = round((t4-t3)/Ts);
    punkte_neg  = punkte_pos;	//negative Beschleunigung = positive Beschleunigung
				//ansonsten bleibende Geschwindigkeit 
    
    
    //Zustandsmachine (switch für Ausgang Beschleunigung) starten
    punkte  = 1;
    zustand = 1;
    xf_end_1 = xf_end;
 
    //printf("punkte_pos: %i  punkte_null: %i    punkte_neg: %i   Zustand: %i  t_b: %f  delta_x:  %f vorzeichen_ddx: %f ,  \n", punkte_pos, punkte_null, punkte_neg, zustand, t_b, dd_x_calc, vorzeichen_ddx);  
  }
  
 
 
 //Trajektorie Beschleunigung erzeugen
  switch(zustand){
    case 1: //positive Beschleunigung
      ddx_d = dd_x_calc;
      if(punkte >= punkte_pos){
	zustand = 2;
	punkte = 0;
      }
      break;
	
    case 2: //keine Beschleunigung
      ddx_d = 0;
      if(punkte >= punkte_null){
	zustand = 3;
	punkte = 0;
      }
      break;
      
    case 3: //negative Beschleunigung
      ddx_d = -dd_x_calc;
      //printf("Case 3: %i   ddx_d: %f zustand: %i \n ", punkte, ddx_d, zustand);
      if(punkte >= punkte_neg){
	zustand = 4;
	punkte = 0;
      }
      break;
      
    case 4:
      ddx_d = 0; 
      //printf("Case 4");
      break;
      
      
    default: 
      std::cout << "Fehler Switch in Trajektorie.cpp \n";
      break;
  }//end case


  punkte++;
  


  //Berechnen der Integrale
  if(delta_x != 0){
    dx_d = Ts*ddx_d + dx_d_1;
    x_d  = Ts*dx_d + x_d_1; 
  }
    
  //Parameter übergeben
  x_d_1 = x_d;
  dx_d_1 = dx_d;
  ddx_d_1 = ddx_d;
  //Ausgabe für Kontrolle
  //printf("x_d: %f  dx_d_1: %f    ddx_d_1: %f   \n", x_d, dx_d_1, ddx_d_1);    
  //printf("%f;%f;%f\n", x_d, dx_d_1, ddx_d_1);    
  
  
  
  //set Output
  out_ddx_d.getSignal().setValue(ddx_d);
  out_dx_d.getSignal().setValue(dx_d);
  out_x_d.getSignal().setValue(x_d);
  
  //set Timestamp
  out_ddx_d.getSignal().setTimestamp(in_Ts.getSignal().getTimestamp());
  out_dx_d.getSignal().setTimestamp(in_Ts.getSignal().getTimestamp());
  out_x_d.getSignal().setTimestamp(in_Ts.getSignal().getTimestamp());
  
  //set Name
  out_ddx_d.getSignal().setName("acceloration m/s²");
  out_dx_d.getSignal().setName("velocity m/s");
  out_x_d.getSignal().setName("position m/s²");
}