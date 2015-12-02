#include <cmath>
#include <einbein/Regelung/Fusspunkt/constantFusspunkt.hpp>
#include <einbein/Regelung/Fusspunkt/VorKin.hpp>
#include <einbein/templates_function.hpp>



//namespace
using namespace einbein;
using namespace eeros;
using namespace eeros::math;



//Konstruktor
VorKin::VorKin(){

};

//Destruktor
VorKin::~VorKin(){};



void VorKin::run(){
//-----------------------------  set Input-------------------------------------------------
  alpha1	= in_alpha1.getSignal().getValue(); 
  beta1		= in_beta1.getSignal().getValue();   
  gamma1	= in_gamma1.getSignal().getValue();
  enc1 		= in_enc1.getSignal().getValue();
  enc2 		= in_enc2.getSignal().getValue();  
  enc3 		= in_enc3.getSignal().getValue();

 
//------------------------------------------------------------------------------

  //geometrische Punkte berechnen
  calculateGeoData(P21_M1, P31_M1, P61_M1, eP2P6_M1, h1, sigma_1, enc1);
  calculateGeoData(P22_M2, P32_M2, P62_M2, eP2P6_M2, h2, sigma_2, enc2);
  calculateGeoData(P23_M3, P33_M3, P63_M3, eP2P6_M3, h3, sigma_3, enc3);


  //Transoformation der P3i-Werte in das KS {IMU}.  R-Matrizen sind Konstante
  P31_IMU     =  R_IMU_M1_R * P31_M1;
  P32_IMU     =  R_IMU_M2_R * P32_M2;
  P33_IMU     =  R_IMU_M3_R * P33_M3;
  

  
  //Fusspunkt berechnen"IM {IMU}
  calculateP3i2pf(Pf_IMU, ek1_IMU, ek2_IMU, ek3_IMU,  P31_IMU,  P32_IMU,  P33_IMU);
   
  //Rotationsmatrix "Quasigelenk x-y-z" \\TODO ev. wird Matrix noch von links nach rechts berechnet!!!!!!
  R_0_IMU_R = R_0_IMU_R_rotX.createRotX(alpha1)*(R_0_IMU_R_rotX.createRotY(beta1)*R_0_IMU_R_rotX.createRotZ(gamma1));
  
  //Fusspunkt im {0}
  Pf_0 = R_0_IMU_R*Pf_IMU;
 
  
//-----------------------------  set Output------------------------------------------------- 

  out_Pf_IMU.getSignal().setValue(Pf_IMU);
  out_Pf_0.getSignal().setValue(Pf_0);
  
  //set Timestamp
  out_Pf_IMU.getSignal().setTimestamp(in_alpha1.getSignal().getTimestamp());
  out_Pf_0.getSignal().setTimestamp(in_alpha1.getSignal().getTimestamp());  
   
  //set Name
  out_Pf_IMU.getSignal().setName("Pf_IMU [m]");
  out_Pf_0.getSignal().setName("Pf_0 [m]");
  
}//end run



//Berechnet Punkte vom Oberschenkel. Rückgabewert per Reference
void VorKin::calculateGeoData(Vector3 &P2i_Mi, Vector3 &P3i_Mi,  Vector3 &P6i_Mi,Vector3 &eP2P6_Mi, double &hi, double &sigma_i, double enc_i){
      /*
       * In dieser Funktion werde die Punkte im Oberschenkel berechnet. Input ist jeweils der Encoderwert.
       */
    
      z = enc_i + m + enc_zusatz;
      
 
     //Winkelberechnungen
      gamma   = atan(rP5i_P1i_Mi(2)/rP5i_P1i_Mi(0));
      Lambda  = atan(n/z);
      l       = sqrt(n*n + z*z);
      beta    = acos((a*a + c*c - l*l)/(2*a*c));
      kappa   = acos((c*c + l*l - a*a)/(2*c*l));
      alpha   = kappa + Lambda;
      sigma_i = beta - gamma;
      
         
      //Rotationsmatrix um y-Achse von P1
      R_1_P1_R(0,0) = cos(sigma_i);
      R_1_P1_R(1,0) = 0.0;
      R_1_P1_R(2,0) = -sin(sigma_i);
      R_1_P1_R(0,1) = 0.0;
      R_1_P1_R(1,1) = 1.0;
      R_1_P1_R(2,1) = 0.0;
      R_1_P1_R(0,2) = sin(sigma_i);
      R_1_P1_R(1,2) = 0.0;
      R_1_P1_R(2,2) = cos(sigma_i);
      
      //P2 und P3
      rP2_p1_Mi = R_1_P1_R*c_vec;    
      P2i_Mi = P1i_Mi + rP2_p1_Mi;
      P3i_Mi = P1i_Mi + R_1_P1_R*rP3i_P1i_P1i;
      
      //P6
      chi = 2*pi-beta-alpha-pi/2;
      eta = (pi-chi-gamma);
      
      //Robtationsmatrix um P5
      R_P5_P6_R(0,0) = cos(eta);
      R_P5_P6_R(1,0) = 0.0;
      R_P5_P6_R(2,0) = -sin(eta);
      R_P5_P6_R(0,1) = 0.0;
      R_P5_P6_R(1,1) = 1.0;
      R_P5_P6_R(2,1) = 0.0;
      R_P5_P6_R(0,2) = sin(eta);
      R_P5_P6_R(1,2) = 0.0;
      R_P5_P6_R(2,2) = cos(eta);
            
      P6i_Mi = P5i_Mi + R_P5_P6_R*n_vec;
      
      //Einheitsvektor von P6 zu P2
      eP2P6_Mi = (P2i_Mi-P6i_Mi)/norm(P2i_Mi-P6i_Mi);

      //h
      hi = c*sin(alpha);  
      
      
}//end VorKin





//Berechnet Fusspunkt und deren Einheitsvektoren. Rückgabe per Reference
void VorKin::calculateP3i2pf(Vector3 &Pf_IMU, Vector3 &ek1_IMU, Vector3 &ek2_IMU, Vector3 &ek3_IMU, Vector3 P31_IMU,Vector3 P32_IMU, Vector3 P33_IMU){
    /*
    *in dieser Funktion wird der Fusspunkt Pf in Abhängigkeit der Punkte P3i
    *bestimmt (Delta-Prinzip)
    *P31, P32, P33:     Befestigungspunkte, welche verändert werden können.
    *Pf41, Pf42, Pf43:  Abstand vom Ende des Unterschenkel bis zum Fusspunkt

      o P31             o P3x
       \               /
        \             /
         \           / l_Bein
          \         /
           \       /
           P41---P4x
               |

    */
  
    //Variablen zuweisen
    //Befestigungspunkte
    x31 = P31_IMU(0);
    y31 = P31_IMU(1);
    z31 = P31_IMU(2);
    x32 = P32_IMU(0);
    y32 = P32_IMU(1);
    z32 = P32_IMU(2);
    x33 = P33_IMU(0);
    y33 = P33_IMU(1);
    z33 = P33_IMU(2);

    //Fusspunkte
    x41 = rf41_IMU(0);
    y41 = rf41_IMU(1);
    z41 = rf41_IMU(2);
    x42 = rf42_IMU(0);
    y42 = rf42_IMU(1);
    z42 = rf42_IMU(2);
    x43 = rf43_IMU(0);
    y43 = rf43_IMU(1);
    z43 = rf43_IMU(2);

 
    //Pf berechnen
    //Hilfsvariablen bestimmen
    x1_ = x31 + x41;
    x2_ = x32 + x42;
    x3_ = x33 + x43;
    y1_ = y31 + y41;
    y2_ = y32 + y42;
    y3_ = y33 + y43;
    z1_ = z31 + z41;
    z2_ = z32 + z42;
    z3_ = z33 + z43;


    //Variablen für Matrixdivision
    a1  = z1_*z1_ - z2_*z2_ + x1_*x1_ - x2_*x2_ + y1_*y1_ - y2_*y2_;
    a2  = z1_*z1_ - z3_*z3_ + x1_*x1_ - x3_*x3_ + y1_*y1_ - y3_*y3_;
    b1  = z1_ - z2_;
    b2  = z1_ - z3_;
    c11 = x1_ - x2_;
    c21 = x1_ - x3_;
    c12 = y1_ - y2_;
    c22 = y1_ - y3_;  
    
      
    //Lösung der Matrixdivision (vereinfacht)
    //xf = lambda_1 + lambda_2*zf;
    //yf = lambda_3 + lambda_4*zf;
    lambda_1 = (c22*a1-c12*a2)/(2*(c11*c22-c21*c12));
    lambda_2 = -(b1*c22-b2*c12)/(c11*c22-c21*c12);
    lambda_3 = (c11*a2-c21*a1)/(2*(c11*c22-c21*c12));
    lambda_4 = -(b2*c11-b1*c21)/(c11*c22-c21*c12);
    
        
    //zf berechnen (quadtratische Funktion): negativer realer Wert wird ausgewählt
    //Variable für quatratische Formel bestimmen: p*zf^2 + q*zf + r = 0
    p = lambda_2*lambda_2 + lambda_4*lambda_4 + 1;
    q = 2*(lambda_2*lambda_1-lambda_2*x1_ + lambda_4*lambda_3 - lambda_4*y1_ - z1_);
    r = lambda_1*lambda_1 + x1_*x1_ - 2*lambda_1*x1_ + lambda_3*lambda_3 + y1_*y1_  + z1_*z1_ - l_Unterschenkel*l_Unterschenkel - 2*lambda_3*y1_;
    zf_positiv = (-q + sqrt(q*q-4*p*r))/(2*p);
    zf_negativ = (-q - sqrt(q*q-4*p*r))/(2*p);

        
    //Lösung auswählen
    if (zf_positiv <= 0){
        zf = zf_positiv;
    }

    if (zf_negativ <= 0){
        zf = zf_negativ;
    }
    
    else{
        zf = 0;
	printf("Keine Lösung beim Berechnen von zf in VorKin Methode calculateP3i2pf"); //TODO Ausgabe über log
	//log.info() << "Application OmniMoBot started...";
    }//end if
        

    //Fusspunkt aus einzelnen Lösungen zusammensetzen
    //xf und yf
    xf = lambda_1 + lambda_2*zf;
    yf = lambda_3 + lambda_4*zf;
    Pf_IMU(0) = xf;
    Pf_IMU(1) = yf;
    Pf_IMU(2) = zf;
    
    
    
    //Der Einheitsvektor wird in Abhängigkeit des Fusspunktes, dem Vektor rf4i
    //und dem Punkt P3i des Unterschenkels bestimmt
    //P41 berechenen
    P41_IMU =  Pf_IMU - rf41_IMU;
    P42_IMU =  Pf_IMU - rf42_IMU;
    P43_IMU =  Pf_IMU - rf43_IMU;

    //Einheitsvektor der Beine berechenen
    ek1_IMU = (P31_IMU - P41_IMU)/norm(P31_IMU - P41_IMU);
    ek2_IMU = (P32_IMU - P42_IMU)/norm(P32_IMU - P42_IMU);
    ek3_IMU = (P33_IMU - P43_IMU)/norm(P33_IMU - P43_IMU);
    
//------------------------------------------------------------//    
    //Kontrolle der Berechnungen 
    
    //Kontrolle Punkt 31
    r43_1 = Pf_IMU-P31_IMU-rf41_IMU;
    l_1 = norm(r43_1);

    if (abs(l_1) > (l_Unterschenkel+1e-8)){
        printf("Fehler Berechnung l_1 in Funktion p3ipf ");//TODO Ausgabe über log
    }//end if
    
    //Kontrolle Punk 32
    r43_2 = Pf_IMU-P32_IMU-rf42_IMU;
    l_2 = norm(r43_2);
    if (abs(l_2) > (l_Unterschenkel+1e-8)) {
         printf("Fehler Berechnung l_2 in Funktion p3ipf ");//TODO Ausgabe über log
    }//end if

    //Kontrolle Punk 32
    r43_3 = Pf_IMU-P33_IMU-rf43_IMU;
    l_3 = norm(r43_3);
    if (abs(l_3) > (l_Unterschenkel+1e-8)) {
        printf("Fehler Berechnung l_3 in Funktion p3ipf ");//TODO Ausgabe über log
    }//end if  

    
}//end calculateP3i2pf

