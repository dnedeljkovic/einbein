#include <cmath>
#include <einbein/Regelung/Fusspunkt/constantFusspunkt.hpp>
#include <einbein/Regelung/Fusspunkt/VorKin.hpp>
#include <einbein/templates_function.hpp>


//namespace
using namespace constFusspunkt;
using namespace einbein;
using namespace eeros;
using namespace eeros::math;



VorKin::VorKin(){};


VorKin::~VorKin(){};

//Punkte Oberschenkel berechnen 
void VorKin::calculateGeoData(Vector3& P1i_Mi, Vector3& P2i_Mi, Vector3& P3i_Mi, Vector3& P5i_Mi, Vector3& P6i_Mi,Vector3& eP2P5_Mi, double& hi, double& sigma_i, double enc_i){
     z = enc_i + m + enc_zusatz;
      
     //Winkelberechnungen
      gamma   = atan(rP5i_P1i_Mi(3)/rP5i_P1i_Mi(1));
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




void VorKin::calculateP3i2pf(Vector3& Pf_IMU, Vector3& ek1_IMU, Vector3& ek2_IMU, Vector3& ek3_IMU, Vector3 P31_IMU,Vector3 P32_IMU, Vector3 P33_IMU){
    /*
    in dieser Funktion wird der Fusspunkt Pf in Abhängigkeit der Punkte P3i
    bestimmt (Delta-Prinzip)
    P31, P32, P33:     Befestigungspunkte, welche verändert werden können.
    Pf41, Pf42, Pf43:  Abstand vom Ende des Unterschenkel bis zum Fusspunkt

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
    x31 = P31_IMU(1);
    y31 = P31_IMU(2);
    z31 = P31_IMU(3);
    x32 = P32_IMU(1);
    y32 = P32_IMU(2);
    z32 = P32_IMU(3);
    x33 = P33_IMU(1);
    y33 = P33_IMU(2);
    z33 = P33_IMU(3);

    //Fusspunkte
    x41 = rf41_IMU(1);
    y41 = rf41_IMU(2);
    z41 = rf41_IMU(3);
    x42 = rf42_IMU(1);
    y42 = rf42_IMU(2);
    z42 = rf42_IMU(3);
    x43 = rf43_IMU(1);
    y43 = rf43_IMU(2);
    z43 = rf43_IMU(3);

 
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
    a1  = z1_*z1_ - z2_*z2_ + x1_*x1_ - x2_*x3_ + y1_*y1_ - y2_*y2_;
    a2  = z1_*z1_ - z3_*z3_ + x1_*x1_ - x3_*x3_ + y1_*y1_ - y3_*y3_;
    b1  = z1_ - z2_;
    b2  = z1_ - z3_;
    c11 = x1_ - x2_;
    c21 = x1_ - x3_;
    c12 = y1_ - y2_;
    c22 = y1_ - y3_;  
    
    
    
  
}
