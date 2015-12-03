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
  F_Fuss_vec 	= in_F_Fuss_vec.getSignal().getValue();

  
 
//---------------------------- Fusspunkt berechnen (Vorwärtskinematik) -------------------

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
   
  //Rotationsmatrix "Quasigelenk x-y-z" 
   R_0_IMU_R = R_0_IMU_R_rotX.createRotX(alpha1)*(R_0_IMU_R_rotX.createRotY(beta1)*R_0_IMU_R_rotX.createRotZ(gamma1));
  
  //Fusspunkt im {0}
  Pf_0 = R_0_IMU_R*Pf_IMU;
 

  
  
//------------- Motorenkraft berechnen (direkt transformierte Jacobimatrix)---------------
  
  //Kraft an Oberschenkel P3
  calculateFPf2F3i(F31_IMU, F32_IMU, F33_IMU, F_Fuss_vec, ek1_IMU, ek2_IMU, ek3_IMU);
  
  
  //Transformation der Kraft in das KS {Mi}
  F31_M1 = R_M1_IMU_R * F31_IMU;
  F32_M2 = R_M2_IMU_R * F32_IMU;
  F33_M3 = R_M3_IMU_R * F33_IMU;
  
  
  //Kraft an Motoren FM1, FM2, FM3 als Skalar
  calculateF3i2FMi(FM1, F31_M1, h1, sigma_1);
  calculateF3i2FMi(FM2, F32_M2, h2, sigma_2);
  calculateF3i2FMi(FM3, F33_M3, h3, sigma_3);
  
  FMsoll(0) = FM1;
  FMsoll(1) = FM2;
  FMsoll(2) = FM3; 
  
  printf("FMsoll(0) %f,  FMsoll(1)%f,  FMsoll(2) %f \n", FMsoll(0), FMsoll(1), FMsoll(2)  );
  
//-----------------------------  set Output ----------------------------------------------- 

  out_Pf_IMU.getSignal().setValue(Pf_IMU);
  out_Pf_0.getSignal().setValue(Pf_0);
  out_FMsoll.getSignal().setValue(FMsoll); 
  
  //set Timestamp
  out_Pf_IMU.getSignal().setTimestamp(in_alpha1.getSignal().getTimestamp());
  out_Pf_0.getSignal().setTimestamp(in_alpha1.getSignal().getTimestamp());
  out_FMsoll.getSignal().setTimestamp(in_alpha1.getSignal().getTimestamp());
   
  //set Name
  out_Pf_IMU.getSignal().setName("Pf_IMU [m]");
  out_Pf_0.getSignal().setName("Pf_0 [m]");
  out_FMsoll.getSignal().setName("FMi [N]");
  
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


//Berechnung Kraft, welche dem Oberschenkel übergeben werden. Rückgabe per Reference
void VorKin::calculateFPf2F3i(Vector3 &F31_IMU, Vector3 &F32_IMU, Vector3 &F33_IMU, Vector3 &F_Fuss_vec, Vector3 ek1_IMU,Vector3 ek2_IMU, Vector3 ek3_IMU){
    
    F_x  = F_Fuss_vec(0);
    F_y  = F_Fuss_vec(1);
    F_z  = F_Fuss_vec(2);
  
    printf("ek1_IMU(0) %f, ek1_IMU(1) %f, ek1_IMU(2) %f\n", ek1_IMU(0) , ek1_IMU(1) , ek1_IMU(2));
    printf("ek2_IMU(0) %f, ek2_IMU(1) %f, ek2_IMU(2) %f\n", ek2_IMU(0) , ek2_IMU(1) , ek2_IMU(2));
    printf("ek3_IMU(0) %f, ek3_IMU(1) %f, ek3_IMU(2) %f\n", ek3_IMU(0) , ek3_IMU(1) , ek3_IMU(2));
    
    //A-Matrix
    a11	= e_x.transpose()*ek1_IMU;
    a12 = e_x.transpose()*ek2_IMU;
    a13 = e_x.transpose()*ek3_IMU;
    a21 = e_y.transpose()*ek1_IMU;
    a22 = e_y.transpose()*ek2_IMU;
    a23 = e_y.transpose()*ek3_IMU;
    a31 = e_z.transpose()*ek1_IMU;
    a32 = e_z.transpose()*ek2_IMU;
    a33 = e_z.transpose()*ek3_IMU;
    
    //printf("a11 %f, a12 %f, a13 %f, a21 %f, a22 %f, a23 %f, a31 %f, a32 %f, a33 %f\n", a11 , a12 , a13 , a21 , a22 , a23 , a31 , a32 , a33);
    printf("e_x.transpose()*ek1_IMU %f\n", e_x.transpose()*ek1_IMU);
    
    
    //A_Matrix
    A(0,0) = a22*a33-a23*a32;	A(0,1) = a13*a32-a12*a33;   A(0,2) = a12*a23-a13*a22;
    A(1,0) = a23*a31-a21*a33;	A(1,1) = a11*a33-a13*a31;   A(1,2) = a13*a21-a11*a23;
    A(2,0) = a21*a32-a22*a31;	A(2,1) = a12*a31-a11*a32;   A(2,2) = a11*a22-a12*a21;
    
    
    //det(A)
    det_A = A.det();
    A_invers = 1/det_A*A;
    
    
    //Kraft als Skalar am Punkt 3
    F3_skalar = A_invers*F_Fuss_vec;
    
  
    //Lösung kontrollieren
    if (isnan(F3_skalar(0)) == 1 ){
        F3_skalar(0) = 1e-3;
        printf("F_Motor_vec(0) --> NaN !!\n");
	}
   
         
 
    if (isnan(F3_skalar(1)) == 1 ){
        F3_skalar(1) = 1e-3;
        printf("F_Motor_vec(1) --> NaN !!\n");
	}
    
    
    if (isnan(F3_skalar(2)) == 1 ){
        F3_skalar(2) = 1e-3;
        printf("F_Motor_vec(2) --> NaN !!\n");
	}
    
    
    //Lösung Übergeben   
    F31_IMU = ek1_IMU *F3_skalar(0);
    F32_IMU = ek2_IMU *F3_skalar(1);
    F33_IMU = ek3_IMU *F3_skalar(2);
    
  
}//end calculateFPf2F3i 


//Berechnung Motorenkraft als Skalar. Rückgabe per Reference
void VorKin::calculateF3i2FMi(double& FMi, Vector3 F31_Mi, double hi, double sigma_i){
    //Moment durch F3i
    R_Mi_P1_R.createRotY(sigma_i);
    
    rP3iP1i_Mi = R_Mi_P1_R*rP3i_P1i_P1i;    
    tau1_Mi_vec.crossProduct(rP3iP1i_Mi, F31_Mi);
    
    //Projektion
    //tau_1 = dot(b_Mi, tau1_Mi_vec); mit b_Mi = [0 1 0]';
    tau_1 = tau1_Mi_vec(2);
    FMi = tau_1/hi; 
    
}//end calculateF3i2FMi


  


