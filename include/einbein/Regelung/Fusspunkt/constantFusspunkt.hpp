#ifndef EINBEIN_CONSTANTFUSSPUNKT_HPP
#define EINBEIN_CONSTANTFUSSPUNKT_HPP


//includes
#include <eeros/math/Matrix.hpp>
using namespace eeros::math;


namespace einbein{
  
  /*///------------------------------------------------------------------------------
   * 	Konstante Vorwärtskinemtaik
   *///------------------------------------------------------------------------------
  
  
    //static constant ist für alle erzeugten Objekten gleich 
  
    static const double pi  	= 3.14159265358979323846; 	// [m]

    // geometrische Parameter Roboter (CAD)
    static const double n 		= 0.013; 	// [m]
    static const double m 		= 0.04275; 	// [m]
    static const double enc_zusatz 	= 0.025; 	// [m]
    static const double c 		= 0.0750257; 	// [m]
    static const double a 		= 0.128196762829644;	// norm(rP5i_P1i_Mi)[m]
    static const double l_Unterschenkel = 0.4;	// [m]
    static const double epsilon 	= 2*pi/3; 	// Versatzwinkel Motor [m]    
    
 //------------------------------------------------------------------------------------------------
    //konstante Vektoren    
    static const eeros::math::Matrix<3,1,double> rP1i_1_Mi 	= eeros::math::Matrix<3,1,double> ({0.112,
										    0.0,
										    -0.0276}); 	//Abstand vom KS {1} zum KS {P1}, dargestellt im KS {1} [m]
    
    static const eeros::math::Matrix<3,1,double> rP5i_1_Mi 	= eeros::math::Matrix<3,1,double> ({0.2255,
										    0.0,
										    0.032});	//Abstand vom KS {1} zum Punkt P5, dargestellt im KS {1} [m]
    
    static const eeros::math::Matrix<3,1,double> r1_IMU_Mi 	= eeros::math::Matrix<3,1,double> ({0.0,
										    0.0,
										    -0.0876553});//Abstand vom KS {IMU} zum KS {P1}, dargestellt im KS {Mi} [m] (da bei allen Mi gleich--> Rotation um z-Achse)
    
    static const eeros::math::Matrix<3,1,double> rP3i_P1i_P1i = eeros::math::Matrix<3,1,double> ({0.129955,
										    0.0,
										    0.00342046});//Abstand vom KS {P1} zum Punkt P3, dargestellt im KS {P1} [m]
    
    static const eeros::math::Matrix<3,1,double> P1i_Mi       = r1_IMU_Mi + rP1i_1_Mi; 			 //Von {IMU} zu Punkt 1, dargestellt im KS{1} [m]
    static const eeros::math::Matrix<3,1,double> P5i_Mi       = r1_IMU_Mi + rP5i_1_Mi;             		 //Von {IMU} zu Punkt 5 dargestellt im KS{1} [m]
    static const eeros::math::Matrix<3,1,double> rP5i_P1i_Mi  = P5i_Mi- P1i_Mi;                		 //Abstand vom KS {P1} zum Punkt P5, dargestellt im KS {1} [m]
  
    static const eeros::math::Matrix<3,1,double> c_vec  = eeros::math::Matrix<3,1,double> ({c,
										    0.0,
										    0.0});
    
    static const eeros::math::Matrix<3,1,double> n_vec  = eeros::math::Matrix<3,1,double> ({n,
										    0.0,
										    0.0});
  
    static const eeros::math::Matrix<3,1,double> rf41_IMU  = eeros::math::Matrix<3,1,double> ({-0.0190,
											0.0,
											-0.0325});

    static const eeros::math::Matrix<3,1,double> rf42_IMU  = eeros::math::Matrix<3,1,double> ({0.0095,
											-0.016454482671904,
											-0.0325});    

    static const eeros::math::Matrix<3,1,double> rf43_IMU  = eeros::math::Matrix<3,1,double> ({0.0095,
											0.016454482671904,
											-0.0325}); 
 
    
    
 //------------------------------------------------------------------------------------------------
    //konstante Rotationsmatrizen
    //KS {1}
    static const eeros::math::Matrix<3,3,double> R_IMU_1_R  = eeros::math::Matrix<3,3,double> ({1.0,	0.0,	0.0,	 
												0.0,	1.0,	0.0,
												0.0,	0.0, 	1.0}); 
        
    //KS {M1}
    static const eeros::math::Matrix<3,3,double> R_1_M1_R = eeros::math::Matrix<3,3,double>({1.0,	0.0,	0.0,	 
											     0.0,	1.0,	0.0,
											     0.0,	0.0, 	1.0});		//TODO Ausgabe kontrollieren und andere T anpassen
    
    static const eeros::math::Matrix<3,3,double> R_IMU_M1_R  =  R_IMU_1_R*R_1_M1_R; 
    
    
    //KS {M2}
    static const eeros::math::Matrix<3,3,double> R_M1_M2_R = eeros::math::Matrix<3,3,double>({cos(epsilon),	-sin(epsilon), 	0, 	 
											      sin(epsilon),	cos(epsilon), 	0,
											      0.0,		0.0, 		1.0}).transpose();
    
    
    static const eeros::math::Matrix<3,3,double> R_IMU_M2_R  =  R_IMU_M1_R*R_M1_M2_R;
    
    
    //KS {M3}
    static const eeros::math::Matrix<3,3,double> R_M2_M3_R = eeros::math::Matrix<3,3,double>({cos(epsilon),	-sin(epsilon),	 0,	 
											    sin(epsilon),	cos(epsilon),	 0,
											    0.0,		0.0, 		1.0}).transpose();
    
    static const eeros::math::Matrix<3,3,double> R_IMU_M3_R = R_IMU_M2_R*R_M2_M3_R;
    
    //Hilfstransformationen
    static const eeros::math::Matrix<3,3,double> R_M1_IMU_R = R_IMU_M1_R.transpose(); 
    static const eeros::math::Matrix<3,3,double> R_M2_IMU_R = R_IMU_M2_R.transpose(); 
    static const eeros::math::Matrix<3,3,double> R_M3_IMU_R = R_IMU_M3_R.transpose(); 
    
    
    
     //zeitabhängige Rotationsmatrizen
    static const eeros::math::Matrix<3,3,double> R_0_IMU_R;


   /*///------------------------------------------------------------------------------
   * 	Konstante Direkte transformierte Jacobimatrix
   *///------------------------------------------------------------------------------
    static const eeros::math::Matrix<3,1,double> e_x  = eeros::math::Matrix<3,1,double> ({1.0,
											  0.0,
											  0.0,});
    
    static const eeros::math::Matrix<3,1,double> e_y  = eeros::math::Matrix<3,1,double> ({0.0,
											  1.0,
											  0.0,});    
    
    static const eeros::math::Matrix<3,1,double> e_z  = eeros::math::Matrix<3,1,double> ({0.0,
											  0.0,
											  1.0,});    
    
    
    
    
    
    
    
    
    /* --> wenn auf Objekt zugegriffen werden sollte
    class Fusspunkt{
  public:
    static Fusspunkt* instance(){ 
      if( instanz == 0 )
	  instanz = new Fusspunkt();
      return instanz;
    }
    private:
     static Fusspunkt *instanz;
     Fusspunkt(){
       R_1_test.eye();
     }
    public:
    eeros::math::Matrix<3,3, double> R_1_test;
    //eeros::math::Matrix<3,3> R_1_tes;
    //R_1 = R_1_tes::eye();
    
  };
  */
}
#endif // CONSTANTSFUSSPUNKT_HPP



