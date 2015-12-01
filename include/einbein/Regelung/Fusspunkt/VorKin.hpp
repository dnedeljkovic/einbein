#ifndef EINBEIN_VORKIN_HPP
#define EINBEIN_VORKIN_HPP


#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>

using namespace eeros;
using namespace eeros::math;


namespace einbein{
  class VorKin : public eeros::control::Block{
    
    public:
	      VorKin();
	      virtual ~VorKin();
	      
	      
	      //define inputs
	      virtual eeros::control::Input<>& getIn_alpha1(){return in_alpha1;}
	      virtual eeros::control::Input<>& getIn_beta1(){return in_beta1;}
	      virtual eeros::control::Input<>& getIn_gamma1(){return in_gamma1;}
	      virtual eeros::control::Input<>& getIn_enc1(){return in_enc1;}	    
	      virtual eeros::control::Input<>& getIn_enc2(){return in_enc2;}	    
	      virtual eeros::control::Input<>& getIn_enc3(){return in_enc3;}
	      
	      
	      
	    
    protected: 
	      //define inputs
	      eeros::control::Input<> in_alpha1;	    
	      eeros::control::Input<> in_beta1;			  
	      eeros::control::Input<> in_gamma1;	  
	      eeros::control::Input<> in_enc1;	    
	      eeros::control::Input<> in_enc2;			  
	      eeros::control::Input<> in_enc3;
	      
    
    private:  
	      //Variablen für Methode calculateGeoData
	      double gamma, Lambda, l, beta, kappa, alpha, sigma_i, z, chi, eta, hi;	
	      eeros::math::Matrix<3,3> R_1_P1_R;
	      eeros::math::Matrix<3,3> R_P5_P6_R;
	      eeros::math::Matrix<3,1> rP2_p1_Mi;
	      eeros::math::Matrix<3,1> P2i_Mi, P6i_Mi, eP2P6_Mi, P3i_Mi;

	      
	      
	      //Variablen für Methode calculateP3i2pf
	      double x31, y31, z31, x32, y32, z32, x33, y33, z33;
	      double x41, y41, z41, x42, y42, z42, x43, y43, z43;
	      double x1_,x2_, x3_, y1_, y2_, y3_, z1_, z2_, z3_;	   
	      double a1, a2, b1, b2, c11, c21, c12, c22;
	      double lambda_1, lambda_2, lambda_3, lambda_4;
	      double p, q, r, xf, yf, zf, zf_positiv, zf_negativ; 
	      eeros::math::Matrix<3,1> P41_IMU, P42_IMU, P43_IMU;
	      eeros::math::Matrix<3,1> r43_1, r43_2, r43_3;
	      double l_1, l_2, l_3;
	      	
	      
	      //Methoden
	      virtual void calculateGeoData(Vector3& P1i_Mi, Vector3& P2i_Mi, Vector3& P3i_Mi, Vector3& P5i_Mi, Vector3& P6i_Mi,Vector3& eP2P5_Mi, double& hi, double& sigma_i, double enc_i);
	      virtual void calculateP3i2pf(Vector3& Pf_IMU, Vector3& ek1_IMU, Vector3& ek2_IMU, Vector3& ek3_IMU, Vector3 P31_IMU,Vector3 P32_IMU, Vector3 P33_IMU);
		
	      
	      //run
	      virtual void run();
	      
	      
	      
	      //Input
	      double alpha1, beta1, gamma1, enc1, enc2, enc3;
	      
	      //Werte von geoData
	      double P11_M1,P21_M1, P31_M1, P51_M1, h1, sigma_1;
	      double P12_M2,P22_M2, P32_M2, P52_M2, h2, sigma_2;
	      double P13_M3,P23_M3, P33_M3, P53_M3, h3, sigma_3;
    
  };//end class VorKIn
  
  
}//end namspace einbein

#endif // EINBEIN_VORKIN_HPP
