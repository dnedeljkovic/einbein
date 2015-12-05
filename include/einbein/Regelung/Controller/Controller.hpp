#ifndef EINBEIN_CONTROLLER_HPP
#define EINBEIN_CONTROLLER_HPP


#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/math/Matrix.hpp>

#include <einbein/Regelung/Controller/constantController.hpp>


using namespace eeros;
using namespace eeros::control;
using namespace eeros::math;


namespace einbein{
  class Controller : public eeros::control::Block{
    
    public:
	      Controller();
	      virtual ~Controller();
	     
	      //define inputs
	      virtual eeros::control::Input<int>& getIn_Zustand(){return in_Zustand;}
	      virtual eeros::control::Input<double>& getIn_Ts(){return in_Ts;}
	      virtual eeros::control::Input<Matrix<11,1>>& getIn_VarZustand(){return in_VarZustand;}
	      virtual eeros::control::Input<Matrix<14,1>>& getIn_DataImu(){return in_DataImu;}
	  
	      
	      //define outputs
	      virtual eeros::control::Output<double>& getOut_xf_end_0(){return out_xf_end_0;}
	      virtual eeros::control::Output<double>& getOut_yf_end_0(){return out_yf_end_0;}
	      virtual eeros::control::Output<double>& getOut_F_alpha1_y(){return out_F_alpha1_y;}
	      virtual eeros::control::Output<double>& getOut_F_beta1_x(){return out_F_beta1_x;}
	      virtual eeros::control::Output<double>& getOut_F_h(){return out_F_h;}
	 
	      
	    
    protected: 
	      //define inputs
	      eeros::control::Input<double> in_Ts;	
	      eeros::control::Input<int> in_Zustand;	
	      eeros::control::Input<Matrix<11,1>> in_VarZustand;	    
	      eeros::control::Input<Matrix<14,1>> in_DataImu;	
	      
	      
	      //define outputs
	      eeros::control::Output<double> out_xf_end_0;
	      eeros::control::Output<double> out_yf_end_0;	      
	      eeros::control::Output<double> out_F_alpha1_y;
	      eeros::control::Output<double> out_F_beta1_x;
	      eeros::control::Output<double> out_F_h;
  
	      
    private:  
//-----------------run Methode------------------------------------------------	      
	      virtual void run();
	      //input
	      Matrix<11,1> VarZustand;
	      Matrix<14,1> DataImu;
	      double Ts;
	      int Zustand;
	      double rz_Platte_max, rx_Platte_Landung, vx_Platte_Absprung, ry_Platte_Landung;
	      double vy_Platte_Absprung, rx_FussPlatte, ry_FussPlatte, rz_FussPlatte; 
	      double alpha1_ist, beta1_ist, omega_x1_ist, omega_y1_ist;
	      
	      //intern
	      double xf_end_intern, yf_end_intern, T_beta1, T_alpha1;
	      double posx_calc, posy_calc;
	      double vx_Platte_soll, vy_Platte_soll;
	      

	      //output
	      double F_h, xf_end_0, yf_end_0, F_beta1_x, F_beta1_z, F_alpha1_y, F_alpha1_z = 0;
	      
	      
	      
	      int point;
	      

	   

    
  };//end class CONTROLLER
  
  
}//end namspace einbein

#endif // EINBEIN_CONTROLLER_HPP