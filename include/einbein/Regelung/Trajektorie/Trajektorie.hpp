#ifndef EINBEIN_TRAJEKTORIE_HPP
#define EINBEIN_TRAJEKTORIE_HPP

#include <eeros/control/Block.hpp>
#include <eeros/control/Input.hpp>
#include <eeros/control/Output.hpp>
#include <eeros/control/TimeDomain.hpp>

#include <einbein/templates_function.hpp>



namespace einbein{
  
  class Trajektorie : public eeros::control::Block{
    
    public:
	    Trajektorie();
	    virtual ~Trajektorie();
	    
	    //define inputs
	    virtual eeros::control::Input<>& getIn_Ts(){
	      return in_Ts;
	    }
	    
	    virtual eeros::control::Input<>& getIn_T_sprung(){
	      return in_T_sprung;
	    }
	    
	    virtual eeros::control::Input<>& getIn_xf_start(){
	      return in_xf_start;
	    }
	    
	    virtual eeros::control::Input<>& getIn_xf_end(){
	      return in_xf_end;
	    }
	    
	    
	    //define outputs
	    virtual eeros::control::Output<>& getOut_ddx_d(){
	      return out_ddx_d;
	    }
	    
	    virtual eeros::control::Output<>& getOut_dx_d(){
	      return out_dx_d;
	    }
	    
	    virtual eeros::control::Output<>& getOut_x_d(){
	      return out_x_d;
	    }
	    
	    virtual void run();
	    
 
    protected: 
	    //define inputs
	    eeros::control::Input<> in_Ts;
	    
	    eeros::control::Input<> in_T_sprung;
	    eeros::control::Input<> in_xf_start;
	    eeros::control::Input<> in_xf_end;
	    
	    //define outputs
	    eeros::control::Output<> out_ddx_d;
	    eeros::control::Output<> out_dx_d;
	    eeros::control::Output<> out_x_d;
	    
      
      
    private:
	    double Ts, T_sprung, xf_start, xf_end, xf_end_1, delta_xf;
	    double ddx_d, dx_d, x_d;
	    double ddx_d_1, dx_d_1, x_d_1, delta_xf_1;
	    double t_Luft, t1, t2, t3, t4;
	    int punkte, zustand_intern, punkte_pos, punkte_neg, punkte_null;
	    double delta_x, t_b, dd_x_calc, vorzeichen_ddx;
  };//end class Trajektorie
  
}//end namespace einein

#endif // EINBEIN_TRAJEKTORIE_HPP