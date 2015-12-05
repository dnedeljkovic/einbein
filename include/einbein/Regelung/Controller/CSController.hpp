#ifndef EINBEIN_CSCONTROLLER_HPP
#define EINBEIN_CSCONTROLLER_HPP


#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <einbein/Regelung/Controller/Controller.hpp>

namespace einbein {

class CSController{
  public:
  Controller controller;
  CSController(double ts);
  void start();
  void stop();
  void join();
  eeros::math::Matrix<11,1> var;
  eeros::math::Matrix<14,1> Imu;
  eeros::control::Constant<int> Zustand; 
  eeros::control::Constant<double> Ts;
  eeros::control::Constant<Matrix<11,1>> VarZustand;
  eeros::control::Constant<Matrix<14,1>> DataImu;
  

    
  private:
  eeros::control::TimeDomain timedomain;
};

}

#endif // EINBEIN_CSVORKIN_HPP