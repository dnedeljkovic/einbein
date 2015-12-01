#ifndef EINBEIN_CONTROLSYSTEM_HPP
#define EINBEIN_CONTROLSYSTEM_HPP

#include <einbein/Regelung/Trajektorie/Trajektorie.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>

namespace einbein {

class ControlSystem{
  public:
  ControlSystem(double ts);
  void start();
  void stop();
  void join();
  eeros::control::Constant<double> Ts;
  eeros::control::Constant<double> x_start;
  eeros::control::Constant<double> x_end;
  eeros::control::Constant<double> T_sprung;
  
  Trajektorie trajektorie;
  
  private:
  eeros::control::TimeDomain timedomain;
};

}

#endif // EINBEIN_CONTROLSYSTEM_HPP
