#ifndef EINBEIN_CSVORKIN_HPP
#define EINBEIN_CSVORKIN_HPP

#include <einbein/Regelung/Fusspunkt/VorKin.hpp>
#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <einbein/Regelung/Fusspunkt/constantFusspunkt.hpp>

namespace einbein {

class CSVorKin{
  public:
  VorKin vorKin;
  CSVorKin(double ts);
  void start();
  void stop();
  void join();
  eeros::control::Constant<double> alpha1;
  eeros::control::Constant<double> beta1;
  eeros::control::Constant<double> gamma1;
  eeros::control::Constant<double> enc1;
  eeros::control::Constant<double> enc2;
  eeros::control::Constant<double> enc3;
  eeros::control::Constant<Vector3> FVec_Fuss;
  
  double out_Pf_IMU;
  
  private:
  eeros::control::TimeDomain timedomain;
};

}

#endif // EINBEIN_CSVORKIN_HPP