#ifndef EINBEIN_CSPDV_HPP
#define EINBEIN_CSPDV_HPP


#include <eeros/control/TimeDomain.hpp>
#include <eeros/control/Constant.hpp>
#include <einbein/Regelung/PDV/PDV.hpp>


using namespace eeros;
using namespace einbein;

namespace einbein {

class CSPDV{
  public:
  PDV pDV;
  CSPDV(double ts);
  void start();
  void stop();
  void join();

  eeros::control::Constant<double>  xIst_0; 
  eeros::control::Constant<double> xSoll_0;

    
  private:
  eeros::control::TimeDomain timedomain;
};

}

#endif // EINBEIN_CSVORKIN_HPP