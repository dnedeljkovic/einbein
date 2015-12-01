#ifndef EINBEIN_TEMPLATES_HPP
#define EINBEIN_TEMPLATES_HPP

#include <eeros/math/Matrix.hpp>



//templates Sginum
template<class T>
int signum(T t){
  if (t < T(0))
    return T(-1);
  return t > T(0);
}


//template norm --> norm(vec_1-vec_2)
inline double norm(eeros::math::Matrix<3,1> vec){
  double norm_skal = sqrt(vec(0)*vec(0) + vec(1)*vec(1) + vec(2)*vec(2));
  return norm_skal;
}


#endif // EINBEIN_TEMPLATES_HPP