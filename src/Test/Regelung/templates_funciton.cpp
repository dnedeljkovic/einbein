#include <einbein/templates_function.hpp>
#include <eeros/math/Matrix.hpp>


//template norm --> norm(vec_1-vec_2)
double norm(eeros::math::Matrix<3,1> vec){
  double norm_skal = sqrt(vec(0)*vec(0) + vec(1)*vec(1) + vec(2)*vec(2));
  return norm_skal;
}





