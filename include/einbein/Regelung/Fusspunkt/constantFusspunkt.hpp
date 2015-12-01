#ifndef EINBEIN_CONSTANTFUSSPUNKT_HPP
#define EINBEIN_CONSTANTFUSSPUNKT_HPP


//includes
#include <eeros/math/Matrix.hpp>

using namespace eeros::math;

namespace constFusspunkt{
  
     constexpr double pi  	= 3.14159265359; 	// [m]

    // geometrische Parameter Roboter (CAD)
    constexpr double n 		= 0.013; 	// [m]
    constexpr double m 		= 0.04275; 	// [m]
    constexpr double enc_zusatz = 0.025; 	// [m]
    constexpr double c 		= 0.0750257; 	// [m]
    constexpr double a 		=  0.12820;	// [m]
    constexpr double l_Unterschenkel = 0.4;	// [m]
        
    
    const eeros::math::Matrix<3,1,double> rP1i_1_Mi 	= eeros::math::Matrix<3,1,double> ({0.112,
										    0.0,
										    -0.0276}); 	//Abstand vom KS {1} zum KS {P1}, dargestellt im KS {1} [m]
    
    const eeros::math::Matrix<3,1,double> rP5i_1_Mi 	= eeros::math::Matrix<3,1,double> ({0.2255,
										    0.0,
										    0.032});	//Abstand vom KS {1} zum Punkt P5, dargestellt im KS {1} [m]
    
    const eeros::math::Matrix<3,1,double> r1_IMU_Mi 	= eeros::math::Matrix<3,1,double> ({0.0,
										    0.0,
										    -0.0876553});//Abstand vom KS {IMU} zum KS {P1}, dargestellt im KS {Mi} [m] (da bei allen Mi gleich--> Rotation um z-Achse)
    
    const eeros::math::Matrix<3,1,double> rP3i_P1i_P1i = eeros::math::Matrix<3,1,double> ({0.129955,
										    0.0,
										    0.00342046});//Abstand vom KS {P1} zum Punkt P3, dargestellt im KS {P1} [m]
    
    const eeros::math::Matrix<3,1,double> P1i_Mi       = r1_IMU_Mi + rP1i_1_Mi; 			 //Von {IMU} zu Punkt 1, dargestellt im KS{1} [m]
    const eeros::math::Matrix<3,1,double> P5i_Mi       = r1_IMU_Mi + rP5i_1_Mi;             		 //Von {IMU} zu Punkt 5 dargestellt im KS{1} [m]
    const eeros::math::Matrix<3,1,double> rP5i_P1i_Mi  = P5i_Mi- P1i_Mi;                		 //Abstand vom KS {P1} zum Punkt P5, dargestellt im KS {1} [m]
  
    const eeros::math::Matrix<3,1,double> c_vec  = eeros::math::Matrix<3,1,double> ({c,
										    0.0,
										    0.0});
    
    const eeros::math::Matrix<3,1,double> n_vec  = eeros::math::Matrix<3,1,double> ({n,
										    0.0,
										    0.0});
  
    const eeros::math::Matrix<3,1,double> rf41_IMU  = eeros::math::Matrix<3,1,double> ({-0.0190,
											0.0,
											-0.0325});

    const eeros::math::Matrix<3,1,double> rf42_IMU  = eeros::math::Matrix<3,1,double> ({0.0095,
											-0.016454482671904,
											-0.0325});    

    const eeros::math::Matrix<3,1,double> rf43_IMU  = eeros::math::Matrix<3,1,double> ({0.0095,
											0.016454482671904,
											-0.0325});       
}
#endif // CONSTANTSFUSSPUNKT_HPP



