/**
 * @file polynome.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Polynomes object for trajectories.
 * 
 * @version 0.1
 * @date 2019-11-07
 * 
 * @copyright Copyright (c) 2019
 * 
 * See https://github.com/jrl-umi3218/jrl-walkgen/blob/master/src/Mathematics/PolynomeFoot.cpp
 * for further enhancement.
 */

#include <iostream>
#include <blmc_robots/mathematics/polynome.hpp>

namespace blmc_robots{

template <>
void TimePolynome<5>::set_parameters(
      double final_time,
      double init_pose,
      double init_speed,
      double final_pose)
{
  // save the parameter in the class and do some renaming
  final_time_ = final_time;
  init_pose_ = init_pose;
  init_speed_ = init_speed;
  init_acc_ = 0.0;
  final_pose_ = final_pose;
  final_speed_ = 0.0;
  final_acc_ = 0.0;
  // shortcuts
  double ft =  final_time_;
  double ip = init_pose_;
  double is = init_speed;
  double ia = final_acc_;
  double fp = final_pose_;
  double fs = final_speed_;
  double fa = final_acc_;
  // do the computation using the analytical solution
  double tmp;
  coefficients_[0] = ip;
  coefficients_[1] = is;
  coefficients_[2] = ia/2.0;
  
  tmp = ft*ft*ft;
  coefficients_[3] = (-3.0/2.0*ia*ft*ft-6.0*is*ft - 10.0*ip + 10.0*fp)/tmp;
  tmp=tmp*ft;
  coefficients_[4] = ( 3.0/2.0*ia*ft*ft + 8.0*is*ft + 15.0*ip - 15.0*fp)/tmp;
  tmp=tmp*ft;
  coefficients_[5] = ( -1.0/2.0*ia*ft*ft - 3.0*is*ft - 6.0*ip + 6.0*fp)/tmp;
}

}