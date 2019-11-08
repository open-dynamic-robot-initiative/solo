/**
 * @file test_polynomes.cpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Test for the polynome.hpp classes
 * @version 0.1
 * @date 2019-11-07
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <fstream>
#include <cmath>        // std::sqrt
#include <ctime>        // std::time
#include <cstdlib>      // std::rand, std::srand
#include <gtest/gtest.h>
#include "blmc_robots/mathematics/polynome.hpp"

#define RAND_MAX_NEW 1e4

/**< @brief The DISABLED_TestPolynomes class is used to disable test. */
class DISABLED_TestPolynomes : public ::testing::Test {};

/**<
 * @brief The TestPolynomes class: test suit template for setting up
 * the unit tests for the Polynomes.
 */
class TestPolynomes : public ::testing::Test {

public:
  TestPolynomes(): ::testing::Test(){
    // use current time as seed for random generator
    std::srand(std::time(nullptr));
  }

  static double rand()
  {
    double sign = 1.0;
    if(std::rand() % 2 == 0)
    {
      sign *= -1.0;
    }
    return sign * static_cast<double>(std::rand()) /
           static_cast<double>(RAND_MAX) *
           static_cast<double>(RAND_MAX_NEW);
  }

protected:
  /**< @brief SetUp, is executed before the unit tests */
  void SetUp() {}

  /**< @brief TearDown, is executed after teh unit tests */
  void TearDown() {}
};

/*! Test the compute function */
TEST_F(TestPolynomes, test_order_5_constructor)
{
  // create the polynome of order 5
  blmc_robots::TimePolynome<5> polynome;

  ASSERT_EQ(0.0, polynome.get_init_pose());
  ASSERT_EQ(0.0, polynome.get_init_speed());
  ASSERT_EQ(0.0, polynome.get_init_acc());
  ASSERT_EQ(0.0, polynome.get_final_pose());
  ASSERT_EQ(0.0, polynome.get_final_speed());
  ASSERT_EQ(0.0, polynome.get_final_acc());
  ASSERT_EQ(0.0, polynome.get_final_time());

}

/*! Test the compute function */
TEST_F(TestPolynomes, test_order_5)
{
  // create the polynome of order 5
  blmc_robots::TimePolynome<5> polynome;
  
  // define a random polynome
  double duration = std::abs(TestPolynomes::rand());
  double final_pose = TestPolynomes::rand();
  double init_pose = TestPolynomes::rand();
  double init_speed = 0.0;
  double eps = 1e-8;

  polynome.set_parameters(duration, init_pose, init_speed, final_pose);
  
  ASSERT_EQ(polynome.compute(-eps), init_pose);
  ASSERT_EQ(polynome.compute_derivative(-eps), init_speed);
  ASSERT_EQ(polynome.compute_sec_derivative(-eps), 0.0);
  ASSERT_EQ(polynome.compute(duration+eps), final_pose);
  ASSERT_EQ(polynome.compute_derivative(duration+eps), 0.0);
  ASSERT_EQ(polynome.compute_sec_derivative(duration+eps), 0.0);
  
  ASSERT_EQ(init_pose, polynome.get_init_pose());
  ASSERT_EQ(init_speed, polynome.get_init_speed());
  ASSERT_EQ(0.0, polynome.get_init_acc());
  ASSERT_EQ(final_pose, polynome.get_final_pose());
  ASSERT_EQ(0.0, polynome.get_final_speed());
  ASSERT_EQ(0.0, polynome.get_final_acc());
  ASSERT_EQ(duration, polynome.get_final_time());

  double ub = std::max(init_pose, final_pose);
  double lb = std::min(init_pose, final_pose);

  double init_time = -0.2 * duration;
  double final_time = duration + std::abs(init_time); 
  double time_step = std::abs(final_time - init_time)/10000.0; 

  double time = init_time;
  while(time < (final_time + 10 * time_step))
  {
    double value = polynome.compute(time);
    time += time_step;
    
    // the data is always between the init and final pose
    ASSERT_LE(value, ub);
    ASSERT_LE(lb, value);
  }

}