/**
 * \file main.cpp
 * \brief gtest main
 * \author Maximilien Naveau
 * \date 2018
 *
 * Main file that runs all unittest using gtest
 * @see https://git-amd.tuebingen.mpg.de/amd-clmc/ci_example/wikis/catkin:-how-to-implement-unit-tests
 */

#include <gtest/gtest.h>

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
