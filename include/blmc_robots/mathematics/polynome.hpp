/**
 * @file polynome.hpp
 * @author Maximilien Naveau (maximilien.naveau@gmail.com)
 * @brief Polynomes object for trajectories.
 * @version 0.1
 * @date 2019-11-06
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#pragma once

#include <array>

namespace blmc_robots
{

/**
 * @brief Simple class that defines \f$ P(x) \f$ a polynome of order ORDER.
 * It provide simple methods to compute \f$ P(x) \f$,
 * \f$ \frac{dP}{dx}(x) \f$, and \f$ \frac{dP^2}{dx^2}(x) \f$
 * 
 * @tparam ORDER is the order of the polynome
 */
template<int ORDER>
class  Polynome
{ 
    /*! Type of the container for the poynome coefficients */
    typedef std::array<double, ORDER> Coefficients;
public:

    /*! Constructor */
    Polynome();

    /*! Destructor */
    ~Polynome();

    /*! Compute the value. */
    double compute(double x);

    /*! Compute the value of the derivative. */
    double compute_derivative(double x);
    
    /*! Compute the value of the second derivative. */
    double compute_sec_derivative(double x);

    /*! Get the coefficients. */
    void get_coefficients(Coefficients &coefficients) const;

    /*! Set the coefficients. */
    void set_coefficients(const Coefficients &coefficients);

    inline int degree()
    { return ORDER; };
    
    /*! Print the coefficient. */
    void print() const;

  protected:

    /*! Vector of coefficients. */
    std::array<double, ORDER+1> coefficients_;
};

/**
 * @brief Simple class that defines \f$ P(t) \f$ a polynome of order ORDER.
 * With \f$ t \f$ being the time in any units.
 * It provide simple methods to compute safely \f$ P(time) \f$,
 * \f$ \frac{dP}{dt}(t) \f$, and \f$ \frac{dP^2}{dt^2}(t) \f$
 * 
 * @tparam ORDER 
 */
template<int ORDER>
class TimePolynome: public Polynome<ORDER>
{ 
public:

    /*! Constructor */
    TimePolynome()
    {
      final_time_ = 0.0;
      init_pose_ = 0.0;
      init_speed_ = 0.0;
      init_acc_ = 0.0;
      final_pose_ = 0.0;
      final_speed_ = 0.0;
      final_acc_ = 0.0;
    };

    /*! Destructor */
    ~TimePolynome(){};

    /*! Compute the value. */
    double compute(double t);

    /*! Compute the value of the derivative. */
    double compute_derivative(double t);
    
    /*! Compute the value of the second derivative. */
    double compute_sec_derivative(double t);
    
    double get_final_time() const
    {return final_time_;}
    double get_init_pose() const
    {return init_pose_;}
    double get_init_speed() const
    {return init_speed_;}
    double get_init_acc() const
    {return init_acc_;}
    double get_final_pose() const
    {return final_pose_;}
    double get_final_speed() const
    {return final_speed_;}
    double get_final_acc() const
    {return final_acc_;}

    /**
     * @brief Computes a polynome trajectory according to the following
     * constraints:
     * \f{eqnarray*}{
     * P(0) &=& init_pose \\ 
     * P(0) &=& init_speed = 0 \\
     * P(0) &=& init_acc = 0 \\ 
     * P(final_time_) &=& final_pose \\ 
     * P(final_time_) &=& final_speed = 0 \\
     * P(final_time_) &=& final_acc = 0
     * \f}
     * 
     * @param final_time is used in the constraints.
     * @param init_pose is used in the constraints.
     * @param init_speed is used in the constraints.
     * @param final_pose is used in the constraints.
     */
    void set_parameters(
      double final_time,
      double init_pose,
      double init_speed,
      double final_pose);

  protected:
    double final_time_; /**< store the inputs for later access */
    double init_pose_; /**< store the inputs for later access */
    double init_speed_; /**< store the inputs for later access */
    double init_acc_; /**< store the inputs for later access */
    double final_pose_; /**< store the inputs for later access */
    double final_speed_; /**< store the inputs for later access */
    double final_acc_; /**< store the inputs for later access */
};

}// namespace blmc_robots

#include "blmc_robots/mathematics/polynome.hxx"
