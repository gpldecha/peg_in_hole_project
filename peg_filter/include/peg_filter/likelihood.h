#ifndef PEG_FILTER_LIKELIHOOD_H_
#define PEG_FILTER_LIKELIHOOD_H_

#include "particle_filter/particle_filter_definitions.h"
#include "point_mass_filter/point_mass_filter.h"
#include "peg_sensor/peg_sensor_model/distance_model.h"
#include "peg_sensor/peg_sensor_model/peg_distance_model.h"

/**
 *      Likelihood functions
 *
 *      Base and derived implementations of the pegs sensor likelihood model.
 *
 *      These functions take multivariate sensor input $Y \in R^{n}$ and compare it
 *      with a measurement model $\hat{Y} = h(X)$ where X is the cartesian position of
 *      the peg.
 *
 *      The likelihood of a measurement is then: N(Y - h(X)|0,Sigma), where N is a multivariate
 *      Gaussian function with mean zero and unit covariance.
 */

namespace likeli{

class Gaussian_likelihood{

public:

    Gaussian_likelihood(const arma::colvec& variance);

    /**
     * @brief gaussian_likelihood
     * @param L     : output likelihood
     * @param Y     : (K x 1),  input sensor values
     * @param hY    : (num_points x K), expected sensor values
     */
    void gaussian_likelihood(double* L, const arma::colvec& Y, const arma::mat& hY);

private:

    arma::colvec one_div_var;

};

class Binary_likelihood{

public:

    Binary_likelihood();

    void set_resolution(const pf::Point_mass_filter::delta* delta_);


    /**
     * @brief likelihood
     * @param L     : output likelihood
     * @param Y     : (K x 1) input sensor value
     * @param hY    : (num_points x K), expected sensor values
     */
    void likelihood(double* L, const arma::colvec& Y, const arma::mat &hY);

private:

    const pf::Point_mass_filter::delta* ptr_delta_;

    double range;

private:

    bool set_sense(double Y);

    inline double binary_surf_edge(const double d_surf, const double d_edge, bool bSense_SURF, bool bSense_EDGE){

        // both Surface and Edge are Sensed
        if(bSense_SURF && bSense_EDGE)
        {
            if(d_surf <= 0.03 && d_edge < 0.01)
            {
                return 1;
            }else{
                return 0;
            }
        }else if(bSense_SURF && !bSense_EDGE)
        {
            ROS_INFO_THROTTLE(1.0," SURFACE AND NO EDGE");
            if(d_surf <= 0.03 && d_edge >= 0.01)
            {
                return 1;
            }else{
                return 0;
            }
        }else if(!bSense_SURF && bSense_EDGE)
        {
            if(/*d_surf <= 0.03 ||*/ d_edge < 0.01)
            {
                return 1;
            }else{
                return 0;
            }

        }else if(!bSense_SURF && !bSense_EDGE)
        {
            if(d_surf <= 0.005 || d_edge < 0.005)
            {
                return 0;
            }else{
                return 1;
            }

        }else{
            return 1;
        }

    }


    inline double gaussian(const double dist_surface,const double var,bool bSense_SURF){
        if(bSense_SURF){
            return std::exp(-0.5 * (1.0/var) * dist_surface);
        }else{
            return 1;
        }
    }

    double binary_surf_likelihood(const double dist_surface,const double range,bool bSense_SURF);

    double binary_edge_likelihood(const double dist_edge, const double range,bool bSense_EDGE);

    inline double binary_socket_likelihood(const double dist_edge, bool bSense){
        return 1;
    }


};

}





#endif
