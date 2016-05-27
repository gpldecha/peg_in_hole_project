#ifndef PEG_FILTER_LIKELIHOOD_H_
#define PEG_FILTER_LIKELIHOOD_H_

#include "particle_filter/particle_filter_definitions.h"
#include "point_mass_filter/point_mass_filter.h"
#include "peg_sensor/peg_sensor_model/distance_model.h"
#include "peg_sensor/peg_sensor_model/peg_distance_model.h"
#include "peg_sensor/peg_sensor_model/peg_sensor_model.h"
#include <visualise/vis_vector.h>


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


inline double gaussian_likelihood(const double h_dist_edge, const double dist_edge){

    return exp(-100 * (h_dist_edge - dist_edge) * (h_dist_edge - dist_edge));

}


inline double binary_surf_likelihood(const double dist_surface,const double range,bool bSense_SURF){
    if(bSense_SURF)
    {
        if(dist_surface <= 0.015)
        {
            return 1;
        }else{
            return 0;
        }
    }
}

/**
 * @brief binary_edge_likelihood
 * @param h_dist_edge           : estimated distance to an edge.
 * @param dist_edge             : sensor measurement
 * @param range                 : resolution
 * @return
 */
inline double binary_edge_likelihood(const double h_dist_edge, const double dist_edge, const double range){
    if(dist_edge >=  0.02) {
        // no sense

        if(h_dist_edge <= 0.009){ // should sense, but there is no sensing
            return 0;
        }else{
            return 1;
        }
    }else{
        //sense
        ROS_INFO_STREAM_THROTTLE(0.1,"SENSING EDGE");
        if(h_dist_edge <= 0.01)
        {
            return 1;
        }else{
            return 0;
        }

      //  return gaussian_likelihood(h_dist_edge,dist_edge);
       /* if(h_dist_edge < 0.04)
        {
            return 1;
        }else{
            return 0;
        }*/
    }
}


inline double binary_likelihood(int hY_socket, int Y_socket){
    if(hY_socket == Y_socket)
    {
        return 1;
    }else{
        return 0;
    }
}

inline double binary_likelihood(int hY1, int Y1, int hY2, int Y2 )
{

    if(hY1 == Y1)
    {
        if(hY2 == hY1)
        {
            return 1;
        }else{
            return 1;
        }


    }else if(hY2 == Y2)
    {

    }else{
        return 0;
    }

}


inline double sgmf(const double x,const double a,const double c){
    return 1.0/(1.0 + std::exp(-a * (x - c)));
}



inline bool set_sense(double Y){
    if((int)Y == 0){
        return false;
    }else{
        return true;
    }
}

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

class Mixed_likelihood{

    enum class Y_TYPE{FT,VIRTUAL};

public:

    Mixed_likelihood(ros::NodeHandle &nh, wobj::WrapObject& wrapped_world,const arma::colvec3& peg_origin);

    void set_resolution(const pf::Point_mass_filter::delta* delta_);

    /**
     * @brief likelihood
     * @param L     : output likelihood
     * @param Y     : (K x 1) input sensor value
     * @param X     : hypothetical position
     */
    void likelihood(double* L, const arma::colvec& Y, const arma::mat &X, const arma::mat33 &Rot);


    /**
     * @brief       : likelihood_stwo
     * @param L     : output likelihood
     * @param Y     : (K x 1) input sensor value
     * @param X     : hypothetical position
     */
    void likelihood_stwo(double* L, const arma::colvec& Y, const arma::mat &X, const arma::mat33 &Rot);


    void likelihood_sthree(double* L, const arma::colvec& Y, const arma::mat &X, const arma::mat33 &Rot);


private:

    const pf::Point_mass_filter::delta* ptr_delta_;

    double      sum_L;
    bool        bSense_SURF;
    bool        bSense_edge;
    double      lik_one;

    double range;
    arma::colvec3 hY_edge, Y_edge;

   wobj::WrapObject&               wrapped_world;
   wobj::WBox&                     wall_box,socket_box;
   wobj::WBox&                     box_h1,box_h2,box_h3;
   wobj::WSocket&                  wsocket;

   wobj::WBox&                     socket_top_edge;
   wobj::WBox&                     socket_bottom_edge;
   wobj::WBox&                     socket_left_edge;
   wobj::WBox&                     socket_right_edge;

   arma::fmat33                     rot, rot_tmp;
   tf::Matrix3x3                    Rot_tmp;

   tf::Vector3                      arrow_origin, arrow_direction;

   geo::fCVec3                      F;
   geo::fCVec3                      F_n;
   geo::fCVec3                      point_i;
   geo::fCVec3                      plat_dir;
   geo::fCVec3                      center_dir;
   geo::fCVec3                      plat_dir_n;
   double                           plat_dir_radius;
   double                           force_norm;
   double                           dist_center;
   double                           dist_socket_c;
   double                           force_yz;
   double                           average_value;

   bool                             b_l_c,b_r_c,b_t_c,b_b_c;

   Y_TYPE                           Y_type;

   bool                             b_insert;
   bool                             b_ring;

   bool in_circle;
   bool in_ls;
   bool in_rs;
   bool in_ts;
   bool in_bs;
   bool in_s;
   bool close_peg;

   const arma::colvec3&                 peg_origin;
   opti_rviz::Vis_vectors               vis_vectors;
   std::vector<opti_rviz::Arrow>        arrows;


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


    double binary_edge_likelihood(const double dist_edge, const double range,bool bSense_EDGE);


};

}





#endif
