#ifndef PDF_MODE_FEATURE_H_
#define PDF_MODE_FEATURE_H_

#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>

#include <armadillo>
#include "visualise/vis_points.h"
#include "visualise/vis_gmm.h"

#include "base_bel_compress.h"
#include "particle_filter/particle_filter_definitions.h"

#include <random>

class ModeBeliefCompress : public BaseBeliefCompression{

    enum  {
        hX = 0,
        hY = 1,
        hZ = 2,
        H = 3
    } fv;

public:

    ModeBeliefCompress(ros::NodeHandle& nh,const std::string& name);

    virtual void update(const arma::colvec3& velocity, const arma::mat& points, const arma::cube& P);

    void visualize();

    arma::colvec3 get_mode();

private:

    void arg_max_x(arma::colvec3& pos, double& w, const arma::colvec3& pos_tmp,const arma::mat& points, const arma::cube &P);

    void reset(const std_msgs::BoolConstPtr& msg);

private:

    arma::mat*                  points_ptr;
    arma::cube*                 P_ptr;
    arma::mat33                 Sigma;  // sample covariance

    std_msgs::Float64MultiArray feature_msg;
    ros::Publisher              pub;

    double                      Entropy;
    double                      max_w, argmax_w;
    double                      max_tmp;
    double                      delta_max_threashold;
    arma::colvec3               mode_point,mode_point_tmp;
    arma::rowvec3               mean;
    arma::uword                 index,index_tmp;
    bool                        bFirst;
    bool                        bReset;

    opti_rviz::Vis_points       vis_points;
    arma::fmat                  vis_pts;
    opti_rviz::Vis_gmm          vis_gmm;

    bool bFristModeFeature;

    ros::Subscriber             sub_;


    std::discrete_distribution<int> multinomial;
    std::default_random_engine generator;
};



#endif
