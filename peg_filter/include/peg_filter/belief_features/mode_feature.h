#ifndef PDF_MODE_FEATURE_H_
#define PDF_MODE_FEATURE_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <armadillo>
#include "visualise/vis_points.h"

class Mode_feature{

    enum  {
        X = 0,
        Y = 1,
        Z = 2,
        W = 3
    } fv;

public:

    Mode_feature(ros::NodeHandle& nh, const arma::mat& points,const arma::colvec& weights);

    void update();

    void visualize();

private:

    void get_mode_feature();

private:

    const arma::mat&            points;
    const arma::colvec&         weights;

    std_msgs::Float64MultiArray feature_msg;
    ros::Publisher              pub;

    double                      max_w;
    double                      max_tmp;
    double                      delta_max_threashold;
    arma::colvec3               mode_point;
    arma::uword                 index,index_tmp;
    bool                        bFirst;

    opti_rviz::Vis_points       vis_points;
    arma::fmat                  vis_pts;

};



#endif
