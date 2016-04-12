#ifndef FORCE_CONTROL_H_
#define FORCE_CONTROL_H_

#include <armadillo>
#include <tf/LinearMath/Matrix3x3.h>
#include <visualise/vis_vector.h>
#include <visualise/vis_cylinder.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>

namespace ph_policy{

class Force_control{

public:

    Force_control(ros::NodeHandle& nh);

    void update(const arma::colvec3 &force, const tf::Vector3& ee_position, const tf::Quaternion& ee_orientation);

    void get_over_edge(arma::colvec3& velocity);

    void regulise_force(arma::colvec3& velocity, double max_force=8);

    void force_safety(arma::colvec3& velocity, double max_force=6);

    void update_x(arma::colvec3& velocity);


private:

   double quadratic(double x);

   double sign(double x){
       if (x > 0){
           return 1;
       }else{
           return -1;
       }
   }

   double scale(double x, double max){
       if(x > max){
           return 1;
       }else if(x < 0){
           return 0;
       }else{
           return x/max;
       }
   }

private:

    tf::Quaternion q_tmp;
    tf::Matrix3x3 Rot,rot,tmp;
    tf::Matrix3x3 Rx,Ry,Rz;
    tf::Vector3   vel_tmp;
    double        fN, grad;
    double        max_N;
    arma::colvec3 F,F_n,F_display;

    tf::Transform trans;
    tf::Transform trans2;
    tf::TransformBroadcaster broadcaster;

    opti_rviz::Vis_cylinder             vis_cylinder;
    opti_rviz::Vis_vectors              vis_vector;
    std::vector<opti_rviz::Arrow>       force_vector;
    std::vector<opti_rviz::Cylinder>    force_cylinders;
    bool bFirst;

};

}



#endif
