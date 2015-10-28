#ifndef PLUG_SENSOR_H_
#define PLUG_SENSOR_H_

/**

**/


#include "wrapobject.h"
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <tf/transform_listener.h>
#include <optitrack_rviz/listener.h>
#include <visualise/vis_vector.h>

#include <armadillo>
class plug_config{
  public:
  plug_config(){
        config_file = "/home/guillaume/roscode/catkin_ws/src/wrapper/models_project/objects/meshes/plug/config/X.txt";
  }
  plug_config(const std::string& config_file):config_file(config_file){}
  std::string config_file;
};

class Contact_points{
public:
    std::size_t     index;
    float           distance;
    arma::fcolvec3  closest_point;
};

class Plug_sensor_vis{

public:

    enum{SURFACE=0,EDGE=1};

public:

    Plug_sensor_vis(const plug_config& config_file,
                wobj::WrapObject &wrap_object);

    void update();

    const std::vector<tf::Vector3>& get_model();

    const std::vector<tf::Vector3>& get_closet_point();

    const std::vector<opti_rviz::Arrow>& get_arrows();

private:

    void get_distance_features();

    void update(const tf::Vector3& T, const tf::Matrix3x3& R);


private:


    wobj::WrapObject&               wrapped_world;
    std::vector<tf::Vector3>        model,model_TF;
    std::vector<Contact_points>     contact_info;
    opti_rviz::Listener             tf_listener;
    tf::Vector3                     position;
    tf::Matrix3x3                   orientation;
    arma::fcolvec3                  tmp_vec3f;
    tf::Vector3                     tmp_Vec3;

    std::vector<opti_rviz::Arrow>   arrows;
    std::vector<tf::Vector3>        closest_points;

    float                           min_distance_edge;
    float                           min_distance_surface;
    float                           current_distance_surface;
    float                           current_distance_edge;

};

#endif
