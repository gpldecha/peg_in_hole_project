#ifndef PEG_FIND_SOCKET_H_
#define PEG_FIND_SOCKET_H_

#include <armadillo>
#include "peg_hole_policy/policies/base_find.h"


namespace ph_policy{

class Find_socket : public Base_find{

public:

    Find_socket(ros::NodeHandle& nh,const std::string& path_sensor_model,const std::string& fixed_frame);

    virtual void get_linear_velocity(tf::Vector3& velocity,const tf::Vector3& peg_origin);

    void get_linear_velocity(tf::Vector3 &velocity, const tf::Vector3 &peg_origin,tf::Vector3& des_origin,tf::Quaternion& des_orient);


private:

    wobj::WBox*    ptr_socket_wbox;
    geo::fCVec3    surf_proj;

};

}

#endif
