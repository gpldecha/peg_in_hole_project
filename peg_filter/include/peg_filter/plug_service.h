#ifndef PLUG_SERVICE_H_
#define PLUG_SERVICE_H_

#include <tf/LinearMath/Matrix3x3.h>
#include <armadillo>
#include <ros/ros.h>
#include <peg_filter/pf_manager.h>

namespace plugfilter {

class Plug_service{

    void tf2mat(const tf::Matrix3x3& m1, arma::mat& m2){

        m2(0,0)    = m1[0][0];
        m2(0,1)    = m1[0][1];
        m2(0,2)    = m1[0][2];

        m2(1,0)    = m1[1][0];
        m2(1,1)    = m1[1][1];
        m2(1,2)    = m1[1][2];

        m2(2,0)    = m1[2][0];
        m2(2,1)    = m1[2][1];
        m2(2,2)    = m1[2][2];
    }

    enum{RESET,BETA,NOISE,START};

public:

    Plug_service(ros::NodeHandle& node,Plug_pf_manager& plug_pf_manager);

private:

    bool callback(particle_filter::String_cmd::Request& req,particle_filter::String_cmd::Response& res);

private:

    ros::ServiceServer                      service;
    Plug_pf_manager&                        plug_pf_manager;
    std::map<std::string,int>               cmds;

};

}



#endif
