#ifndef PLUG_SERVICE_H_
#define PLUG_SERVICE_H_

#include <tf/LinearMath/Matrix3x3.h>
#include <armadillo>
#include <ros/ros.h>
#include <peg_filter/pf_manager.h>
#include <optitrack_rviz/listener.h>
#include <functional>

namespace plugfilter {



class Plug_service{

public:

    enum{RESET,BETA,NOISE,START};

    typedef std::function<void(double x, double y, double z)> Initialise;

public:

    Plug_service(ros::NodeHandle& node,Plug_pf_manager& plug_pf_manager);

    void set_initilisation_function(const Initialise* initialise_f);

private:

    bool callback(particle_filter::String_cmd::Request& req,particle_filter::String_cmd::Response& res);

private:

    ros::ServiceServer                      service;
    Plug_pf_manager&                        plug_pf_manager;
    const Initialise*                       initialise_f;
    std::map<std::string,int>               cmds;

};

}



#endif
