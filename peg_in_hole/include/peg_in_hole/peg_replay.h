#ifndef PEG_REPLAY_H_
#define PEG_REPLAY_H_


#include <ros/ros.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <peg_filter/pf_manager.h>

#include <plug_sensor_manager/sensor_manager.h>
#include <plug_sensor_models/distance_features.h>

#include <optitrack_rviz/type_conversion.h>
#include <optitrack_rviz/broadcaster.h>
#include <optitrack_rviz/listener.h>
#include <optitrack_rviz/type_conversion.h>

#include <peg_in_hole/run_trajectories.h>
#include <peg_in_hole/index_subscriber.h>
#include <peg_in_hole/String_cmd.h>

#include <memory>
#include <string>
#include <vector>
#include <armadillo>


class Peg_replay{

    typedef enum {LOAD,INIT,START,STOP,LIST_INDEX} commands;

    typedef std::shared_ptr<plugfilter::Plug_pf_manager> sptr_pf_manger;
    typedef std::shared_ptr<psm::Sensor_manager>         sptr_sensor_manager;

public:

    Peg_replay(      ros::NodeHandle& node,
               const std::string&   plug_topic,
               const std::string& socket_topic);

    void set_traj_dir_path(const std::string path2dir);

    void update();

    void initalise_vision(ros::NodeHandle& node);

    void visualise();

    void use_contact_model(wobj::WrapObject& wrapped_objects);


private:


    bool cmd_callback(peg_in_hole::String_cmd::Request& req,peg_in_hole::String_cmd::Response& res);

    inline void print(std::string name,tf::Vector3& vec){
        std::cout<< name + " :" << vec.x() << "\t" << vec.y() << "\t" << vec.z() << std::endl;
    }

public:

    Run_trajectories                run_trajectories;
    std::size_t                     t;
    std::size_t                     t_stop;

private:

    opti_rviz::Broadcaster          broadcaster_plug_link;
    tf::Vector3                     plug_link_pos;
    tf::Matrix3x3                   plug_link_rot;

    std::unique_ptr<psm::Plug_contact_model> ptr_plug_contact_model;


    opti_rviz::Broadcaster          broadcaster_socket;
    tf::Vector3                     position_socket;
    tf::Matrix3x3                   orientation_socket;

    Index_subscriber                index_subscriber;
    bool                            bListen_index;

    ros::ServiceServer              service;
    std::map<std::string,commands>  cmds;

    ros::NodeHandle*                ptr_node;
    bool                            bPlayTraj;
    bool                            bInit;


    std::string                     path2dir;

};


#endif
