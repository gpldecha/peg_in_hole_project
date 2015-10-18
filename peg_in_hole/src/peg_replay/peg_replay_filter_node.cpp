#include <ros/ros.h>

#include <visualise/vis_points.h>
#include <visualise/vis_point_cloud.h>
#include <visualise/vis_vector.h>

#include <optitrack_rviz/input.h>
#include <optitrack_rviz/print.h>

#include <peg_world_wrapper/plug_world_wrapper.h>

#include <world_wrapper/visualisation/vis_wbox.h>

#include <peg_filter/plug_service.h>

#include <peg_in_hole/peg_replay.h>

#include <objects/vis_socket.h>

#include <node/publisher.h>

#include <armadillo>

#include <boost/lexical_cast.hpp>


int main(int argc, char** argv){


    std::map<std::string,std::string> input;
    input["-traj_path"]        = "/home/guillaume/MatlabWorkSpace/peg_in_hole/TextData/Albert/";
    input["-rate"]             = "10";
    input["-broadcast_plug"]   = "plug_link";
    input["-broadcast_socket"] = "link_socket";

   if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    ros::init(argc, argv, "peg_in_hole_replay_filter");
    ros::NodeHandle node;
    //ros::Rate rate(boost::lexical_cast<float>(input["-rate"]));    // initialise world
    ros::Rate rate(100);

    /// Wrap the world

    ww::World_wrapper       world_wrapper;
    geo::fCVec3 origin_      = {{0,0,-0.02/2 - 0.03/2}};
    geo::fCVec3 dim_         = {{0.8,0.4,0.05}};
    geo::fCVec3 orientation_ = {{M_PI/2,0,M_PI/2}};

    wobj::WBox wsocket_wall("socket_wall",dim_,origin_,orientation_);

    tf::Vector3 origin(0,0,0);
    tf::Vector3 rpy(M_PI/2,0,M_PI/2);

    obj::Socket_one socket_one("socket_one",origin,rpy,1);

    world_wrapper.wrapped_objects.push_back_box(wsocket_wall);
    world_wrapper.wrapped_objects.push_back_socket(socket_one.wsocket);
    world_wrapper.wrapped_objects.push_back_box(socket_one.wbox);

    /// Filter and Sensor parameters

    Sensor_parameters   sensor_parameters(world_wrapper.wrapped_objects);   
    plugfilter::PF_parameters pf_parameters(world_wrapper.wrapped_objects);


   pf_parameters.particle_filter_type  =  plugfilter::SIR;
   pf_parameters.number_particles      =  1500;

   pf_parameters.likelihood_t          = likeli::FOUR_CONTACT;
   sensor_parameters.t_sensor          =    psm::FOUR_CONTACT_DIST;
   sensor_parameters.b_print_sensor    = true;


   pf_parameters.visualisation_mode    = opti_rviz::Vis_point_cloud::DEFAULT;
   pf_parameters.pf_color_type         = pf::C_WEIGHTS;

   pf_parameters.sampling_parameters.set_diag(0.0005,0.01,0.01);


    /// REPLAY

    Peg_replay peg_replay(node,input["-broadcast_plug"],input["-broadcast_socket"]);
               peg_replay.set_traj_dir_path(input["-traj_path"]);
               peg_replay.run_trajectories.setup_pf(pf_parameters);
               peg_replay.run_trajectories.setup_sensor(sensor_parameters);
               peg_replay.initalise_vision(node);
               peg_replay.use_contact_model(world_wrapper.wrapped_objects);
               peg_replay.run_trajectories.add_reguliser();


    /// Visualise features

    ww::Publisher publisher("visualization_marker",&node,&world_wrapper);
    publisher.init("/world");
    publisher.update_position();

    while(node.ok()){


        peg_replay.update();

        peg_replay.visualise();
        publisher.publish();

        ros::spinOnce();
        rate.sleep();

    }

    return 0;
}
