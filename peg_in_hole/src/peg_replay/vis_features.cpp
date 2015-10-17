
#include <ros/ros.h>
#include <tf/LinearMath/Vector3.h>

#include <objects/socket_one.h>
#include <objects/vis_socket.h>
#include <world_wrapper/world_wrapper.h>
#include <world_wrapper/visualisation/vis_wbox.h>

#include <optitrack_rviz/input.h>
#include <visualise/vis_points.h>
#include <visualise/vis_vector.h>

#include <node/publisher.h>

#include <plug_sensor/plug_sensor.h>

int main(int argc, char** argv){

    std::map<std::string,std::string> input;
    input["-urdf"] = "";
    input["-rate"] = "1000";


    if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }

    opti_rviz::Input::print_input_options(input);

    ros::init(argc, argv, "peg_in_hole");
    ros::NodeHandle node;

    ww::World_wrapper world_wrapper;
    world_wrapper.loadURDF(input["-urdf"]);
    world_wrapper.initialise_origin_orientation(world_wrapper,"world");

    geo::fCVec3 T = {{0,0,-0.02}};
    for(std::size_t i = 0; i < world_wrapper.wrapped_objects.wboxes.size();i++){
        world_wrapper.wrapped_objects.wboxes[i].transform(T);
    }


    geo::fCVec3 origin_      = {{0,0,-0.02/2}};
    geo::fCVec3 dim_         = {{0.8,0.4,0.02}};
    geo::fCVec3 orientation_ = {{M_PI/2,0,M_PI/2}};

    wobj::WBox wsocket_wall("socket_wall",dim_,origin_,orientation_);

    /// add a socket
    tf::Vector3 origin(0,0,0);
    tf::Vector3 rpy(M_PI/2,0,M_PI/2);

    obj::Socket_one socket_one("socket_one",origin,rpy,1);

    world_wrapper.wrapped_objects.push_back_box(wsocket_wall);
    world_wrapper.wrapped_objects.push_back_box(socket_one.wbox);

    world_wrapper.wrapped_objects.push_back_socket(socket_one.wsocket);



    /// Visualise socket

    obj::Vis_socket vis_socket(node,world_wrapper.wrapped_objects.wsocket);
    vis_socket.initialise(25,0.001);

    /// Visualise sensor model;

     plug_config config_file;
     Plug_sensor_vis plug_sensor(config_file,world_wrapper.wrapped_objects);

     opti_rviz::Vis_points vis_points(node,"plug_model");
     vis_points.scale = 0.005;
     vis_points.initialise("world",plug_sensor.get_model());

     opti_rviz::Vis_vectors vis_vectors(node,"closest_features");
     vis_vectors.scale = 0.005;
     std::vector<tf::Vector3> colors(2);
     colors[0] = tf::Vector3(1,0,0);
     colors[1] = tf::Vector3(0,0,1);
     vis_vectors.set_color(colors);
     vis_vectors.initialise("world",plug_sensor.get_arrows());

     /// feature publisher

     ww::Publisher publisher("visualization_marker",&node,&world_wrapper);
     publisher.init("/world");
     publisher.update_position();


     ros::Rate rate(boost::lexical_cast<float>(input["-rate"]));
     while(node.ok()){

         plug_sensor.update();
         vis_points.update(plug_sensor.get_model());
         vis_vectors.update(plug_sensor.get_arrows());

         vis_socket.publish();
         vis_points.publish();
         vis_vectors.publish();
         publisher.publish();


       ros::spinOnce();
       rate.sleep();
    }


    return 0;
}
