#include <ros/ros.h>
#include <peg_sensor/peg_world_wrapper/peg_world_wrapper.h>


int main(int argc, char** argv){

    std::map<std::string,std::string> input;
    input["-urdf"]              = "";
    input["-rate"]              = "100";
    input["-fixed_frame"]       = "world_frame";
    input["-path_sensor_model"] = "";


    if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);


    std::string fixed_frame           = input["-fixed_frame"] ;
    std::string path_sensor_model     = input["-path_sensor_model"];
    std::string urdf_path             = input["-urdf"];

    ros::init(argc, argv, "peg_in_hole");
    ros::NodeHandle nh;

    Peg_world_wrapper peg_world_wrapper(nh,"peg_in_hole",path_sensor_model,fixed_frame);

    ros::Rate rate(boost::lexical_cast<float>(input["-rate"]));
    while(nh.ok()){

        peg_world_wrapper.update();

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
