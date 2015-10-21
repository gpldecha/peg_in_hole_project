
#include <ros/ros.h>

#include <optitrack_rviz/input.h>
#include <optitrack_rviz/listener.h>


#include <peg_world_wrapper/plug_world_wrapper.h>
#include <peg_filter/plug_service.h>

#include <visualise/vis_points.h>
#include <visualise/vis_point_cloud.h>
#include <plug_sensor_models/listener.h>

#include <world_wrapper/visualisation/vis_wbox.h>
#include <node/publisher.h>



void get_veclocity(arma::colvec3& u,const tf::Vector3& origin, const tf::Vector3& origin_tmp)
{
    u(0) = origin.getX() - origin_tmp.getX();
    u(1) = origin.getY() - origin_tmp.getY();
    u(2) = origin.getZ() - origin_tmp.getZ();
}

int main(int argc,char** argv){

    std::map<std::string,std::string> input;
    input["-sensor_topic"]     = "";
    input["-action_topic"]     = "";
    input["-urdf"]             = "";
    input["-rate"]             = "100";
    input["-fixed_frame"]      = "world_frame";

   if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    ros::init(argc, argv, "peg_filter");
    ros::NodeHandle node;
    ros::Rate rate(boost::lexical_cast<float>(input["-rate"]));

    // initialise world

    Plug_world_wrapper plug_world_wrapper(input["-urdf"]);

    /// Particle filter initialisation
    tf::Matrix3x3 Rot_plug;
    Rot_plug.setRPY(0,0,M_PI);
    arma::mat33 rot;

    plugfilter::PF_parameters pf_parameters(plug_world_wrapper.world_wrapper.wrapped_objects);

    ROS_INFO("create particle filter");
    plugfilter::Plug_pf_manager plug_pf_manager(pf_parameters);
                                plug_pf_manager.init_visualise(node);
    plugfilter::Plug_service    plug_service(node,plug_pf_manager);


    /// Listeners Velocity

    opti_rviz::Listener     listener(input["-fixed_frame"],input["-action_topic"]);
    tf::Vector3             origin_plug,origin_plug_tmp;
    tf::Matrix3x3           orientation_plug;
                            listener.update(origin_plug,orientation_plug);
                            origin_plug_tmp = origin_plug;
    /// Listener Sensor

    psm::Plug_sensor_listener plug_sensor_model(node,input["-sensor_topic"],2);


    /// feature publisher

    ww::Publisher publisher("visualization_marker",&node,&plug_world_wrapper.world_wrapper);
    publisher.init("world_frame");
    publisher.update_position();


    ros::spinOnce();


    arma::colvec2 Y;
    arma::colvec3 u;

    double time_taken;
    std::size_t number = 100;
    std::size_t count  = 0;

    u.zeros();
    //u(0) = -0.0001;


    double x_pos = 0.4;



   // tf2mat(Rot_plug,rot);


    while(node.ok()){

        const auto start_time = std::chrono::steady_clock::now();

        u.zeros();
        u(0) = -0.001;
        listener.update(origin_plug,orientation_plug);
        get_veclocity(u,origin_plug,origin_plug_tmp);
        origin_plug_tmp = origin_plug;


        //Y(0) = 1;//plug_sensor_model.data[0];
        /*x_pos = x_pos + u(0);
        if(x_pos < 0.01){
            u(0) = 0;
        }*/


        // plug_pf_manager.update(Y,u,rot);
         plug_pf_manager.visualise();
         publisher.publish();



        time_taken =  time_taken + std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count() / 1000000.0;
        count++;
        if(count == number){
            std::cout<< "time_taken: " << time_taken/(double)number << std::endl;
            count = 0;
            time_taken = 0;
        }


        ros::spinOnce();
        rate.sleep();
   }



    return 0;
}
