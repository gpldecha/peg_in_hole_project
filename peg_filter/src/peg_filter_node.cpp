
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

#include <peg_features/peg_features.h>

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

    std::string sensor_topic = input["-sensor_topic"];
    std::string action_topic = input["-action_topic"];
    std::string urdf_path    = input["-urdf"];
    std::string fixed_frame  = input["-fixed_frame"];
    std::string srate        = input["-rate"];



    ros::init(argc, argv, "peg_filter");
    ros::NodeHandle node;
    ros::Rate rate(boost::lexical_cast<float>(srate));

    // initialise world

    Plug_world_wrapper plug_world_wrapper(urdf_path);

        std::cout<< "peg_filter #0" << std::endl;

    plugfilter::PF_parameters   pf_parameters(plug_world_wrapper.world_wrapper.wrapped_objects);

    plugfilter::Plug_pf_manager plug_pf_manager(pf_parameters);
                                plug_pf_manager.init_visualise(node);
    plugfilter::Plug_service    plug_service(node,plug_pf_manager);

    std::cout<< "peg_filter #1" << std::endl;

    /// Listeners Velocity

    opti_rviz::Listener     listener(fixed_frame,action_topic);
    tf::Vector3             origin_plug,origin_plug_tmp;
    tf::Matrix3x3           orientation_plug;
                            listener.update(origin_plug,orientation_plug);
                            origin_plug_tmp = origin_plug;

    std::cout<< "peg_filter #2" << std::endl;

    /// Listener Sensor

    psm::Plug_sensor_listener plug_sensor_model(node,sensor_topic,2);

        std::cout<< "peg_filter #3" << std::endl;

    /// feature publisher

    ww::Publisher publisher("visualization_marker",&node,&plug_world_wrapper.world_wrapper);
    publisher.init(fixed_frame);
    publisher.update_position();

    /// PDF feature methods

    const arma::mat& points     = plug_pf_manager.particle_filter->particles;
    const arma::colvec& weights = plug_pf_manager.particle_filter->weights;


    Peg_filter_features peg_filter_features(node,points,weights);
    ros::spinOnce();




   /* std::cout<< "=== particle filter node === " << std::endl;
    std::cout<< "   checking sensor " << std::endl;
    while(node.ok()){

        plug_sensor_model.data


        ros::spinOnce();
        rate.sleep();
    }*/


    tf::Matrix3x3 Rot_plug;
    Rot_plug.setRPY(0,0,M_PI);
    arma::mat33 rot;

    arma::colvec2 Y;
    arma::colvec3 u;


    while(node.ok()){

     //   const auto start_time = std::chrono::steady_clock::now();

        listener.update(origin_plug,orientation_plug);
        get_veclocity(u,origin_plug,origin_plug_tmp);
        origin_plug_tmp = origin_plug;

        tf2mat(orientation_plug,rot);

        //Y(0) = 1;//plug_sensor_model.data[0];
        Y.print("Y");


        plug_pf_manager.update(Y,u,rot);

        /// feature computation
        peg_filter_features.update();



        /// particle filter visualisation

        plug_pf_manager.visualise();
        publisher.publish();



      /*  time_taken =  time_taken + std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count() / 1000000.0;
        count++;
        if(count == number){
         //   std::cout<< "time_taken: " << time_taken/(double)number << std::endl;
            count = 0;
            time_taken = 0;
        }*/

        ros::spinOnce();
        rate.sleep();
   }



    return 0;
}
