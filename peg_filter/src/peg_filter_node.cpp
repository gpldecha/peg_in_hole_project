
#include <ros/ros.h>

#include <optitrack_rviz/input.h>
#include <optitrack_rviz/listener.h>

#include <peg_filter/plug_service.h>

#include <visualise/vis_points.h>
#include <visualise/vis_point_cloud.h>

#include "peg_sensor/listener.h"
#include "peg_sensor/peg_sensor_model/distance_model.h"

#include <world_wrapper/visualisation/vis_wbox.h>
#include <node/publisher.h>

#include "peg_filter/belief_features/belief_features.h"
#include <optitrack_rviz/listener.h>


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
    input["-path_sensor_model"] = "";


   if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    std::string sensor_topic        = input["-sensor_topic"];
    std::string action_topic        = input["-action_topic"];
    std::string urdf_path           = input["-urdf"];
    std::string fixed_frame         = input["-fixed_frame"];
    std::string srate               = input["-rate"];
    std::string path_sensor_model   = input["-path_sensor_model"];



    ros::init(argc, argv, "peg_filter");
    ros::NodeHandle nh;
    ros::Rate rate(boost::lexical_cast<float>(srate));

    // initialise world

    /*Peg_world_wrapper peg_world_wrapper(nh,path_sensor_model,fixed_frame);
    wobj::WrapObject&  w_objects = peg_world_wrapper.get_wrapped_objects();
    ww::World_wrapper& w_wrapper = peg_world_wrapper.get_world_wrapper();


    plugfilter::PF_parameters   pf_parameters(w_objects);
                                pf_parameters.likelihood_t = likeli::SIMPLE_CONTACT;


    plugfilter::Plug_pf_manager plug_pf_manager(pf_parameters);
                                plug_pf_manager.init_visualise(nh);
    plugfilter::Plug_service    plug_service(nh,plug_pf_manager);



    /// Listeners Velocity

    opti_rviz::Listener     peg_tip_position_listener(fixed_frame,action_topic);
    tf::Vector3             peg_origin;
    tf::Vector3             peg_origin_tmp;
    tf::Matrix3x3           peg_orient;

                            peg_tip_position_listener.update(peg_origin,peg_orient);
                            peg_origin_tmp = peg_origin;


    /// Listener Sensor

    psm::Peg_sensor_listener plug_sensor_model(nh,sensor_topic,2);


    /// Wold model publisher
    ww::Publisher world_model_publisher("visualization_marker",&nh,&w_wrapper);
    world_model_publisher.init(fixed_frame);
    world_model_publisher.update_position();

    /// PDF feature methods

    const arma::mat& points     = plug_pf_manager.particle_filter->particles;
    const arma::colvec& weights = plug_pf_manager.particle_filter->weights;


    Belief_features     belief_features(nh,points,weights);

    //opti_rviz::Listener peg_tip_position_listener("world_frame","peg_link");

    // psm::Contact_distance_model contact_distance_model(plug_world_wrapper.world_wrapper.wrapped_objects);


   /* std::cout<< "=== particle filter node === " << std::endl;
    std::cout<< "   checking sensor " << std::endl;
    while(node.ok()){

        plug_sensor_model.data


        ros::spinOnce();
        rate.sleep();
    }*/


    tf::Matrix3x3 Rot_peg;
    Rot_peg.setRPY(0,0,M_PI);

    arma::mat33     rot;
    arma::colvec2   Y;
    arma::colvec3   u;
    arma::colvec3   peg_pos;

    while(nh.ok()){

     //   const auto start_time = std::chrono::steady_clock::now();

   /*     peg_tip_position_listener.update(peg_origin,peg_orient);

        get_veclocity(u,peg_origin,peg_origin_tmp);
        tf2mat(peg_orient,rot);*/

/*
        peg_pos(0) = origin_plug.x();
        peg_pos(1) = origin_plug.y();
        peg_pos(2) = origin_plug.z();
       */

       // contact_distance_model.update(Y,peg_pos,rot);

       // Y.print("Y");

        //plug_pf_manager.update(Y,u,rot);

        /// belief feature (belief compression) computation
       // belief_features.update();



        /// particle filter visualisation

       // plug_pf_manager.visualise();
        //publisher.publish();

      /*  time_taken =  time_taken + std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count() / 1000000.0;
        count++;
        if(count == number){
         //   std::cout<< "time_taken: " << time_taken/(double)number << std::endl;
            count = 0;
            time_taken = 0;
        }*/

       //peg_origin_tmp = peg_origin;
        ros::spinOnce();
        rate.sleep();
   }


    return 0;
}
