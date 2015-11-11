
#include <ros/ros.h>

#include <optitrack_rviz/input.h>
#include <optitrack_rviz/listener.h>

#include <peg_filter/plug_service.h>

#include <visualise/vis_points.h>
#include <visualise/vis_point_cloud.h>

#include "peg_sensor/listener.h"
#include "peg_sensor/peg_sensor_model/distance_model.h"
#include "peg_sensor/peg_world_wrapper/peg_world_wrapper.h"

#include "peg_sensor_manager/sensor_manager.h"

#include <world_wrapper/visualisation/vis_wbox.h>
#include <node/publisher.h>

#include "peg_filter/belief_features/belief_features.h"
#include <optitrack_rviz/listener.h>
#include <optitrack_rviz/type_conversion.h>


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
    input["-fixed_frame"]       = "/world_frame";
    input["-path_sensor_model"] = "";


   if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    std::string sensor_topic        = input["-sensor_topic"];
    std::string action_topic        = input["-action_topic"];
    std::string fixed_frame         = input["-fixed_frame"];
    std::string srate               = input["-rate"];
    std::string path_sensor_model   = input["-path_sensor_model"];



    ros::init(argc, argv, "peg_filter");
    ros::NodeHandle nh;
    ros::Rate rate(boost::lexical_cast<float>(srate));

    // initialise world (should be the same as in peg_sensor_classifier_node !!!)


    Peg_world_wrapper peg_world_wrapper(nh,"peg_filter",path_sensor_model,fixed_frame);
    wobj::WrapObject& wrapped_objects   = peg_world_wrapper.get_wrapped_objects();
    wobj::WrapObject& w_objects         = peg_world_wrapper.get_wrapped_objects();

    psm::Sensor_manager sensor_manager(nh,wrapped_objects,peg_world_wrapper.socket_one);
                        sensor_manager.t_sensor = psm::SIMPLE_CONTACT_DIST;

    // 1) measurment function hY = h(X)
    // 2) likelihood function N(Y - hY;0,var)



    arma::colvec variance;
    variance.resize(3);
    variance(0) = (0.005);
    variance(1) = (0.005);
    variance(2) = (0.0001);
    variance    = arma::square(variance);
    likeli::Gaussian_likelihood gaussian_likelihood(variance);


    pf::Measurement_h    peg_measurement   =  std::bind(&psm::Sensor_manager::update_particles,&sensor_manager,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    pf::likelihood_model peg_likelihood    =  std::bind(&likeli::Gaussian_likelihood::gaussian_likelihood,&gaussian_likelihood,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);


    plugfilter::PF_parameters   pf_parameters(peg_measurement,peg_likelihood);
                                pf_parameters.Y_dim = 3;
                                pf_parameters.number_particles = 1500;
                                pf_parameters.pf_color_type    = pf::C_WEIGHTS;
                                pf_parameters.sampling_parameters.set_diag(0.005,0.005,0.005);

    plugfilter::Plug_pf_manager plug_pf_manager(pf_parameters);
                                plug_pf_manager.init_visualise(nh);
    plugfilter::Plug_service    plug_service(nh,plug_pf_manager);



    /// Listeners Velocity

    opti_rviz::Listener     peg_tip_position_listener(fixed_frame,action_topic);
    tf::Vector3             peg_origin_tf;
    tf::Vector3             peg_origin_tf_tmp;
    tf::Matrix3x3           peg_orient_tf;




                            peg_tip_position_listener.update(peg_origin_tf,peg_orient_tf);
                            peg_origin_tf_tmp = peg_origin_tf;


    /// Listener Sensor

    psm::Peg_sensor_listener plug_sensor_model(nh,sensor_topic);


    /// PDF feature methods

    const arma::mat&    points  = plug_pf_manager.particle_filter->particles;
    const arma::colvec& weights = plug_pf_manager.particle_filter->weights;

    Belief_features     belief_features(nh,points,weights);

    tf::Matrix3x3 Rot_peg;
    Rot_peg.setRPY(0,0,M_PI);

    arma::colvec3    u;
    arma::colvec3   peg_origin;
    arma::mat33     peg_orient;
    arma::colvec&   Y = plug_sensor_model.Y;

    bool non_zero_orient,no_Y;
    peg_origin_tf_tmp = peg_origin_tf;


    while(nh.ok()){

     //   const auto start_time = std::chrono::steady_clock::now();

        peg_world_wrapper.update();

        peg_tip_position_listener.update(peg_origin_tf,peg_orient_tf);
        get_veclocity(u,peg_origin_tf,peg_origin_tf_tmp);
        opti_rviz::type_conv::tf2mat(peg_orient_tf,peg_orient);
        opti_rviz::type_conv::tf2vec(peg_origin_tf,peg_origin);


        non_zero_orient = !arma::any(arma::vectorise(peg_orient));
        no_Y = Y.is_empty();

        if(non_zero_orient){
            peg_orient.print("peg_orient is zero");
        }

        if(no_Y){
            std::cout<< "Y is empty" << std::endl;
        }

        if(!no_Y && !non_zero_orient){
           plug_pf_manager.update(Y,u,peg_orient);
        }

        /// belief feature (belief compression) computation

         belief_features.update();


       /// particle filter visualisation

         plug_pf_manager.visualise();
       //  publisher.publish();*/

      /*  time_taken =  time_taken + std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count() / 1000000.0;
          count++;
          if(count == number){
            std::cout<< "time_taken: " << time_taken/(double)number << std::endl;
            count = 0;
            time_taken = 0;
           }
        */
       peg_origin_tf_tmp = peg_origin_tf;
       ros::spinOnce();
       rate.sleep();
   }

    return 0;
}
