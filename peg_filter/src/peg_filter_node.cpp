
#include <ros/ros.h>

#include <optitrack_rviz/input.h>
#include <optitrack_rviz/listener.h>
#include <optitrack_rviz/debug.h>

#include <peg_filter/plug_service.h>

#include <visualise/vis_points.h>
#include <visualise/vis_point_cloud.h>

#include "peg_sensor/listener.h"
#include "peg_sensor/peg_world_wrapper/peg_world_wrapper.h"
#include "peg_sensor/peg_sensor_model/distance_model.h"

#include "peg_sensor_manager/sensor_manager.h"

#include <world_wrapper/visualisation/vis_wbox.h>
#include <node/publisher.h>

#include "peg_filter/belief_features/belief_features.h"
#include "peg_filter/belief_features/mode_feature.h"


#include <optitrack_rviz/listener.h>
#include <optitrack_rviz/type_conversion.h>
#include "netft_rdt_driver/String_cmd.h"
#include <std_msgs/Bool.h>

bool bBiasUpdated=false;

void bias_status_callback(const std_msgs::Bool::ConstPtr& msg)
{
    bBiasUpdated = msg->data;
}

void get_veclocity(arma::colvec3& u,const tf::Vector3& origin, const tf::Vector3& origin_tmp)
{
    u(0) = origin.getX() - origin_tmp.getX();
    u(1) = origin.getY() - origin_tmp.getY();
    u(2) = origin.getZ() - origin_tmp.getZ();
}

void init_delta_length(int init_pmf_type, pf::Point_mass_filter::delta &delta_,pf::Point_mass_filter::length& length_){

    std::cout<< "INIT DELTA LENGTH: " << init_pmf_type << std::endl;

    if(init_pmf_type == 0)
    {
        delta_.m  = 0.02;
        delta_.n  = 0.03;
        delta_.k  = 0.02;

        length_.m = 0.5;
        length_.n = 1.2;
        length_.k = 0.15;
    }
    else if(init_pmf_type == 1)
    {
        delta_.m  = 0.01;
        delta_.n  = 0.04;
        delta_.k  = 0.04;

        length_.m = 0.2;
        length_.n = 0.7;
        length_.k = 0.2;


    }else if(init_pmf_type == 2) // check edge all wall
    {
        delta_.m  = 0.02;
        delta_.n  = 0.02;
        delta_.k  = 0.1;

        length_.m = 0.5;
        length_.n = 1.5;
        length_.k = 0.6;

    }else if(init_pmf_type == 3) // for eddge (close socket)
    {
        delta_.m  = 0.005;
        delta_.n  = 0.005;
        delta_.k  = 0.005;

        length_.m = 0.03;
        length_.n = 0.15;
        length_.k = 0.15;
    }else if(init_pmf_type == 4) // for socket
    {
        delta_.m  = 0.002;
        delta_.n  = 0.002;
        delta_.k  = 0.002;

        length_.m = 0.04;
        length_.n = 0.2;
        length_.k = 0.2;
    }else if(init_pmf_type == 5){ // debug covariance matrix
        delta_.m  = 0.1;
        delta_.n  = 0.1;
        delta_.k  = 0.05;

        length_.m = 0.5;
        length_.n = 0.2;
        length_.k = 0.1;
    }else if(init_pmf_type == 6){ // check convolution

        delta_.m  = 0.02;
        delta_.n  = 0.025;
        delta_.k  = 0.025;

        length_.m = 0.02;
        length_.n = 0.5;
        length_.k = 0.5;

    }else if(init_pmf_type == 7){ // check convolution (2)
        delta_.m  = 0.02;
        delta_.n  = 0.025;
        delta_.k  = 0.025;
        length_.m = 0.02;
        length_.n = 0.5;
        length_.k = 0.5;
    }else if(init_pmf_type == 8){ // check bbox truncation
        delta_.m  = 0.025;
        delta_.n  = 0.025;
        delta_.k  = 0.025;
        length_.m = 0.5;
        length_.n = 0.5;
        length_.k = 0.5;
    }else if(init_pmf_type == 9){ // check for ability for insertion (very small belief)
        delta_.m  = 0.0025;
        delta_.n  = 0.0025;
        delta_.k  = 0.0025;
        length_.m = 0.01;
        length_.n = 0.03;
        length_.k = 0.03;
    }else if(init_pmf_type == 10){
        delta_.m  = 0.0025;
        delta_.n  = 0.01;
        delta_.k  = 0.01;
        length_.m = 0.01;
        length_.n = 0.1;
        length_.k = 0.1;
    }

}

bool is_above_table(arma::colvec3& x_sf){
    ROS_INFO_STREAM_THROTTLE(0.5,"x_sf:       " << x_sf(0) << " " << x_sf(1) << " " << x_sf(2));
    if((x_sf(0) > 0) && (std::fabs(x_sf(1)) <= 0.35) && (std::fabs(x_sf(2)) <= 0.15) ){
        return true;
    }else{
        return false;
    }
}

bool is_in_socket_area(arma::colvec3& x_bb, arma::colvec3& x_tt){
    if(x_bb(1) > -0.05 && x_tt(1) < 0.05 && x_bb(2) > -0.05 && x_tt(2) < 0.05){
        return true;
        ROS_INFO_STREAM_THROTTLE(1.0,"      IS IN SOCKET AREA");
    }else{
        return false;
    }

}

void initialise_prior_pdf(int init_pmf_type,pf::Point_mass_filter& pmf, double x, double y, double z){
    // int test_type = 0;
    arma::colvec3 pos_tmp = {{x,y,z}};
    arma::colvec3 pos_socket;
    std::cout<< "init_pmf_type: " << init_pmf_type << std::endl;
    {
        tf::StampedTransform transform;
        opti_rviz::Listener::get_tf_once("world","link_socket",transform);
        opti_rviz::type_conv::tf2vec(transform.getOrigin(),pos_socket);
    }

    if((init_pmf_type != 1))
    {

        if(init_pmf_type == 3)
        {
            pos_tmp = pos_socket;
            pos_tmp(0) = pos_tmp(0) + 0.02;

        }else{
            std::cout<< "6 || 7 || 8" << std::endl;
            pos_tmp.zeros();
        }

    }else{
        std::cout<< "1" << std::endl;
    }

    if((init_pmf_type == 9) || (init_pmf_type == 10))
    {
        pos_tmp(0) = x;
        pos_tmp(1) = y;
        pos_tmp(2) = z;
    }

    if(init_pmf_type == 0)
    {
        pos_tmp = pos_socket;
        pos_tmp(0) = pos_socket(0) + 0.3;
        pos_tmp(2) = pos_socket(2) + 0.05;
    }

    pf::Point_mass_filter::delta  delta_ ;
    pf::Point_mass_filter::length length_;

    init_delta_length(init_pmf_type,delta_,length_);

    pos_tmp.print("pos_tmp");
    pmf.reset(pos_tmp,delta_,length_);


    arma::cube& P = pmf.P;
    arma::mat& points = pmf.points;

    if(init_pmf_type == 6)
    {
        int i,j,k;
        for(std::size_t n = 0; n < P.n_elem;n++){
            pf::ind2sub(i,j,k,P.n_rows,P.n_cols,n);
            P(i,j,k) =  exp(-(1.0/0.01) * ( arma::sum(arma::pow(points.row(n).st() - pos_tmp,2))));
        }
    }else if(init_pmf_type == 7)
    {
        int i,j,k;
        int l = 0;
        for(std::size_t n = 0; n < P.n_elem;n++){
            pf::ind2sub(i,j,k,P.n_rows,P.n_cols,n);
            if(l % 2 == 0)
            {
                P(i,j,k) =  0;
            }else{
                P(i,j,k) =  1;
            }
            l++;
        }
    }else if(init_pmf_type == 8)
    {
        int i,j,k;
        int l = 0;
        for(std::size_t n = 0; n < P.n_elem;n++){
            pf::ind2sub(i,j,k,P.n_rows,P.n_cols,n);

            if(j >  P.n_cols/2)
            {
                P(i,j,k) =  0;

            }else{
                P(i,j,k) =  1;

            }
        }
    }
}


int main(int argc,char** argv){

    std::map<std::string,std::string> input;
    input["-sensor_topic"]      = "";
    input["-action_topic"]      = "";
    input["-urdf"]              = "";
    input["-rate"]              = "100";
    input["-fixed_frame"]       = "/world";
    input["-peg_link_name"]     = "";
    input["-path_sensor_model"] = "";


    if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    std::string sensor_topic        = input["-sensor_topic"];
    std::string action_topic        = input["-action_topic"];
    std::string fixed_frame         = input["-fixed_frame"];
    std::string peg_link_name       = input["-peg_link_name"];
    std::string srate               = input["-rate"];
    std::string path_sensor_model   = input["-path_sensor_model"];


    ros::init(argc, argv, "peg_filter");
    ros::NodeHandle nh;
    double Hz = boost::lexical_cast<float>(srate);
    ros::Rate rate(Hz);


    // initialise world (should be the same as in peg_sensor_classifier_node !!!)

    /// ------------------  Virtual Sensor for Particles -------------------------

    Peg_world_wrapper peg_world_wrapper(nh,"peg_world_wrapper",path_sensor_model,fixed_frame,peg_link_name);
    psm::Contact_distance_model contact_distance_model(*(peg_world_wrapper.peg_sensor_model.get()));

    psm::Sensor_manager sensor_manager(nh);
    sensor_manager.add("contact",&contact_distance_model);

    if(!sensor_manager.select_model("contact")){
        std::cout<< "FAILED to select_model" << std::endl;
        exit(0);
    }

    /// ------------------  Point Mass Filter -------------------------



    likeli::Binary_likelihood binary_likelihood;
    pf::Measurement_h    peg_measurement        =  std::bind(&psm::Sensor_manager::compute_hY,&sensor_manager,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    pf::likelihood_model peg_likelihood         =  std::bind(&likeli::Binary_likelihood::likelihood,&binary_likelihood,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);




    int init_pmf_type = 0;




    pf::Point_mass_filter::delta    delta_;
    pf::Point_mass_filter::length   length_;
    init_delta_length(init_pmf_type,delta_,length_);
    pf::Point_mass_filter pmf(peg_likelihood,peg_measurement,delta_,length_,3);

    std::cout<< "pmf constructed [peg_filter_node]" << std::endl;


    arma::colvec3 peg_position;
    {
        tf::StampedTransform transform;
        opti_rviz::Listener::get_tf_once(fixed_frame,peg_link_name,transform);
        opti_rviz::type_conv::tf2vec(transform.getOrigin(),peg_position);
    }
    arma::colvec3 table_position;
    {
        tf::StampedTransform transform;
        opti_rviz::Listener::get_tf_once(fixed_frame,"link_wall",transform);
        opti_rviz::type_conv::tf2vec(transform.getOrigin(),table_position);
    }
    arma::colvec3 socket_position;
    {
        tf::StampedTransform transform;
        opti_rviz::Listener::get_tf_once(fixed_frame,"link_socket",transform);
        opti_rviz::type_conv::tf2vec(transform.getOrigin(),socket_position);
    }



    initialise_prior_pdf(init_pmf_type,pmf,peg_position(0),peg_position(1),peg_position(2));

    std::cout<< "pmf prior initialised [peg_filter_node]" << std::endl;

    binary_likelihood.set_resolution(&pmf.get_delta());

    std::cout<< "binary_likelihood set [peg_filter_node]" << std::endl;


    plugfilter::Plug_pf_manager plug_pf_manager;
    plug_pf_manager.add(&pmf,"pmf");
    if(!plug_pf_manager.select_method("pmf")){
        ROS_ERROR("did not manage to select pmf [peg_filter_node]");
    }

    plug_pf_manager.init_visualise(nh);
    plug_pf_manager.set_pf_color_type(pf::C_WEIGHTS);
    plug_pf_manager.set_visualise_mode(opti_rviz::Vis_point_cloud::DEFAULT);



    /// ------------------- Filter Manager Service ---------------------

    plugfilter::Plug_service             plug_service(nh,plug_pf_manager);
    plugfilter::Plug_service::Initialise initialise_f = std::bind(initialise_prior_pdf,init_pmf_type,std::ref(pmf),std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    plug_service.set_initilisation_function(&initialise_f);

    std::cout<< "Plug_service set [peg_filter_node]" << std::endl;

    /// Listeners Velocity

    opti_rviz::Listener     peg_tip_position_listener(fixed_frame,action_topic);
    tf::Vector3             peg_origin_tf;
    tf::Vector3             peg_origin_tf_tmp;
    tf::Matrix3x3           peg_orient_tf;


    peg_tip_position_listener.update(peg_origin_tf,peg_orient_tf);
    peg_origin_tf_tmp = peg_origin_tf;

    /// Listener Sensor

    psm::Peg_sensor_listener plug_sensor_model(nh,sensor_topic);



    /// ------------------  Belief Compression ------------------


    Belief_compression     belief_compression;
    // advertised on topic [mode_feature]
    ModeBeliefCompress     mode_belief_compression(nh,"mode_entropy");

    belief_compression.add_bel_compress_method(&mode_belief_compression);
    if(belief_compression.set_method("mode_entropy"))
    {
        ROS_INFO("belief compression set");
    }else{
        ROS_ERROR("belief compression not set");
        exit(0);
    }


    /// ------------ Get Fist time Position and Orientation ------------

    arma::colvec&    Y = plug_sensor_model.Y;

    tf::Matrix3x3 Rot_peg;
    Rot_peg.setRPY(0,0,M_PI);

    arma::colvec3    u;
    arma::colvec3    peg_origin;
    arma::mat33      peg_orient;
    peg_orient.zeros();
    peg_origin.zeros();
    bool bcorrect_orient = false;
    peg_origin_tf_tmp = peg_origin_tf;
    while(nh.ok() && (bcorrect_orient == false))
    {

        peg_tip_position_listener.update(peg_origin_tf,peg_orient_tf);
        opti_rviz::type_conv::tf2mat(peg_orient_tf,peg_orient);
        opti_rviz::type_conv::tf2vec(peg_origin_tf,peg_origin);
        bcorrect_orient = arma::any(arma::vectorise(peg_orient));
        ros::spinOnce();
        rate.sleep();
    }
    peg_origin_tf_tmp = peg_origin_tf;
    get_veclocity(u,peg_origin_tf,peg_origin_tf_tmp);
    u.print("u");
    peg_orient.print("peg_orient");

    /// ------------ Remove Bias from FT sensor --------------

    ros::ServiceClient client = nh.serviceClient<netft_rdt_driver::String_cmd>("/ft_sensor/bias_cmd");
    ros::Subscriber sub_ft_bias_status = nh.subscribe("/ft_sensor/bias_status", 10, bias_status_callback);
    netft_rdt_driver::String_cmd srv;
    srv.request.cmd  = "bias";
    srv.response.res = "";
    if (client.call(srv))
    {
        ROS_INFO_STREAM("net_ft res: " << srv.response.res);
    }
    else
    {
        ROS_ERROR("Failed to call netft bias service");
    }

    while(nh.ok() && !bBiasUpdated)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("bias reset");

    /// ------------ Wait a litte bit ------------
    {
        while(nh.ok() && Y(0) != 0){
            std::cout << "waiting all zero Y: " << Y(0) << " " << Y(1) << " " << Y(2) << std::endl;
            ros::spinOnce();
            rate.sleep();
        }
    }

    tf::Quaternion qtmp;
    qtmp.setX(0);
    qtmp.setY(-0.5);
    qtmp.setZ(0);
    qtmp.setW(0.5);

    arma::colvec3 wall_position;

    {
        tf::StampedTransform transform;
        opti_rviz::Listener::get_tf_once("world","link_wall",transform);
        opti_rviz::type_conv::tf2vec(transform.getOrigin(),wall_position);
    }

    double time_taken;
    int count = 0;
    int number = 10;
    arma::colvec3 m_u;
    arma::colvec3 plan_norm;
    plan_norm(0) = 1; plan_norm(1) = 0; plan_norm(2) = 0;
    arma::colvec3 target,mode_SF, mode, tt, bb;

    target    = table_position;
    target(0) = target(0) + 0.026;

    while(nh.ok()){

        // const auto start_time = std::chrono::steady_clock::now();

        peg_world_wrapper.update();

        peg_tip_position_listener.update(peg_origin_tf,peg_orient_tf);
        get_veclocity(u,peg_origin_tf,peg_origin_tf_tmp);

        peg_orient_tf.setRotation(qtmp);

        opti_rviz::type_conv::tf2mat(peg_orient_tf,peg_orient);
        opti_rviz::type_conv::tf2vec(peg_origin_tf,peg_origin);

        pf::tf_debuf(peg_origin_tf,peg_orient_tf,"PEG_POS");


        bcorrect_orient = arma::any(arma::vectorise(peg_orient));

        mode   = mode_belief_compression.get_mode();
        bb     = pmf.bbox.bb - socket_position;
        tt     = pmf.bbox.tt - socket_position;

        mode_SF =  mode - socket_position;

        opti_rviz::debug::tf_debuf(mode_SF,"MODE_SF");
        opti_rviz::debug::tf_debuf(mode,"MODE");


        opti_rviz::debug::tf_debuf(target,"TARGET");

        if(!(Y.is_empty()) && bcorrect_orient){
            ROS_INFO_STREAM_THROTTLE(0.5,"mode(0): " << mode_SF(0));
            opti_rviz::debug::tf_debuf(bb,"BB");
            opti_rviz::debug::tf_debuf(tt,"TT");
            ROS_INFO_STREAM_THROTTLE(1.0,"is_above_table(bb):   " << is_above_table(bb));
            ROS_INFO_STREAM_THROTTLE(1.0,"is_above_table(tt):   " << is_above_table(tt));



            if(Y(0) == 1 && is_above_table(bb) && is_above_table(tt) && !is_in_socket_area(bb,tt)){
                ROS_INFO_STREAM_THROTTLE(0.2,"----------------> Is above table");

                if(mode_SF.is_finite()){
                    u(0) = 0.01 * (0.02 - mode_SF(0));
                }else{
                    ROS_INFO_STREAM_THROTTLE(1.0, "MODE_SF IS NOT FINITE");
                }

            }else if(Y(0) == 1)
            {
                u(0) = 0;
            }

            ROS_INFO_STREAM_THROTTLE(1.0,"u:            " << u(0) << " " << u(1) << " " << u(2));

            plug_pf_manager.update(Y,u,peg_orient,Hz);

            /// belief feature (belief compression) computation
            belief_compression.update(u,pmf.points,pmf.P);
        }


        /// particle filter visualisation

        plug_pf_manager.visualise();

        /*time_taken =  time_taken + std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - start_time).count() / 1000000.0;
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
