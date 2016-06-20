#include <ros/ros.h>

#include <optitrack_rviz/input.h>
#include <optitrack_rviz/listener.h>
#include <optitrack_rviz/debug.h>
#include <optitrack_rviz/publisher.h>

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

#include "peg_sensor/peg_sensor_model/distance_model.h"

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

bool is_above_table(arma::colvec3& x_sf){
  //  ROS_INFO_STREAM_THROTTLE(0.5,"x_sf:       " << x_sf(0) << " " << x_sf(1) << " " << x_sf(2));
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
        delta_.n  = 0.004;
        delta_.k  = 0.004;

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
    }else if(init_pmf_type == 8){ // To gather data close to the goal
        delta_.m  = 0.003;
        delta_.n  = 0.003;
        delta_.k  = 0.003;
        length_.m = 0.01;
        length_.n = 0.03;
        length_.k = 0.1;
    }else if(init_pmf_type == 9){ // check for ability for insertion (very small belief)
        delta_.m  = 0.0025;
        delta_.n  = 0.0025;
        delta_.k  = 0.0025;
        length_.m = 0.01;
        length_.n = 0.03;
        length_.k = 0.03;
    }else if(init_pmf_type == 10){ //  check likelihood
        delta_.m  = 0.005;
        delta_.n  = 0.005;
        delta_.k  = 0.005;

        length_.m = 0.035;
        length_.n = 0.15;
        length_.k = 0.15;
    }
}

void init_delta_length_experiment(int experiment_type, pf::Point_mass_filter::delta &delta_,pf::Point_mass_filter::length& length_){

    if(experiment_type == 1){       // to the right of the socket, larger uncertainty in Z
        delta_.m  = 0.0025;
        delta_.n  = 0.0025;
        delta_.k  = 0.0025;
        length_.m = 0.01;
        length_.n = 0.03;
        length_.k = 0.15;
    }else if(experiment_type == 2){
        delta_.m  = 0.0025;
        delta_.n  = 0.0025;
        delta_.k  = 0.0025;
        length_.m = 0.01;
        length_.n = 0.15;
        length_.k = 0.03;
    }else{
        delta_.m  = 0.003;
        delta_.n  = 0.003;
        delta_.k  = 0.003;
        length_.m = 0.01;
        length_.n = 0.03;
        length_.k = 0.1;
    }
}

void initialise_prior_pdf(int init_pmf_pos,int init_pmf_type,pf::Point_mass_filter& pmf, double x, double y, double z){
    // int test_type = 0;
    arma::colvec3 pos_peg = {{x,y,z}};
    arma::colvec3 pos_socket, pos_tmp;
    std::cout<< "init_pmf_type: " << init_pmf_type << std::endl;
    {
        tf::StampedTransform transform;
        opti_rviz::Listener::get_tf_once("world","link_socket",transform);
        opti_rviz::type_conv::tf2vec(transform.getOrigin(),pos_socket);
    }

    pf::Point_mass_filter::delta  delta_ ;
    pf::Point_mass_filter::length length_;

    init_delta_length(init_pmf_type,delta_,length_);


    pos_tmp     = pos_socket;
    pos_tmp(0)  = pos_tmp(0) + 0.01;

    //pos_tmp.print("pos_tmp");
    pmf.reset(pos_tmp,delta_,length_);
}

void initialise_prior_experiment(int expermient_type,pf::Point_mass_filter& pmf, double x, double y, double z){

    arma::colvec3 peg_position = {{x,y,z}};
    arma::colvec3 pos_socket;
    arma::colvec3 pos_pmf;

    {
        tf::StampedTransform transform;
        opti_rviz::Listener::get_tf_once("world","link_socket",transform);
        opti_rviz::type_conv::tf2vec(transform.getOrigin(),pos_socket);
    }

    pf::Point_mass_filter::delta  delta_ ;
    pf::Point_mass_filter::length length_;
    init_delta_length_experiment(expermient_type,delta_,length_);

    pmf.reset(peg_position,delta_,length_);

    if(expermient_type == 1){
        pos_pmf     = pos_socket;
        pos_pmf(1)  = pos_pmf(1) + 0.085;
        pos_pmf(0)  = peg_position(0);
    }else if(expermient_type == 2){
        pos_pmf     = pos_socket;
        pos_pmf(2)  = pos_pmf(2) + 0.085;
        pos_pmf(0)  = peg_position(0);
    }

    pos_pmf(0)  = peg_position(0);
    pos_pmf(1)  = peg_position(1);
    //pos_pmf(2)  = peg_position(2);

    pmf.reset(pos_pmf,delta_,length_);
}



int main(int argc,char** argv){

    std::map<std::string,std::string> input;
    input["-ft_sensor_topic"]   = "";
    input["-virtual_sensor_topic"]   = "";
    input["-action_topic"]      = "";
    input["-urdf"]              = "";
    input["-rate"]              = "100";
    input["-fixed_frame"]       = "/world";
    input["-peg_link_name"]     = "";
    input["-path_sensor_model"] = "";
    input["-socket_type"]       = "";


    if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    std::string ft_sensor_topic         = input["-ft_sensor_topic"];
    std::string virtual_sensor_topic    = input["-virtual_sensor_topic"];
    std::string action_topic            = input["-action_topic"];
    std::string fixed_frame             = input["-fixed_frame"];
    std::string peg_link_name           = input["-peg_link_name"];
    std::string srate                   = input["-rate"];
    std::string path_sensor_model       = input["-path_sensor_model"];
    std::string ssocket_type            = input["-socket_type"];

    SOCKET_TYPE socket_type;

    if(ssocket_type == "one"){
        socket_type = SOCKET_TYPE::ONE;
    }else if(ssocket_type == "two"){
        socket_type = SOCKET_TYPE::TWO;
    }else if(ssocket_type == "three"){
        socket_type = SOCKET_TYPE::THREE;
    }else{
        ROS_ERROR_STREAM("No such socket type defined: "  + ssocket_type);
        return 0;
    }

    ros::init(argc, argv, "peg_filter");
    ros::NodeHandle nh;
    double Hz = boost::lexical_cast<float>(srate);
    ros::Rate rate(Hz);

    ROS_INFO_STREAM("loaded variables! [peg_filter_node]");

    /// ---------------------- Variables -----------------------------


    arma::colvec3    peg_origin;
    arma::mat33      peg_orient;


    /// ------------------  Virtual Sensor for Particles -------------------------

    Peg_world_wrapper peg_world_wrapper(nh,socket_type,true,"peg_world_wrapper",path_sensor_model,fixed_frame,peg_link_name); // don't publish needs the model for the likelihood
    psm::Contact_distance_model contact_distance_model(*(peg_world_wrapper.peg_sensor_model.get()));

    psm::Sensor_manager sensor_manager(nh);
    sensor_manager.add("contact",&contact_distance_model);

    if(!sensor_manager.select_model("contact")){
        ROS_ERROR("FAILED to select_model");
        exit(0);
    }


    /// ------------------  Sensor Variables -------------------------


    psm::Peg_sensor_listener ft_sensor_listener(nh,ft_sensor_topic);
    psm::Peg_sensor_listener virtual_sensor_listener(nh,virtual_sensor_topic);


    arma::colvec&    Y_ft       = ft_sensor_listener.Y;
    arma::colvec&    Y_virtual  = virtual_sensor_listener.Y;
    arma::colvec     Y_mixed;
    int Y_type                  = 3;

    Y_mixed.resize(10);

    opti_rviz::Publisher    pub_mix(nh,"mixed_classifier");




    /// ------------------  Point Mass Filter -------------------------
    ///
    ///
    ///
    ///

    bool bExperiment    = true;

    // Debug
    int init_pmf_type   =  10;
    int init_pmf_pos    =  1;

    // experiment
    int init_pmf_exp    =  1;
    int init_exp_pos    =  1;

    ///
    ///
    ///     Dimension of Measurement

    int dim_Y = 12;



    //likeli::Binary_likelihood   binary_likelihood;
    likeli::Mixed_likelihood    mixed_likelihood(nh,peg_world_wrapper.get_wrapped_objects(),peg_origin);

    pf::Measurement_h    peg_measurement        =  std::bind(&psm::Sensor_manager::compute_hY,&sensor_manager,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    pf::likelihood_model peg_likelihood;

    // the type of likelihood function will depend on the sensor (FT/virtual)

    if(Y_type == 0)
    {
    //    peg_likelihood   =  std::bind(&likeli::Binary_likelihood::likelihood,&binary_likelihood,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);

    }else if(Y_type == 2)
    {
        peg_likelihood   =  std::bind(&likeli::Mixed_likelihood::likelihood,&mixed_likelihood,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4);
    }else if (Y_type == 3)
    {

        if(socket_type == SOCKET_TYPE::ONE){
            ROS_WARN("SETTING LIKELIHOOD TO SOCKET ONE");
            peg_likelihood   =  std::bind(&likeli::Mixed_likelihood::likelihood,&mixed_likelihood,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4);

        }else if(socket_type == SOCKET_TYPE::TWO){

            ROS_WARN("SETTING LIKELIHOOD TO SOCKET TWO");
            peg_likelihood   =  std::bind(&likeli::Mixed_likelihood::likelihood_stwo,&mixed_likelihood,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4);


        }else if(socket_type == SOCKET_TYPE::THREE){

            ROS_WARN("SETTING LIKELIHOOD TO SOCKET THREE");
            peg_likelihood   =  std::bind(&likeli::Mixed_likelihood::likelihood_sthree,&mixed_likelihood,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4);

        }else{
            ROS_ERROR("PEG FILTER NO SUCH SOCKET TYPE");
        }


    }else{
        ROS_ERROR_STREAM("No such [" << Y_type << "] defined!");
        exit(0);
    }



    pf::Point_mass_filter::delta    delta_;
    pf::Point_mass_filter::length   length_;

    if(bExperiment){
        init_delta_length_experiment(init_pmf_exp,delta_,length_);
    }else{
        init_delta_length(init_pmf_type,delta_,length_);
   }

   pf::Point_mass_filter pmf(peg_likelihood,peg_measurement,delta_,length_,dim_Y);

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

    if(bExperiment){
        initialise_prior_experiment(init_pmf_exp,pmf,peg_position(0),peg_position(1),peg_position(2));
    }else{
        initialise_prior_pdf(init_pmf_pos,init_pmf_type,pmf,peg_position(0),peg_position(1),peg_position(2));
    }

    std::cout<< "pmf prior initialised [peg_filter_node]" << std::endl;

    //binary_likelihood.set_resolution(&pmf.get_delta());
    mixed_likelihood.set_resolution(&pmf.get_delta());

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
    plugfilter::Plug_service::Initialise initialise_f  = std::bind(initialise_prior_pdf,init_pmf_pos,init_pmf_type,std::ref(pmf),std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);
    plugfilter::Plug_service::Initialise initialise_ex = std::bind(initialise_prior_experiment,init_pmf_exp,std::ref(pmf),std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);

    if(bExperiment){
        plug_service.set_initilisation_function(&initialise_ex);
    }else{
        plug_service.set_initilisation_function(&initialise_f);
    }

    std::cout<< "Plug_service set [peg_filter_node]" << std::endl;

    /// Listeners Velocity

    opti_rviz::Listener     peg_tip_position_listener(fixed_frame,action_topic);
    tf::Vector3             peg_origin_tf;
    tf::Vector3             peg_origin_tf_tmp;
    tf::Matrix3x3           peg_orient_tf;


    peg_tip_position_listener.update(peg_origin_tf,peg_orient_tf);
    peg_origin_tf_tmp = peg_origin_tf;


    /// ------------------  Belief Compression ------------------


    Belief_compression     belief_compression;
    // advertised on topic [mode_feature]
    ModeBeliefCompress     mode_belief_compression(nh,"mode_entropy");

    belief_compression.add_bel_compress_method(&mode_belief_compression);
    if(belief_compression.set_method("mode_entropy"))
    {
        ROS_INFO("belief compression set [peg_filter_node]");
    }else{
        ROS_ERROR("belief compression not set [peg_filter_node]");
        exit(0);
    }


    /// ------------ Get Fist time Position and Orientation ------------



    tf::Matrix3x3 Rot_peg;
    Rot_peg.setRPY(0,0,M_PI);

    arma::colvec3    u;

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
        ROS_ERROR("Failed to call netft bias service [peg_filter_node]");
    }

    while(nh.ok() && !bBiasUpdated)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("bias reset");

    /// ------------ Wait a litte bit ------------
    {
        while(nh.ok() && Y_ft(0) != 0){
            std::cout << "waiting all zero Y: " << Y_ft(0) << " " << Y_ft(1) << " " << Y_ft(2) << std::endl;
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


   opti_rviz::Vis_point_cloud  vis_pc_rec(nh,"pfilter_record");
   vis_pc_rec.set_channel(opti_rviz::Vis_point_cloud::CHANNEL_TYPE::Intensity);
   vis_pc_rec.initialise("world",pmf.points);


   arma::colvec3 plan_norm;
    plan_norm(0) = 1; plan_norm(1) = 0; plan_norm(2) = 0;
    arma::colvec3 target,mode_SF, mode, tt, bb;

    target    = table_position;
    target(0) = target(0) + 0.026;

    ros::Time start_time = ros::Time::now();

    while(nh.ok()){

        // const auto start_time = std::chrono::steady_clock::now();

        peg_world_wrapper.update();

        peg_tip_position_listener.update(peg_origin_tf,peg_orient_tf);
        get_veclocity(u,peg_origin_tf,peg_origin_tf_tmp);

        peg_orient_tf.setRotation(qtmp);

        opti_rviz::type_conv::tf2mat(peg_orient_tf,peg_orient);
        opti_rviz::type_conv::tf2vec(peg_origin_tf,peg_origin);

        bcorrect_orient = arma::any(arma::vectorise(peg_orient));

        mode   = mode_belief_compression.get_mode();
        bb     = pmf.bbox.bb - socket_position;
        tt     = pmf.bbox.tt - socket_position;

        mode_SF =  mode - socket_position;

        if(!(Y_ft.is_empty()) && bcorrect_orient){

            opti_rviz::debug::tf_debuf(bb,"BB");
            opti_rviz::debug::tf_debuf(tt,"TT");

            if(Y_ft(0) == 1 && is_above_table(bb) && is_above_table(tt) && !is_in_socket_area(bb,tt)){
               // ROS_INFO_STREAM_THROTTLE(0.2,"----------------> Is above table");

                if(mode_SF.is_finite()){
                   // u(0) = 0.01 * (0.02 - mode_SF(0));
                }else{
                    ROS_INFO_STREAM_THROTTLE(1.0, "MODE_SF IS NOT FINITE");
                }

            }else if(Y_ft(0) == 1 && !Y_virtual(psm::Contact_distance_model::C_SOCKET))
            {
               // u(0) = 0;
            }

            switch (Y_type) {
            case 0:
            {
                plug_pf_manager.update(Y_ft,u,peg_orient,Hz);
                break;
            }
            case 1:
            {
                plug_pf_manager.update(Y_virtual,u,peg_orient,Hz);
                break;
            }
            case 2:
            {
                Y_mixed(0) = Y_ft(0);  // contact / no contact
                Y_mixed(1) = Y_ft(1);  // Fx
                Y_mixed(2) = Y_ft(2);  // Fy
                Y_mixed(3) = Y_ft(3);  // Fz
                Y_mixed(4) = Y_ft(4);  // prob_left_edge
                Y_mixed(5) = Y_ft(5);  // prob_right_edge
                Y_mixed(6) = Y_ft(6);  // prob_up_edge
                Y_mixed(7) = Y_ft(7);  // prob_down_edge
                pub_mix.publish(Y_mixed);
                plug_pf_manager.update(Y_mixed,u,peg_orient,Hz);
                break;
            }
            case 3:
            {

                /*
                    C_SURF          =0
                    C_EDGE_DIST     =1
                    C_EDGE_LEFT     =2
                    C_EDGE_RIGHT    =3
                    C_EDGE_TOP      =4
                    C_EDGE_BOT      =5
                    C_RING          =6
                    C_S_HOLE        =7
                    C_SOCKET        =8
                    C_EDGE_V1       =9
                    C_EDGE_V2       =10
                    C_EDGE_V3       =11
                */

                Y_mixed(0)  = Y_ft(0);                                                  // contact / no contact

                Y_mixed(1)  = Y_virtual(psm::Contact_distance_model::C_EDGE_DIST);      // Fx
                Y_mixed(2)  = Y_virtual(psm::Contact_distance_model::C_EDGE_V2);        // Fy
                Y_mixed(3)  = Y_virtual(psm::Contact_distance_model::C_EDGE_V3);        // Fz

                Y_mixed(4)  = Y_virtual(psm::Contact_distance_model::C_EDGE_LEFT);      // prob_left_edge
                Y_mixed(5)  = Y_virtual(psm::Contact_distance_model::C_EDGE_RIGHT);     // prob_right_edge
                Y_mixed(6)  = Y_virtual(psm::Contact_distance_model::C_EDGE_TOP);       // prob_up_edge
                Y_mixed(7)  = Y_virtual(psm::Contact_distance_model::C_EDGE_BOT);       // prob_down_edge
                Y_mixed(8)  = Y_virtual(psm::Contact_distance_model::C_SOCKET);         // Y_socket
                Y_mixed(9)  = Y_virtual(psm::Contact_distance_model::C_RING);           // Closest distance to ring edge

                pub_mix.publish(Y_mixed);
                plug_pf_manager.update(Y_mixed,u,peg_orient,Hz);

                break;
            }

            default:
                break;
            }

            /// belief feature (belief compression) computation
            belief_compression.update(u,pmf.points,pmf.P);
        }

        /// particle filter visualisation

        plug_pf_manager.visualise();

        /// particle filter vis for recording

        if( (ros::Time::now() - start_time).toSec() > 0.02 ){
            vis_pc_rec.update(pmf.points,pmf.P.memptr());
            vis_pc_rec.publish();
            start_time = ros::Time::now();
        }


        peg_origin_tf_tmp = peg_origin_tf;
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
