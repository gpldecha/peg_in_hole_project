#include "peg_hole_policy/peg_hole_policy.h"
#include <control_toolbox/filters.h>
#include <optitrack_rviz/type_conversion.h>
#include <optitrack_rviz/publisher.h>

#include <chrono>

namespace ph_policy{

Peg_hole_policy::Peg_hole_policy(ros::NodeHandle& nh,
                                 const std::string& fixed_frame,
                                 const std::string& ft_topic,
                                 const std::string& ft_classifier_topic,
                                 const std::string& belief_state_topic,
                                 const std::string& record_topic_name,
                                 Peg_sensor_model&  peg_sensor_model,
                                 wobj::WrapObject& wrapped_objects):
    world_frame(fixed_frame),
    ee_peg_listener(fixed_frame,"lwr_peg_link"),
    get_back_on(wrapped_objects.get_wbox("link_wall")),
    vis_vector(nh,"direction"),
    ft_listener_(nh,"/ft_sensor/netft_data"),
    peg_sensor_model(peg_sensor_model),
    state_machine(peg_sensor_model),
    Ros_ee_j(nh),
    switch_controller(nh),
    pub_ee_pos_SF(nh,"ee_pos_SF"),
    pub_belief_SF(nh,"belief_feature_SF")
{

    {
        tf::StampedTransform transform;
        opti_rviz::Listener::get_tf_once(world_frame,"link_socket",transform);
        opti_rviz::type_conv::tf2vec(transform.getOrigin(),socket_pos_WF);
        opti_rviz::type_conv::tf2mat(transform.getBasis(),Rt);
        opti_rviz::type_conv::tf2vec(transform.getOrigin(),T);
        Rt          = Rt.st();
    }

    belief_state_WF.resize(4);
    belief_state_SF.resize(4);

    server_srv                  = nh.advertiseService("cmd_peg_policy",&Peg_hole_policy::cmd_callback,this);
    net_ft_sc_                  = nh.serviceClient<netft_rdt_driver::String_cmd>("/ft_sensor/bias_cmd");
    x_des_subscriber            = nh.subscribe("/lwr/joint_controllers/des_ee_pos",1,&Peg_hole_policy::x_des_callback,this);

    sensor_classifier_sub_      = nh.subscribe(ft_classifier_topic,1,&Peg_hole_policy::sensor_classifier_callback,this);
    belief_info_sub             = nh.subscribe(belief_state_topic,1,&Peg_hole_policy::belief_state_callback,this);

    openloopx_sub               = nh.subscribe("/lwr/joint_controllers/x_open_loop",1,&Peg_hole_policy::Peg_hole_policy::openloopx_callback,this);

    belief_mode_reset_pub_      = nh.advertise<std_msgs::Bool>("belief_features/reset",1);


    record_client               = nh.serviceClient<record_ros::String_cmd>(record_topic_name);


    world_frame                 = fixed_frame;


    initial_config              = true;
    tf_count                    = 0;
    control_rate                = 200.0;
    current_policy              = policy::NONE;
    grav_msg.data               = false;

    arrows.resize(1);
    arrows[0].set_scale(0.01,0.015,0.015);
    arrows[0].set_rgba(1,1,0,1);
    vis_vector.initialise("world",arrows);



    specialised_policy.reset(   new ph_policy::Specialised(peg_sensor_model.get_wrapped_objects())                     );
    gmm_policy.reset(           new ph_policy::GMM()                                                                                        );
    search_policy.reset(        new ph_policy::Search_policy(nh,get_back_on,*(specialised_policy.get()),*(gmm_policy.get()),state_machine,peg_sensor_model)  );



    /// ------------------ Publishers for transformed frame of reference


    // if want to record
    bRecord=true;

}

bool Peg_hole_policy::stop(){
    return true;
}

void   Peg_hole_policy::print_debug(){

}

void Peg_hole_policy::reset(){
    current_policy           = policy::NONE;
    ctrl_type                = ctrl_types::JOINT_POSITION;


    ee_peg_listener.update(current_origin_WF,current_orient_WF);
    current_origin_tmp = current_origin_WF;

    net_ft_reset_bias();

    q_des_q_.setRPY(0,0,0);

}

void Peg_hole_policy::rviz_velocity(){
    /// Plot direction velocity
    arrows[0].origin    = current_origin_WF;
    arrows[0].direction = velocity;
    arrows[0].direction = arrows[0].direction.normalize();
    arrows[0].direction = 0.1 *  arrows[0].direction;

    vis_vector.update(arrows);
    vis_vector.publish();
}

void Peg_hole_policy::set_angular_velocity(){

    /// COMPUTE ANGULAR VELOCITY
    qdiff = des_orient_WF - current_orient_WF;
    Eigen::Quaternion<double>  dq (qdiff.getW(),qdiff.getX(),qdiff.getY(),qdiff.getZ());
    Eigen::Quaternion<double>   q(current_orient_WF.getW(),current_orient_WF.getX(),current_orient_WF.getY(), current_orient_WF.getZ());
    Eigen::Vector3d w = motion::d2qw<double>(q,dq);

    angular_vel_cmd_(0) = w[0];
    angular_vel_cmd_(1) = w[1];
    angular_vel_cmd_(2) = w[2];

}


bool Peg_hole_policy::update(){

    ROS_INFO_STREAM("Peg Hole Policy ON!");

    if(!switch_controller.activate_controller("joint_controllers")){
        return false;
    }

    ROS_INFO("Reset");
    reset();

    ros::Time time = ros::Time::now();



    ROS_INFO_STREAM("Staring Peg hole loop");
    ros::Rate loop_rate(control_rate);
    while(ros::ok()) {
        time            = ros::Time::now();
        velocity_tmp    = velocity;

        ee_peg_listener.update(current_origin_WF,current_orient_WF);
        opti_rviz::type_conv::tf2vec(current_origin_WF,arma_current_origin_WF);


        des_orient_WF       = current_orient_WF;
        des_origin_         = current_origin_WF;


        opti_rviz::type_conv::tf2vec(current_origin_WF,arma_current_origin_WF);

        current_origin_SF   = arma_current_origin_WF - socket_pos_WF;

        current_dx      = (current_origin_WF - current_origin_tmp) * (1.0 / control_rate);
        wrench_         = ft_listener_.current_msg.wrench;
        force(0)        = wrench_.force.x;
        force(1)        = wrench_.force.y;
        force(2)        = wrench_.force.z;

        search_policy->update_force_vis(force,current_origin_WF,current_orient_WF);
        opti_rviz::debug::tf_debuf(x_des_q_,q_des_q_,"x_des_q_");



        if(ctrl_type != ctrl_types::JOINT_POSITION){

            switch(current_policy){
            case policy::SEARCH_POLICY:
            {

                if(bFirst)
                {
                    bFirst=false;
                    check_record(true);
                }

                tf::Matrix3x3 current_orient;
                current_orient.setRotation(current_orient_WF);
                search_policy->get_velocity(velocity,des_orient_WF,arma_current_origin_WF,current_orient,Y_c,force,belief_state_WF,belief_state_SF,socket_pos_WF,open_loop_x_origin_arma_WF);
                opti_rviz::debug::tf_debuf(open_loop_x_origin_tf,open_loop_x_orient_tf,"x_open_loop");

                search_policy_type = search_policy->get_policy();

                if(arma::norm(open_loop_x_origin_arma_WF - arma_current_origin_WF) > 0.035)
                {
                    ROS_WARN_THROTTLE(1.0,"open loop target too far away from current position!");
                    velocity.setZero();
                }

                break;
            }
            default:
            {
                velocity.setZero();
                break;
            }
            };


            smooth_velocity();

            rviz_velocity();

            set_angular_velocity();

            // Debug
            //velocity.setZero();


            opti_rviz::type_conv::tf2vec(velocity,linear_vel_cmd_);
            orientation_cmd_ = Eigen::Quaterniond(des_orient_WF.w(),des_orient_WF.x(),des_orient_WF.y(),des_orient_WF.z());
            ROS_INFO_STREAM_THROTTLE(1.0,"linear_vel_cmd_: " << linear_vel_cmd_[0] << " " << linear_vel_cmd_[1] << " " << linear_vel_cmd_[2] );
            // SET COMMAND VALUES
            set_command(linear_vel_cmd_,angular_vel_cmd_,orientation_cmd_);
        }


        pub_belief_SF.publish(belief_state_SF);
        pub_ee_pos_SF.publish(current_origin_SF);
        ros::spinOnce();
        loop_rate.sleep();
        current_origin_tmp = current_origin_WF;
    }

    peg_hole_policy::String_cmd::Request  req;
    peg_hole_policy::String_cmd::Response res;
    req.cmd = "grav_comp";
    cmd_callback(req,res);

    current_policy = policy::NONE;
    linear_vel_cmd_.setZero();
    angular_vel_cmd_.setZero();

    ROS_INFO_STREAM("Peg Hole Policy OFF!");

    return true;
}

void Peg_hole_policy::smooth_velocity(){
    if(velocity.length() != 0){
        // assume constant velocity

        if(search_policy_type == Search_policy::POLICY::INSERT){
            ROS_INFO_STREAM_THROTTLE(0.5,"NEW INSERT");
        }else{

            velocity = velocity / (velocity.length() + std::numeric_limits<double>::min());
            velocity = 0.01 * velocity;

            for(std::size_t i = 0; i < 3;i++){
                velocity[i]  = filters::exponentialSmoothing(velocity[i],velocity_tmp[i], 0.02);
                if(std::isnan(velocity[i]) || std::isinf(velocity[i]))
                {
                    velocity[i] = 0;
                    ROS_WARN_THROTTLE(0.1,"velocity is nan or inf (peg_hole_policy)");
                }
            }

        }
    }

    // Safety check magnitude of velocity
    if(velocity.length() >= 0.2){
        ROS_WARN_STREAM_THROTTLE(1.0,"velocity magnite[" << velocity.length() << "] to large! [peg_hole_policy]");
        velocity.setZero();
    }
}

void Peg_hole_policy::set_command(const Eigen::Vector3d& linear_vel_cmd,
                                  const Eigen::Vector3d& angular_vel_cmd,
                                  const Eigen::Quaterniond &orientation_cmd){

    /// LINEAR VELOCITY
    des_ee_vel_msg.linear.x = linear_vel_cmd[0];
    des_ee_vel_msg.linear.y = linear_vel_cmd[1];
    des_ee_vel_msg.linear.z = linear_vel_cmd[2];

    /// ANGULAR VELOCITY
    des_ee_vel_msg.angular.x = angular_vel_cmd[0];
    des_ee_vel_msg.angular.y = angular_vel_cmd[1];
    des_ee_vel_msg.angular.z = angular_vel_cmd[2];

    /// ORIENTATION
    orient_msg.x = orientation_cmd.x();
    orient_msg.y = orientation_cmd.y();
    orient_msg.z = orientation_cmd.z();
    orient_msg.w = orientation_cmd.w();

    sendCartVel(des_ee_vel_msg);
    sendOrient(orient_msg);
}

void Peg_hole_policy::disconnect(){

    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once("world","lwr_peg_link",transform);
    tf::Vector3 target = transform.getOrigin();
    target[0] = target[0] + 0.05;
    opti_rviz::type_conv::tf2geom(target,current_orient_WF,ee_pos_msg);
    sendCartPose(ee_pos_msg);
}

void Peg_hole_policy::x_des_callback(const geometry_msgs::Pose& pos){
    x_des_q_.setX(pos.position.x);
    x_des_q_.setY(pos.position.y);
    x_des_q_.setZ(pos.position.z);
    q_des_q_.setX(pos.orientation.x);
    q_des_q_.setY(pos.orientation.y);
    q_des_q_.setZ(pos.orientation.z);
    q_des_q_.setW(pos.orientation.w);
}

bool Peg_hole_policy::net_ft_reset_bias(){

    net_ft_msg.request.cmd  = "bias";
    if(net_ft_sc_.call(net_ft_msg)){
        ROS_INFO_STREAM("reset_bias: " << net_ft_msg.response.res);
        return true;
    }else{
        ROS_ERROR("failed to reset bias");
        return false;
    }
}

void Peg_hole_policy::sensor_classifier_callback(const std_msgs::Float64MultiArray& msg){

    if(msg.data.size()  != Y_c.n_elem)
    {
        Y_c.resize(msg.data.size());
    }
    for(std::size_t i = 0; i < Y_c.n_elem;i++){
        Y_c(i) = msg.data[i];
    }
}

void Peg_hole_policy::belief_state_callback(const std_msgs::Float64MultiArrayConstPtr &msg){

    if(msg->data.size() == belief_state_WF.n_elem){
        for(std::size_t i = 0; i < msg->data.size();i++){
            belief_state_WF(i) = msg->data[i];
        }

        pos_tmp(0) = belief_state_WF(0);
        pos_tmp(1) = belief_state_WF(1);
        pos_tmp(2) = belief_state_WF(2);

        pos_tmp = Rt * pos_tmp - Rt * T;

        belief_state_SF(0) = pos_tmp(0);
        belief_state_SF(1) = pos_tmp(1);
        belief_state_SF(2) = pos_tmp(2);
        belief_state_SF(3) = belief_state_WF(3);

        opti_rviz::debug::tf_debuf(pos_tmp,"mode_SF_1/pos_tmp");

        //  ROS_INFO_STREAM_THROTTLE(1.0,"Gmm_planner belief_state: " << belief_state(0) << " " << belief_state(1) << " "
        //                           << belief_state(2));
    }else{
        ROS_WARN_STREAM_THROTTLE(1.0,"msg->data.size() != belief_state.n_elem [Peg_hole_policy::belief_state_callback]");
    }
}

void Peg_hole_policy::check_record(bool start){
    if(bRecord){
        if(start){
            record_ros::String_cmd  record_cmd;
            record_cmd.request.cmd      = "record";
            record_client.call(record_cmd);
            ROS_INFO("====> Starting to record!");
        }else if(!start){
            record_ros::String_cmd  record_cmd;
            record_cmd.request.cmd      = "stop";
            record_client.call(record_cmd);
            ROS_INFO("====> Stopping to record!");
        }
    }
}

void Peg_hole_policy::openloopx_callback(const geometry_msgs::PoseStampedConstPtr &msg){


    open_loop_x_origin_tf.setX(msg->pose.position.x);
    open_loop_x_origin_tf.setY(msg->pose.position.y);
    open_loop_x_origin_tf.setZ(msg->pose.position.z);

    opti_rviz::type_conv::tf2vec(open_loop_x_origin_tf,open_loop_x_origin_arma_WF);

    open_loop_x_orient_tf.setX(msg->pose.orientation.x);
    open_loop_x_orient_tf.setY(msg->pose.orientation.y);
    open_loop_x_orient_tf.setZ(msg->pose.orientation.z);
    open_loop_x_orient_tf.setW(msg->pose.orientation.w);

}

bool Peg_hole_policy::cmd_callback(peg_hole_policy::String_cmd::Request& req, peg_hole_policy::String_cmd::Response& res){

    std::string cmd = req.cmd;

    ROS_INFO("cmd_callback:: Peg_hole_policy");
    std::cout<< "cmd: " << cmd << std::endl;
    if (cmd == "special"){
        reset_belief(res.res);
        current_policy = policy::SEARCH_POLICY;
        ctrl_type      = ctrl_types::CARTESIAN;
        res.res        = "special policy ON!";
        std::vector<std::string> args = {{"reset"}};
        search_policy->reset();
        search_policy->command("special",args);
        grav_msg.data  = false;
        sendGrav(grav_msg);

    }else if(cmd == "greedy"){
        reset_belief(res.res);

        current_policy = policy::SEARCH_POLICY;
        ctrl_type      = ctrl_types::CARTESIAN;

        search_policy->reset();
        search_policy->command("greedy");


    }else if(cmd == "gmm"){
        reset_belief(res.res);

        current_policy = policy::SEARCH_POLICY;
        ctrl_type      = ctrl_types::CARTESIAN;
        search_policy->reset();
        search_policy->command("gmm");

    }else if(cmd == "qem"){
        reset_belief(res.res);

        current_policy = policy::SEARCH_POLICY;
        ctrl_type      = ctrl_types::CARTESIAN;
        search_policy->reset();
        search_policy->command("qem");


    }else if(cmd == "pause"){
        current_policy = policy::NONE;
        ctrl_type      = ctrl_types::CARTESIAN;
        res.res = "pause!";
    }else if(cmd == "go_front"){
        current_policy = policy::NONE;
        ctrl_type      = ctrl_types::JOINT_POSITION;
        joint_pos_msg.data  = {{-1.02974,0.471239,0.401426,-1.76278,-1.0472,-0.802851,0.785398}};
        damp_msg.data       = {{0.7,0.7,0.7,0.7,0.7,0.7,0.7}};
        stiff_msg.data      = {{100,100,100,100,100,100,100}};
        sendDamp(damp_msg);
        sendStiff(stiff_msg);
        sendJointPos(joint_pos_msg);
        res.res = "going front";
        ros::spinOnce();
    }else if(cmd == "go_peg_right"){
        current_policy      = policy::NONE;
        ctrl_type           = ctrl_types::JOINT_POSITION;
        {
            using namespace opti_rviz;
            joint_pos_msg.data  = {{deg2rad(-29.2),deg2rad(34.35),deg2rad(20),deg2rad(-105),deg2rad(-16.13),deg2rad(-47.46),deg2rad(0)}};
        }
        damp_msg.data       = {{0.7,0.7,0.7,0.7,0.7,0.7,0.7}};
        stiff_msg.data      = {{100,100,100,100,100,100,100}};
        sendDamp(damp_msg);
        sendStiff(stiff_msg);
        sendJointPos(joint_pos_msg);
        res.res = "going front";
        ros::spinOnce();
    }else if(cmd == "insert"){
        current_policy = policy::SEARCH_POLICY;
        ctrl_type      = ctrl_types::CARTESIAN;
        search_policy->command("insert");
        res.res = "insert..!";
        ros::spinOnce();
    }else if(cmd=="go_socket"){
        // search_policy.set_action(actions::FIND_SOCKET_HOLE);
        // search_policy.set_socket_policy(SOCKET_POLICY::TO_SOCKET);
        res.res = "socket..!";
    }else if(cmd == "disconnect"){
        disconnect();
        res.res = "disconnecting..!";
        ros::spinOnce();
    }else if(cmd == "go_left"){
        current_policy = policy::NONE;
        ctrl_type      = ctrl_types::JOINT_POSITION;
        joint_pos_msg.data = {{0.803,0.4995,0.0286,-1.986,0.9915,-1.1997,-0.5516}};
        sendJointPos(joint_pos_msg);
        res.res = "going left";
        ros::spinOnce();
    }else if(cmd == "home"){
        current_policy = policy::NONE;
        ctrl_type      = ctrl_types::JOINT_POSITION;
        joint_pos_msg.data  =  {{0,0.785398,0.122173,-2.01099,-0.174533,0.261799,0}};
        sendJointPos(joint_pos_msg);
        res.res = "going home";
        ros::spinOnce();
    }else if(cmd == "stop"){
        check_record(false);

        current_policy = policy::NONE;
        ctrl_type      = ctrl_types::JOINT_POSITION;
        grav_msg.data  = true;
        if(grav_msg.data){res.res = "gravity: false";}else{res.res = "gravity: true";}
        sendGrav(grav_msg);

        bFirst=true;

    }else if(cmd == "bel_reset"){
        reset_belief(res.res);
    }else{
        res.res = "no such cmd [" + cmd + "]!";
        return false;
    }
    std::cout<< res.res << std::endl;


    return true;
}

void Peg_hole_policy::reset_belief(std::string& res){
    res = res + "--------> Rest belief Mode";
    std_msgs::Bool bmsg;
    bmsg.data=true;
    belief_mode_reset_pub_.publish(bmsg);
}






}
