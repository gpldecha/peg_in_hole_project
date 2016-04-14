#include "peg_hole_policy/peg_hole_policy.h"
#include <control_toolbox/filters.h>
#include <optitrack_rviz/type_conversion.h>

#include <chrono>

namespace ph_policy{

Peg_hole_policy::Peg_hole_policy(ros::NodeHandle& nh,
                                 const std::string& fixed_frame,
                                 belief::Gmm_planner& gmm_planner,
                                 Peg_world_wrapper &peg_world_wrapper):
    ee_peg_listener(fixed_frame,"lwr_peg_link"),
    state_machine(nh,*(peg_world_wrapper.peg_sensor_model.get())),
    search_policy(nh,gmm_planner,state_machine,peg_world_wrapper.get_wrapped_objects(),*(peg_world_wrapper.peg_sensor_model.get())),
    vis_vector(nh,"direction"),
    ft_listener_(nh,"/ft_sensor/netft_data"),
    Ros_ee_j(nh),
    switch_controller(nh)
{

    server_srv              = nh.advertiseService("cmd_peg_policy",&Peg_hole_policy::cmd_callback,this);
    ft_classifier_sub_      = nh.subscribe("ft_classifier",1,&Peg_hole_policy::ft_classifier_callback,this);
    net_ft_sc_              = nh.serviceClient<netft_rdt_driver::String_cmd>("/ft_sensor/bias_cmd");
    x_des_subscriber        = nh.subscribe("/lwr/joint_controllers/des_ee_pos",1,&Peg_hole_policy::x_des_callback,this);

    world_frame             = fixed_frame;


    initial_config          = true;
    tf_count                = 0;
    control_rate            = 200.0;
    current_policy          = policy::NONE;


    grav_msg.data = false;

    arrows.resize(1);
    arrows[0].set_scale(0.01,0.015,0.015);
    arrows[0].set_rgba(1,1,0,1);
    vis_vector.initialise("world",arrows);



}

bool Peg_hole_policy::stop(){

}

bool Peg_hole_policy::update(){

    ROS_INFO_STREAM("Peg Hole Policy ON!");

    if(!switch_controller.activate_controller("joint_controllers")){
        return false;
    }
    current_policy           = policy::NONE;
    tf::Quaternion qdiff;

    tf::Vector3     velocity, velocity_tmp;


    ee_peg_listener.update(current_origin_WF,current_orient_WF);
    current_origin_tmp = current_origin_WF;

    ctrl_type       = ctrl_types::JOINT_POSITION;

    // start in open loop
    string_msg.data = "velocity_open";
    sendString(string_msg);

    bool success = true;
    ros::Rate       loop_rate(control_rate);
    ros::Time       time, last_time;
    ros::Duration   ros_dt;

    time = ros::Time::now();
    last_time = time;
    velocity.setZero();

    net_ft_reset_bias();
    arma::colvec3 force;

    q_des_q_.setRPY(0,0,0);

    ROS_INFO_STREAM("Staring Peg hole loop");
    while(ros::ok()) {

        last_time       = time;
        time            = ros::Time::now();
        ros_dt          = time - last_time;
        velocity_tmp    = velocity;

        ee_peg_listener.update(current_origin_WF,current_orient_WF);

        des_orient_WF   = current_orient_WF;
        des_origin_     = current_origin_WF;
        current_dx      = (current_origin_WF - current_origin_tmp) * (1.0 / control_rate);
        wrench_         = ft_listener_.current_msg.wrench;

        force(0)        = wrench_.force.x;
        force(1)        = wrench_.force.y;
        force(2)        = wrench_.force.z;

        search_policy.update_force_vis(force,current_origin_WF,current_orient_WF);
        opti_rviz::debug::tf_debuf(x_des_q_,q_des_q_,"x_des_q_");


        if(ctrl_type != ctrl_types::JOINT_POSITION){

            switch(current_policy){
            case policy::SEARCH_POLICY:
            {
                tf::Matrix3x3 current_orient;
                current_orient.setRotation(current_orient_WF);
                search_policy.get_velocity(velocity,des_orient_WF,current_origin_WF,current_orient,Y_c,wrench_);
                break;
            }
            default:
            {
                velocity.setZero();
                break;
            }
            };


            if(velocity.length() != 0){
                // assume constant velocity
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

            // Safety check magnitude of velocity
            if(velocity.length() >= 0.2){
                ROS_WARN_STREAM_THROTTLE(1.0,"velocity magnite[" << velocity.length() << "] to large! [peg_hole_policy]");
                velocity.setZero();
            }

            /// Plot direction velocity
            arrows[0].origin    = current_origin_WF;
            arrows[0].direction = velocity;
            arrows[0].direction = arrows[0].direction.normalize();
            arrows[0].direction = 0.1 *  arrows[0].direction;

            vis_vector.update(arrows);
            vis_vector.publish();

            /// COMPUTE ANGULAR VELOCITY
            qdiff = des_orient_WF - current_orient_WF;
            Eigen::Quaternion<double>  dq (qdiff.getW(),qdiff.getX(),qdiff.getY(),qdiff.getZ());
            Eigen::Quaternion<double>   q(current_orient_WF.getW(),current_orient_WF.getX(),current_orient_WF.getY(), current_orient_WF.getZ());
            Eigen::Vector3d w = motion::d2qw<double>(q,dq);

            //velocity.setZero();

            linear_vel_cmd_(0)  = velocity[0];
            linear_vel_cmd_(1)  = velocity[1];
            linear_vel_cmd_(2)  = velocity[2];

            angular_vel_cmd_(0) = w[0];
            angular_vel_cmd_(1) = w[1];
            angular_vel_cmd_(2) = w[2];

            orientation_cmd_ = Eigen::Quaterniond(des_orient_WF.w(),des_orient_WF.x(),des_orient_WF.y(),des_orient_WF.z());

            ROS_INFO_STREAM_THROTTLE(1.0,"linear_vel_cmd_: " << linear_vel_cmd_[0] << " " << linear_vel_cmd_[1] << " " << linear_vel_cmd_[2] );

            //linear_vel_cmd_.setZero();

            // SET COMMAND VALUES
            set_command(linear_vel_cmd_,angular_vel_cmd_,orientation_cmd_);
        }


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

    return success;
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
    //  std::cout<< "pos: " << pos.position.x << " " << pos.position.y << " " << pos.position.z << std::endl;
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

void Peg_hole_policy::ft_classifier_callback(const std_msgs::Float32MultiArray& msg){

    if(msg.data.size() == 3){
        Y_c(0) = msg.data[0];
        Y_c(1) = msg.data[1];
        Y_c(2) = msg.data[2];
    }else{
        ROS_WARN_STREAM_THROTTLE(1.0,"Peg_hole_policy::ft_classifier_callback: msg.data.size(): " << msg.data.size());
    }

}

bool Peg_hole_policy::cmd_callback(peg_hole_policy::String_cmd::Request& req, peg_hole_policy::String_cmd::Response& res){

    std::string cmd = req.cmd;

    ROS_INFO("cmd_callback:: Peg_hole_policy");
    std::cout<< "cmd: " << cmd << std::endl;
    if (cmd == "gmm"){
        current_policy = policy::SEARCH_POLICY;
        ctrl_type      = ctrl_types::CARTESIAN;
        res.res        = "gmr policy ON!";
        search_policy.reset();
        grav_msg.data  = false;
        sendGrav(grav_msg);
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
        search_policy.set_action(actions::FIND_SOCKET_HOLE);
        search_policy.set_socket_policy(SOCKET_POLICY::INSERT);
        res.res = "insert..!";
        ros::spinOnce();
    }else if(cmd=="go_socket"){
        search_policy.set_action(actions::FIND_SOCKET_HOLE);
        search_policy.set_socket_policy(SOCKET_POLICY::TO_SOCKET);
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
    }else if(cmd == "grav_comp"){
        current_policy = policy::NONE;
        ctrl_type      = ctrl_types::JOINT_POSITION;
        grav_msg.data  = true;//!grav_msg.data;
        if(grav_msg.data){res.res = "gravity: false";}else{res.res = "gravity: true";}
        sendGrav(grav_msg);
    }else{
        res.res = "no such cmd [" + cmd + "]!";
        return false;
    }
    std::cout<< res.res << std::endl;


    return true;
}






}
