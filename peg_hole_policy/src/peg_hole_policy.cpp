#include "peg_hole_policy/peg_hole_policy.h"
#include <control_toolbox/filters.h>
#include <chrono>
namespace ph_policy{

Peg_hole_policy::Peg_hole_policy(ros::NodeHandle& nh,
                                 const std::string& path_sensor_model,
                                 const std::string& fixed_frame):
    Base_ee_j_action(nh,"joint_controllers"),
    Base_action_server(nh),
    bel_planner(nh,),
    state_machine(nh,"peg_sensor_classifer"),
    find_table(nh,path_sensor_model,fixed_frame),
    find_socket(nh,path_sensor_model,fixed_frame),
    insert_peg(nh,path_sensor_model,fixed_frame),
    ee_peg_listener(fixed_frame,"lwr_peg_link"),
    vis_vector(nh,"direction"),
    velocity_controller(nh),
    ft_listener_(nh,"tf_sensor/netft_data")
{
    /*
     * ros::NodeHandle&   nh,
                const std::string& world_frame,
                const std::string& sensor_topic,
                const std::string& ft_classifier_topic,
                const std::string& bel_feature_topic,
                std::string path_parameters*/


    server_srv              = nh.advertiseService("cmd_peg_policy",&Peg_hole_policy::cmd_callback,this);
    ft_classifier_sub_      = nh.subscribe("ft_classifier",10,&Peg_hole_policy::ft_classifier_callback,this);
    net_ft_sc_              = nh.serviceClient<netft_rdt_driver::String_cmd>("/tf_sensor/bias_cmd");


    world_frame             = fixed_frame;


    initial_config          = true;
    tf_count                = 0;
    control_rate            = 100.0;
    current_policy          = NONE;

    linear_cddynamics.reset( new motion::CDDynamics(3,0.01,4) );
    angular_cddynamics.reset( new motion::CDDynamics(3,0.01,4) );

    nd_cdd = ros::NodeHandle("cdd");

    dynamic_server_cdd_param.reset(new dynamic_reconfigure::Server< peg_hole_policy::cdd_filterConfig>(nd_cdd));
    dynamic_server_cdd_param->setCallback( boost::bind(&Peg_hole_policy::cdd_callback,    this, _1, _2));

    motion::Vector velLimits(3);
    for(std::size_t i = 0; i < 3; i++){
        velLimits(i)  = 0.04; // x ms^-1
    }
    linear_cddynamics->SetVelocityLimits(velLimits);

    for(std::size_t i = 0; i < 3; i++){
        velLimits(i)  = 0.02; // x ms^-1
    }
    angular_cddynamics->SetVelocityLimits(velLimits);

    stiffness.resize(7);
    stiffness_tmp.resize(7);
    for(std::size_t i = 0; i < KUKA_NUM_JOINTS;i++){
        stiffness[i]         = 200;
        stiffness_tmp[i]     = 200;
        stiff_msg.data[i] = stiffness[i];
        damp_msg.data[i]  = 0;
    }
    bGrav=false;
    grav_msg.data = false;

    vis_vector.scale = 1;
    vis_vector.g     = 0;
    vis_vector.b     = 0;
    vis_vector.r     = 1;
    arrows.resize(1);

    arrows[0].shaft_diameter = 0.01;
    arrows[0].head_diameter  = 0.015;
    arrows[0].head_length    = 0.015;

    vis_vector.initialise("world",arrows);

    velocity_reguliser_ = Velocity_reguliser(0.2,0.1);


    std::string path_parameters = "/home/guillaume/MatlabWorkSpace/peg_in_hole_RL/PolicyModelSaved/PolicyModel_txt/gmm_xhu";
    std::vector<std::size_t> in  = {{0,1,2,3}};
    std::vector<std::size_t> out = {{4,5,6}};
    gma_planner = planners::GMAPlanner(path_parameters,in,out);

}



bool Peg_hole_policy::execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal){

    if(!activate_controller("joint_controllers")){
        return false;
    }
    current_policy           = NONE;
    double min_speed         = 0.1;
    double max_speed         = 0.15;

    /// setting bel curve max and min speeds

    if(min_speed <= 0.0){
        max_speed = 0.01;
        ROS_WARN("max_speed not set in goal, max speed set to default: 1cm/s");
    }

    find_table.beta_vel_reg     = 0.1;
    find_socket.beta_vel_reg    = 0.1;
    insert_peg.beta_vel_reg     = 0.1;

    find_table.velocity_reguliser.set_max_speed_ms(max_speed);
    find_socket.velocity_reguliser.set_max_speed_ms(max_speed);
    insert_peg.velocity_reguliser.set_max_speed_ms(max_speed);

    find_table.velocity_reguliser.set_min_speed_ms(min_speed);
    find_socket.velocity_reguliser.set_min_speed_ms(min_speed);
    insert_peg.velocity_reguliser.set_min_speed_ms(min_speed);

    tf::Quaternion qdiff;

    tf::Vector3     velocity, velocity_tmp;
    tf::Vector3     current_origin, current_origin_tmp;
    tf::Quaternion  current_orient_WF;


    motion::Vector filter_vel(3);
    filter_vel.setZero();
    linear_cddynamics->SetState(filter_vel);
    linear_cddynamics->SetDt(1.0/control_rate);


    ee_peg_listener.update(current_origin,current_orient_WF);
    current_origin_tmp = current_origin;

    static tf::TransformBroadcaster br;

    ctrl_type = JOINT_POSITION;

    bool success = true;
    ros::Rate       loop_rate(control_rate);
    ros::Time       time, last_time;
    ros::Duration   ros_dt;

    time = ros::Time::now();
    last_time = time;
    velocity.setZero();

    net_ft_reset_bias();

    while(ros::ok()) {

        last_time       = time;
        time            = ros::Time::now();
        ros_dt          = time - last_time;
        velocity_tmp    = velocity;

        ee_peg_listener.update(current_origin,current_orient_WF);
        des_orient_WF   = current_orient_WF;
        des_origin_     = current_origin;
        current_dx      = (current_origin - current_origin_tmp) * (1.0 / control_rate);
        wrench_         = ft_listener_.current_msg.wrench;


        state_machine.update(Y_c,wrench_);

        if(ctrl_type != JOINT_POSITION){

            switch(current_policy){
            case FIND_TABLE:
            {
                find_table.get_linear_velocity(velocity,current_origin,des_origin_,des_orient_WF);

                tf::Quaternion tmp2;
                tmp2.setRPY(0,-M_PI/2,0);
                des_orient_WF = des_orient_WF * tmp2;
                break;
            }
            case FIND_SOCKET:
            {
                find_socket.get_linear_velocity(velocity,current_origin,des_origin_,des_orient_WF);
                tf::Quaternion tmp2;
                tmp2.setRPY(0,-M_PI/2,0);
                des_orient_WF = tmp2 * des_orient_WF;
                break;
            }
            case FIND_HOLE:
            {
                insert_peg.get_linear_velocity(velocity,current_origin);
                tf::Quaternion tmp2;
                tmp2.setRPY(0,-M_PI/2,0);
                des_orient_WF = tmp2 * des_orient_WF;
                break;
            }
            case SEARCH_POLICY:
            {
                gma_planner.condition(F);
                gma_planner.get_ee_linear_velocity(gma_velocity);
                velocity[0] = gma_velocity[0];
                velocity[1] = gma_velocity[1];
                velocity[2] = gma_velocity[2];

                tf::Quaternion tmp2;
                tmp2.setRPY(0,-M_PI/2,0);
                des_orient_WF = tmp2 * des_orient_WF;
                break;
            }
            default:
            {

                velocity.setZero();
                break;
            }
            }

            // assume constant velocity
            velocity = velocity / (velocity.length() + std::numeric_limits<double>::min());
            velocity = 0.10 * velocity;

            for(std::size_t i = 0; i < 3;i++){
                velocity[i]  = filters::exponentialSmoothing(velocity[i],velocity_tmp[i], 0.2);
                if(std::isnan(velocity[i]) || std::isinf(velocity[i]))
                {
                    velocity[i] = 0;
                    ROS_WARN_THROTTLE(0.1,"velocity is nan or inf (peg_hole_policy)");
                }
            }



            /// COMPUTE ANGULAR VELOCITY
            qdiff = des_orient_WF - current_orient_WF;
            Eigen::Quaternion<double>  dq (qdiff.getW(),qdiff.getX(),qdiff.getY(),qdiff.getZ());
            Eigen::Quaternion<double>   q(current_orient_WF.getW(),current_orient_WF.getX(),current_orient_WF.getY(), current_orient_WF.getZ());
            Eigen::Vector3d w = motion::d2qw<double>(q,dq);

            linear_vel_cmd_(0)  = velocity[0];
            linear_vel_cmd_(1)  = velocity[1];
            linear_vel_cmd_(2)  = velocity[2];

            angular_vel_cmd_(0) = w[0];
            angular_vel_cmd_(1) = w[1];
            angular_vel_cmd_(2) = w[2];

            orientation_cmd_ = Eigen::Quaterniond(des_orient_WF.w(),des_orient_WF.x(),des_orient_WF.y(),des_orient_WF.z());


            arrows[0].origin    = current_origin;
            arrows[0].direction = velocity;
            arrows[0].direction = arrows[0].direction.normalize();
            arrows[0].direction = 0.1 *  arrows[0].direction;
            vis_vector.update(arrows);
            vis_vector.publish();



            tf::Transform transform;
            transform.setOrigin(current_origin);
            transform.setRotation(current_orient_WF);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "current"));

            transform.setOrigin(des_origin_);
            transform.setRotation(des_orient_WF);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "target"));



            // SET COMMAND VALUES
            set_command(linear_vel_cmd_,angular_vel_cmd_,orientation_cmd_);


        }



        feedback.progress = 0;
        as_.publishFeedback(feedback);
        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("Preempted");
            as_.setPreempted();
            success = false;
            break;
        }

        ros::spinOnce();
        loop_rate.sleep();
        current_origin_tmp = current_origin;
    }


    peg_hole_policy::String_cmd::Request  req;
    peg_hole_policy::String_cmd::Response res;
    req.cmd = "grav_comp";
    cmd_callback(req,res);

    current_policy = NONE;
    linear_vel_cmd_.setZero();
    angular_vel_cmd_.setZero();

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

void Peg_hole_policy::cdd_callback(peg_hole_policy::cdd_filterConfig& config, uint32_t level){

    linear_cddynamics->SetWn(config.Wn);

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
    if(cmd == "go_table"){
        current_policy = FIND_TABLE;
        ctrl_type      = CARTESIAN;
        res.res = "going to table!";
    }else if(cmd == "go_socket"){
        current_policy = FIND_SOCKET;
        ctrl_type      = CARTESIAN;
        res.res = "going to socket!";
    }else if(cmd == "insert"){
        current_policy = FIND_HOLE;
        ctrl_type      = CARTESIAN;
        res.res = "going to connect peg and socket!";
    }else if(cmd == "pause"){
        current_policy = NONE;
        ctrl_type      = CARTESIAN;
        res.res = "pause!";
    }else if(cmd == "go_front"){
        current_policy = NONE;
        ctrl_type      = JOINT_POSITION;
        joint_pos_msg.data = {{-1.02974,0.471239,0.401426,-1.76278,-1.0472,-0.802851,0.785398}};
        sendJointPos(joint_pos_msg);
        res.res = "going front";
        ros::spinOnce();
    }else if(cmd == "go_left"){
        current_policy = NONE;
        ctrl_type      = JOINT_POSITION;
        joint_pos_msg.data = {{0.803,0.4995,0.0286,-1.986,0.9915,-1.1997,-0.5516}};
        sendJointPos(joint_pos_msg);
        res.res = "going left";
        ros::spinOnce();
    }else if(cmd == "home"){
        current_policy = NONE;
        ctrl_type      = JOINT_POSITION;
        joint_pos_msg.data  =  {{0,0.785398,0.122173,-2.01099,-0.174533,0.261799,0}};
        sendJointPos(joint_pos_msg);
        res.res = "going home";
        ros::spinOnce();
    }else if(cmd == "grav_comp"){
        current_policy = NONE;
        ctrl_type      = JOINT_POSITION;
        grav_msg.data  = !grav_msg.data;
        sendGrav(grav_msg);
    }else{
        res.res = "no such cmd [" + cmd + "]!";
        return false;
    }

    return true;
}






}
