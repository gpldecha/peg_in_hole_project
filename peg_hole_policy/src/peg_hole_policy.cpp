#include "peg_hole_policy/peg_hole_policy.h"
#include <chrono>
namespace ph_policy{

Peg_hole_policy::Peg_hole_policy(ros::NodeHandle& nh,
                                 const std::string& path_sensor_model,
                                 const std::string& fixed_frame):
    Base_ee_action(nh),
    state_machine(nh,"peg_sensor_classifer"),
    find_table(nh,path_sensor_model,fixed_frame),
    find_socket(nh,path_sensor_model,fixed_frame),
    insert_peg(nh,path_sensor_model,fixed_frame),
    Base_action_server(nh)
{

    server_srv              = nh.advertiseService("cmd_peg_policy",&Peg_hole_policy::cmd_callback,this);

    world_frame             = fixed_frame;

    initial_config          = true;
    tf_count                = 0;
    control_rate            = 100.0;
    current_policy          = NONE;
}

bool Peg_hole_policy::execute_CB(asrv::alib_server& as_,asrv::alib_feedback& feedback,const asrv::cptrGoal& goal){

   if(!activate_controller("one_task_inverse_kinematics")){
       return false;
   }
   current_policy           = NONE;
   double min_speed         = 0.001;
   double max_speed         = goal->max_speed;

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

    ROS_INFO("reachingThreashold:    %f [m]",reachingThreshold);
    ROS_INFO("orientationThreshold:  %f [rad]",orientationThreshold);
    ROS_INFO("max_speed              %f",max_speed);
    ROS_INFO("min_speed              %f",min_speed);
    ROS_INFO("rate                   %f",control_rate);

    tf::Quaternion qdiff;

    tf::Vector3     velocity_ms;
    tf::Vector3     current_origin;
    tf::Quaternion  current_orient;

    ros::Rate loop_rate(control_rate);
    bool success = true;
    while(ros::ok()) {

        current_origin = ee_pose_current.getOrigin();
        current_orient = ee_pose_current.getRotation();

        switch(current_policy){
        case FIND_TABLE:
        {
            find_table.get_linear_velocity(velocity_ms,current_origin);
            break;
        }
        case FIND_SOCKET:
        {
            find_socket.get_linear_velocity(velocity_ms,current_origin);
            break;
        }
        case FIND_HOLE:
        {
            insert_peg.get_linear_velocity(velocity_ms,current_origin);
            break;
        }
        default:
        {
            des_ee_pose.setOrigin(current_origin);
            des_ee_pose.setRotation(current_orient);
            velocity_ms.setZero();
            break;
        }
        }



        des_ee_vel_msg.linear.x = velocity_ms[0];
        des_ee_vel_msg.linear.y = velocity_ms[1];
        des_ee_vel_msg.linear.z = velocity_ms[2];

        // Computing angular velocity from quaternion differentiation
        qdiff =  des_ee_pose.getRotation() - current_orient;
        Eigen::Quaternion<double>  dq (qdiff.getW(),qdiff.getX(),qdiff.getY(),qdiff.getZ());
        Eigen::Quaternion<double>   q(current_orient.getW(),current_orient.getX(),current_orient.getY(), current_orient.getZ());

        Eigen::Vector3d w = motion::d2qw<double>(q,dq);

       // ROS_INFO_STREAM_THROTTLE(1.0,"q: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w());


        des_ee_vel_msg.angular.x = w[0];
        des_ee_vel_msg.angular.y = w[1];
        des_ee_vel_msg.angular.z = w[2];

        sendVel(des_ee_vel_msg);

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
    }

    current_policy = NONE;
    des_ee_vel_msg.linear.x  = 0;
    des_ee_vel_msg.linear.y  = 0;
    des_ee_vel_msg.linear.z  = 0;
    des_ee_vel_msg.angular.x = 0;
    des_ee_vel_msg.angular.y = 0;
    des_ee_vel_msg.angular.z = 0;
    sendVel(des_ee_vel_msg);

    return success;
}


bool Peg_hole_policy::cmd_callback(peg_hole_policy::String_cmd::Request& req, peg_hole_policy::String_cmd::Response& res){

    std::string cmd = req.cmd;
    if(cmd == "go_table"){
        current_policy = FIND_TABLE;
        res.res = "going to table!";
    }else if(cmd == "go_socket"){
        current_policy = FIND_SOCKET;
        res.res = "going to socket!";
    }else if(cmd == "insert"){
        current_policy = FIND_HOLE;
        res.res = "going to connect peg and socket!";
    }else if(cmd == "pause"){
        current_policy = NONE;
        res.res = "pause!";
    }else{
        res.res = "no such cmd [" + cmd + "]!";
        return false;
    }

    return true;
}






}
