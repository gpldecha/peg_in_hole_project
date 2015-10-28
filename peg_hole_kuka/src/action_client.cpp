#include "peg_hole_kuka/action_client.h"
#include "exploration_planner/simple_exploration.h"



PEG_action_client::PEG_action_client(){

}

void PEG_action_client::initialise(){
    init_cart();
   // init_simple_bel_planner();
   // init_two_joint_stiff();
   // init_two_grav_stiff();
}

void PEG_action_client::get_link_socket_goal(tf::Transform& transform){
    tf::StampedTransform stransform;
    opti_rviz::Listener::get_tf_once("/world_frame","/link_socket",stransform,10);
    transform = stransform;
}


void PEG_action_client::init_simple_bel_planner(){

    ac::Goal                        goal;
    geometry_msgs::Transform         plug_socket;

    // Listen for actual position of the socket

    tf::StampedTransform splug_socket;
    opti_rviz::Listener::get_tf_once("world_frame","link_socket",splug_socket,10);

    ROS_INFO("got socket frame of reference");

    tf::Vector3         origin               = splug_socket.getOrigin();
    tf::Quaternion      orient               = splug_socket.getRotation();


    plug_socket.translation.x        = origin.getX();
    plug_socket.translation.y        = origin.getY();
    plug_socket.translation.z        = origin.getZ();

    plug_socket.rotation.x           = orient.getX();
    plug_socket.rotation.y           = orient.getY();
    plug_socket.rotation.z           = orient.getZ();
    plug_socket.rotation.w           = orient.getW();

    goal.action_name                 = "simple_bel_planner";
    goal.attractor_frame             = plug_socket;

    goals["simple_bel_planner"]      = goal;
    ROS_INFO("home initialised");

}

void PEG_action_client::init_cart(){

    ac::Goal                 goal;
    geometry_msgs::Transform  link_socket;
    tf::Transform             tf_link_socket;
    get_link_socket_goal(tf_link_socket);


    tf::Quaternion            rotation;
    rotation.setRPY(0,0,M_PI);
    tf_link_socket.setRotation( tf_link_socket.getRotation() * rotation );

    link_socket.translation.x   = tf_link_socket.getOrigin().x();
    link_socket.translation.y   = tf_link_socket.getOrigin().y();
    link_socket.translation.z   = tf_link_socket.getOrigin().z();

    link_socket.rotation.w      = tf_link_socket.getRotation().w();
    link_socket.rotation.x      = tf_link_socket.getRotation().x();
    link_socket.rotation.y      = tf_link_socket.getRotation().y();
    link_socket.rotation.z      = tf_link_socket.getRotation().z();

    goal.action_name            = "goto_cart";
    goal.attractor_frame        = link_socket;
    goal.action_type            = "closed_loop";
    goals["plug"]               = goal;

    ROS_INFO("plug no uncertainty initialised");
}

void PEG_action_client::init_two_joint_stiff(){

    ac::Goal       goal;
    Eigen::VectorXd des_velocity;
    Eigen::VectorXd des_stiffness;
    des_velocity.resize(7);
    des_stiffness.resize(7);

    kuka_fri_bridge::JointStateImpedance jointStateImpedance;
    jointStateImpedance.position.resize(7);
    jointStateImpedance.velocity.resize(7);
    jointStateImpedance.effort.resize(7);
    jointStateImpedance.stiffness.resize(7);

    des_velocity  << 0,0,0,0,0,0,0;
    des_stiffness << 500,500,500,500,500,500,500;

    for(std::size_t i = 0; i < 7;i++){
        jointStateImpedance.velocity[i]      = des_velocity[i];
        jointStateImpedance.stiffness[i]     = des_stiffness[i];
    }

    goal.action_name           = "grav_comp";
    goal.action_type           = "velocity";
    goal.JointStateImpedance   = jointStateImpedance;
    goals["to_joint_stiff"]    = goal;
    ROS_INFO("to_joint_stiff initialised");
}

void PEG_action_client::init_two_grav_stiff(){

    ac::Goal goal;
    kuka_fri_bridge::JointStateImpedance jointStateImpedance;
    jointStateImpedance.position.resize(7);
    jointStateImpedance.velocity.resize(7);
    jointStateImpedance.effort.resize(7);
    jointStateImpedance.stiffness.resize(7);

    Eigen::VectorXd des_velocity;
    Eigen::VectorXd des_stiffness;
    des_velocity.resize(7);
    des_stiffness.resize(7);

    // Go to Gravity Compensation
    des_velocity  << 0,0,0,0,0,0,0;
    des_stiffness << 0,0,0,0,0,0,0;

    for(std::size_t i = 0; i < 7;i++){
        jointStateImpedance.velocity[i]      = des_velocity[i];
        jointStateImpedance.stiffness[i]     = des_stiffness[i];
    }

    goal.action_name          = "grav_comp";
    goal.action_type          = "velocity";
    goal.JointStateImpedance  = jointStateImpedance;
    goals["to_grav_comp"]     = goal;
    ROS_INFO("to_grav_com initialised");
}
