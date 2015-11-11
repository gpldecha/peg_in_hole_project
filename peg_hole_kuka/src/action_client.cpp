#include "peg_hole_kuka/action_client.h"
#include "exploration_planner/simple_exploration.h"



PEG_action_client::PEG_action_client(){

}

void PEG_action_client::initialise(){
    init_cart();
    init_simple_bel_planner();
    // init_joint();
}

void PEG_action_client::get_link_socket_goal(tf::Transform& transform){
    tf::StampedTransform stransform;
    opti_rviz::Listener::get_tf_once("/world_frame","/link_socket",stransform,10);
    transform = stransform;
}


void PEG_action_client::init_simple_bel_planner(){

    geometry_msgs::Transform        link_socket;
    tf::Transform                   tf_link_socket;
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

    {
        ac::Goal                    goal;
        goal.action_name            = "simple_bel_planner";
        goal.attractor_frame        = link_socket;

        goals["simple_bel_planner"] = goal;
    }
    {
        ac::Goal                    goal;
        goal.action_name            = "simple_bel_planner";
        goal.attractor_frame        = link_socket;
        goals["gmm_bel_planner"]    = goal;
    }

    enum{KUKA_DOF = 7};
    std::array<double,KUKA_DOF> des_position;
    kuka_fri_bridge::JointStateImpedance jointStateImpedance;
    jointStateImpedance.position.resize(KUKA_DOF);
    jointStateImpedance.velocity.resize(KUKA_DOF);
    jointStateImpedance.effort.resize(KUKA_DOF);
    jointStateImpedance.stiffness.resize(KUKA_DOF);
    // Go to a target joint configuration (INIT)
    {
        ac::Goal goal;
        des_position  =  {{-0.2968,0.1748,0.1107,-2.01099,-0.2413,-0.59104,0.14}};

        for(std::size_t i = 0; i < KUKA_DOF;i++)
            jointStateImpedance.position[i]      = des_position[i];

        goal.action_name            = "goto_joint";
        goal.JointStateImpedance    = jointStateImpedance;
        goals["go_front"]           = goal;
    }

    {
        ac::Goal goal;
        des_position  =  {{0.803,0.4995,0.0286,-1.986,0.9915,-1.1997,-0.5516}};

        for(std::size_t i = 0; i < KUKA_DOF;i++)
            jointStateImpedance.position[i]      = des_position[i];

        goal.action_name            = "goto_joint";
        goal.JointStateImpedance    = jointStateImpedance;
        goals["go_left"]            = goal;
    }

    std::array<double,KUKA_DOF> des_velocity;
    std::array<double,KUKA_DOF> des_stiffness;

    // Go Back to Joint Impedance Mode
    {
        ac::Goal goal;
        des_velocity  =  {{0,0,0,0,0,0,0}};
        des_stiffness =  {{100,100,100,100,100,100,100}};

        for(std::size_t i = 0; i < 7;i++){
            jointStateImpedance.velocity[i]      = des_velocity[i];
            jointStateImpedance.stiffness[i]     = des_stiffness[i];
        }

        goal.action_name            = "grav_comp";
        goal.action_type            = "velocity";
        goal.JointStateImpedance    = jointStateImpedance;
        goals["joint_imp_s"]        = goal;
    }



}

void PEG_action_client::init_cart(){

    {
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
        goals["plug"]               = goal;

        ROS_INFO("plug no uncertainty initialised");
    }

    {
        ac::Goal                 goal;
        geometry_msgs::Transform  link_socket;
        tf::Transform             tf_link_socket;
        get_link_socket_goal(tf_link_socket);


        tf::Quaternion            rotation;
        rotation.setRPY(0,0,M_PI);
        tf_link_socket.setRotation( tf_link_socket.getRotation() * rotation );

        link_socket.translation.x   = -0.6;
        link_socket.translation.y   = 0;
        link_socket.translation.z   = 0.4;

        link_socket.rotation.w      = tf_link_socket.getRotation().w();
        link_socket.rotation.x      = tf_link_socket.getRotation().x();
        link_socket.rotation.y      = tf_link_socket.getRotation().y();
        link_socket.rotation.z      = tf_link_socket.getRotation().z();

        goal.action_name            = "goto_cart";
        goal.attractor_frame        = link_socket;
        goal.action_type            = "closed_loop";
        goals["home"]               = goal;

        ROS_INFO("plug no uncertainty initialised");
    }


}


void PEG_action_client::init_joint(){

    {
        ac::Goal goal;
        goal.JointState.position = {{0,0,0,0,0,0,0}};
        goal.action_name = "goto_joint";
        goals["home"] = goal;
    }

}
