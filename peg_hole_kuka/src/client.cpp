#include "peg_hole_kuka/client.h"
#include "exploration_planner/simple_exploration.h"



PEG_action_client::PEG_action_client(){

}

void PEG_action_client::initialise(){
   // {
        /*ac::Goal                  goal;
        geometry_msgs::Transform  link_socket;
        tf::Transform             tf_link_socket;
        get_link_socket_goal(tf_link_socket);

        tf::Quaternion            rotation;
        rotation.setRPY(0,0,M_PI);
        tf_link_socket.setRotation( tf_link_socket.getRotation() * rotation );
      //  rotation.setRPY(M_PI/4,0,0);
      //  tf_link_socket.setRotation( rotation* tf_link_socket.getRotation() );


        link_socket.translation.x   = tf_link_socket.getOrigin().x();
        link_socket.translation.y   = tf_link_socket.getOrigin().y();
        link_socket.translation.z   = tf_link_socket.getOrigin().z();

        link_socket.rotation.w      = tf_link_socket.getRotation().w();
        link_socket.rotation.x      = tf_link_socket.getRotation().x();
        link_socket.rotation.y      = tf_link_socket.getRotation().y();
        link_socket.rotation.z      = tf_link_socket.getRotation().z();

        goal.action_type            = "plug_search";
        goal.target_frame           = link_socket;
        goal.max_speed              = 0.02;
        goal.min_speed              = 0.005;
        goals["plug_search"]        = goal;

    }

    {
        ac::Goal goal;
        goal.action_type        = "linear";
        goals["linear"]         = goal;

    }*/
   // init_cart();
   // init_simple_bel_planner();
}

void PEG_action_client::get_link_socket_goal(tf::Transform& transform){
    tf::StampedTransform stransform;
    opti_rviz::Listener::get_tf_once("/world","/link_socket",stransform);
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

/*    enum{KUKA_DOF = 7};
    std::array<double,KUKA_DOF> des_position;
    kuka_fri_bridge::JointStates jointStateImpedance;
    jointStateImpedance.position.resize(KUKA_DOF);
    jointStateImpedance.velocity.resize(KUKA_DOF);
    jointStateImpedance.effort.resize(KUKA_DOF);
    jointStateImpedance.stiffness.resize(KUKA_DOF);
    // Go to a target joint configuration (INIT)
    {
        ac::Goal goal;
        des_position  =  {{-1.02974,0.471239,0.401426,-1.76278,-1.0472,-0.802851,0.785398}};

        for(std::size_t i = 0; i < KUKA_DOF;i++)
            jointStateImpedance.position[i]      = des_position[i];

        goal.action_type            = "goto_joint";
        goal.JointStates            = jointStateImpedance;
        goals["go_front"]           = goal;
    }
    {
        ac::Goal goal;
        des_position  =  {{0.803,0.4995,0.0286,-1.986,0.9915,-1.1997,-0.5516}};

        for(std::size_t i = 0; i < KUKA_DOF;i++)
            jointStateImpedance.position[i]      = des_position[i];

        goal.action_type            = "goto_joint";
        goal.JointStates            = jointStateImpedance;
        goals["go_left"]            = goal;
    }
    {
        ac::Goal goal;
        des_position  =  {{0,0.785398,0.122173,-2.01099,-0.174533,0.261799,0}};

        for(std::size_t i = 0; i < KUKA_DOF;i++){
            jointStateImpedance.position[i]      = des_position[i];
        }

        goal.action_type            = "goto_joint";
        goal.JointStates            = jointStateImpedance;
        goals["home"]               = goal;
    }

    std::array<double,KUKA_DOF> des_velocity;
    std::array<double,KUKA_DOF> des_stiffness;
*/


}

void PEG_action_client::init_cart(){

  /*  enum{KUKA_DOF = 7};
    kuka_fri_bridge::JointStates jointStateImpedance;
    jointStateImpedance.name.resize(KUKA_DOF);
    jointStateImpedance.position.resize(KUKA_DOF);
    jointStateImpedance.velocity.resize(KUKA_DOF);
    jointStateImpedance.effort.resize(KUKA_DOF);
    jointStateImpedance.stiffness.resize(KUKA_DOF);*/


}


