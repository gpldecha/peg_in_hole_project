#include "peg_hole_policy/policies/search_policy.h"
#include "optitrack_rviz/type_conversion.h"
#include <optitrack_rviz/debug.h>

namespace ph_policy{


Planning_stack::Planning_stack(){
    reset();
}

void Planning_stack::reset(){
    stack = {{FIND_TABLE,GO_TO_EDGE,GO_TO_SOCKET,FIND_SOCKET_HOLE,INSERT}};
    index = 0;
    current_action = stack[index];
    bFrist         = true;
}

void Planning_stack::set_action(actions action){
    for(std::size_t i = 0; i < stack.size();i++){
        if(stack[i] == action){
            index = i;
            std::cout<< "found action: " << actions2str(action) << std::endl;
            return;
        }
    }

    std::cout<< "did not find action: " << actions2str(action) << std::endl;
}

void Planning_stack::update(const std::vector<STATES>& states){

    current_action = stack[index];

    if(current_action == FIND_TABLE){
        if(has_state(STATES::TABLE,states)){
            next_action();
        }
    }else if(current_action == GO_TO_EDGE){
        if(has_state(STATES::CLOSE_EDGE,states)){
            next_action();
        }
    }else if(current_action == GO_TO_SOCKET){
        if(has_state(STATES::SOCKET,states) || has_state(STATES::MODE_CLOSE_HOLE,states)){
            next_action();
        }
    }else if(current_action == FIND_SOCKET_HOLE){
        if(has_state(STATES::SOCKET,states)){
            //   next_action();
        }
    }else if(current_action == INSERT){

    }

    if(has_state(STATES::OFF_TABLE,states)){
        current_action = GET_BACK_ONTO_TABLE;
    }
}

bool Planning_stack::has_state(const STATES state,const std::vector<STATES>& states){
    return std::find(states.begin(),states.end(),state) != states.end();
}

const void Planning_stack::print(double seconds) const{
    if(seconds==0){
        ROS_INFO_STREAM("action: " << actions2str(get_action()));
    }else{
        ROS_INFO_STREAM_THROTTLE(seconds,"action: " << actions2str(get_action()));
    }
}

actions Planning_stack::get_action() const{
    return current_action;
}

void Planning_stack::next_action(){
    index++;
    if(index >= stack.size()){
        ROS_WARN_THROTTLE(1.0,"planning stack reached end!");
        index=stack.size()-1;
    }
    ROS_INFO_STREAM("next action : [" << actions2str(get_action()) << "]");
}

Search_policy::Search_policy(ros::NodeHandle& nh,
                             belief::Gmm_planner& gmm_belief,
                             State_machine& state_machine,
                             wobj::WrapObject& wrap_object,
                             Peg_sensor_model &peg_sensor_model):
    gmm_belief(gmm_belief),
    state_machine(state_machine),
    go_edge(wrap_object),
    force_control(nh),
    go_socket(wrap_object),
    get_back_on(wrap_object.get_wbox("link_wall")),
    socket_box(wrap_object.get_wbox("wbox_socket")),
    peg_sensor_model(peg_sensor_model),
    peg_ee_listener("world","lwr_peg_link")
{
    q_tmp.setRPY(0,-M_PI/2,0);
    reset();
}


void Search_policy::update_force_vis(const arma::colvec3& force,const tf::Vector3& position, const tf::Quaternion& orientation){
    force_control.update(force,position,orientation);
}

void Search_policy::reset(){

    planning_stack.reset();
    get_back_on.reset();
    go_edge.set_target(go_edge.w_ml);
    go_socket.set_target(go_socket.s_ml);
    arma_velocity.zeros();


    {
        tf::StampedTransform transform;
        opti_rviz::Listener::get_tf_once("world","link_socket",transform);
        opti_rviz::type_conv::tf2vec(transform.getOrigin(),socket_pos_WF);
    }


    {
        std::string path_gmr_right_circle = "/home/guillaume/MatlabWorkSpace/peg_in_hole_RL/gmr_policies/model/cpp/gmm_right_circle";
        right_circle_gmr = planners::GMR_EE_Planner(path_gmr_right_circle);
    }
    {
        std::string path_left_circle = "/home/guillaume/MatlabWorkSpace/peg_in_hole_RL/gmr_policies/model/cpp/gmm_left_circle";
        left_circle_gmr = planners::GMR_EE_Planner(path_left_circle);
    }
    {
        std::string path_combo_gmm = "/home/guillaume/MatlabWorkSpace/peg_in_hole_RL/gmr_policies/model/cpp/gmm_xsocket";
        gmm_socket = planners::GMR_EE_Planner(path_combo_gmm);
    }

 //   planning_stack.set_action(actions::GO_TO_SOCKET);
 //   socket_policy = SOCKET_POLICY::RIGHT_OUTER;
}

void Search_policy::set_action(actions ctrl_policy){
    planning_stack.set_action(ctrl_policy);
}

void Search_policy::set_socket_policy(SOCKET_POLICY socket_policy){
    this->socket_policy = socket_policy;
}

void Search_policy::get_velocity(tf::Vector3&    velocity,
                                 tf::Quaternion& des_orient,
                                 const tf::Vector3 &peg_origin,
                                 const tf::Matrix3x3 &peg_orient,
                                 const arma::colvec& Y_c,
                                 geometry_msgs::Wrench &wrench)
{

    mode_pos_WF(0) = gmm_belief.belief_state(0);
    mode_pos_WF(1) = gmm_belief.belief_state(1);
    mode_pos_WF(2) = gmm_belief.belief_state(2);

    mode_pos_SF(0) = gmm_belief.belief_state_SF(0);
    mode_pos_SF(1) = gmm_belief.belief_state_SF(1);
    mode_pos_SF(2) = gmm_belief.belief_state_SF(2);

    state_machine.update(arma_velocity,force,gmm_belief.belief_state,gmm_belief.belief_state_SF,peg_orient,Y_c,get_back_on);
    state_machine.print(1);

    planning_stack.update(state_machine.get_state());
    ctrl_policy = planning_stack.get_action();
    planning_stack.print(1);

    force(0) = wrench.force.x;
    force(1) = wrench.force.y;
    force(2) = wrench.force.z;

    des_orient.setRPY(0,0,0);
    des_orient = q_tmp * des_orient;


    tf::Vector3 tmp_pos;
    opti_rviz::type_conv::vec2tf(mode_pos_WF,tmp_pos);
    peg_sensor_model.update_model(tmp_pos,peg_orient);
    peg_sensor_model.get_distance_features();

    opti_rviz::debug::tf_debuf<float>(peg_sensor_model.get_closet_point(contact_type::EDGE),"edge_projection");
    opti_rviz::debug::tf_debuf(mode_pos_WF,"mode_pos_WF");
    arma_velocity.zeros();
    force_control.get_over_edge(arma_velocity);
    opti_rviz::type_conv::tf2vec(peg_origin,arma_peg_origin);


    switch(ctrl_policy)
    {
    case FIND_TABLE:
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"-> FIND_TABLE");
        gmm_belief.get_linear_velocity(arma_velocity);
        ROS_INFO_STREAM_THROTTLE(1.0,"arma_velocity: " << arma_velocity(0) << " " << arma_velocity(1) << " " << arma_velocity(2));

        break;
    }
    case GO_TO_EDGE:
    {
        go_edge.update(arma_velocity,mode_pos_WF);

        if(Planning_stack::has_state(STATES::AIR,state_machine.get_state()) &&
           Planning_stack::has_state(STATES::MEDIUM_UNCERTAINTY,state_machine.get_state()) &&
            std::fabs(arma_peg_origin(1)) > 0.49)
            {
                arma_velocity.zeros();
                arma_velocity(0) = -1;
                ROS_INFO_STREAM_THROTTLE(1.0,"-> OFF table go forward");
            }

        if(Planning_stack::has_state(STATES::STUCK_EDGE,state_machine.get_state())){
            ROS_INFO_STREAM_THROTTLE(1.0, "-> STUCK at an edge");
            force_control.get_over_edge(arma_velocity);
        }else{
            force_control.update_x(arma_velocity);
        }

        break;
    }
    case GO_TO_SOCKET:
    {
        insert_peg.get_linear_velocity(arma_velocity,mode_pos_WF);
        if(Planning_stack::has_state(STATES::STUCK_EDGE,state_machine.get_state())){
            ROS_INFO_STREAM_THROTTLE(1.0, "-> STUCK at an edge");
            force_control.get_over_edge(arma_velocity);
        }else{
            force_control.update_x(arma_velocity);
        }

        break;
    }
    case GET_BACK_ONTO_TABLE:
    {
        get_back_on.update(arma_velocity,mode_pos_SF,mode_pos_WF,Y_c);
        opti_rviz::type_conv::vec2tf(arma_velocity,velocity);
        if(Planning_stack::has_state(STATES::STUCK_EDGE,state_machine.get_state())){
            ROS_INFO_STREAM_THROTTLE(1.0, "-> STUCK at an edge");
            force_control.get_over_edge(arma_velocity);
        }
        break;
    }
    case FIND_SOCKET_HOLE:
    {

        if(socket_policy == SOCKET_POLICY::LEFT_OUTER){
            left_circle_gmr.gmr(mode_pos_SF);
            left_circle_gmr.get_ee_linear_velocity(arma_velocity);
            if(Planning_stack::has_state(STATES::TOP_SOCKET,state_machine.get_state())){
                socket_policy=SOCKET_POLICY::TO_SOCKET;
                ROS_INFO_STREAM("----------> SWITCH GO TO SOCKET HOLE <-------------");
            }
        }else if(socket_policy == SOCKET_POLICY::RIGHT_OUTER){
            right_circle_gmr.gmr(mode_pos_SF);
            right_circle_gmr.get_ee_linear_velocity(arma_velocity);
            if(Planning_stack::has_state(STATES::TOP_SOCKET,state_machine.get_state())){
                socket_policy=SOCKET_POLICY::TO_SOCKET;
                ROS_INFO_STREAM("----------> SWITCH GO TO SOCKET HOLE <-------------");
            }
        }else if(socket_policy == SOCKET_POLICY::TO_SOCKET){
            ROS_INFO_STREAM_THROTTLE(1.0,"      FINAL ACTION");
            gmm_socket.gmr(mode_pos_SF);
            gmm_socket.get_ee_linear_velocity(arma_velocity);

            if(Planning_stack::has_state(STATES::SOCKET_ENTRY,state_machine.get_state())){
                ROS_INFO_STREAM("----------> SWITCH GO TO INSERT <-------------");
                socket_policy=SOCKET_POLICY::GO_TO_INSERT;
            }
        }else if(socket_policy == SOCKET_POLICY::GO_TO_INSERT){
         //   gmm_socket.gmr(mode_pos_SF);
          //  gmm_socket.get_ee_linear_velocity(arma_velocity);

            arma_velocity = socket_pos_WF  - arma_peg_origin;
            arma_velocity = arma::normalise(arma_velocity);

            if(Planning_stack::has_state(STATES::SLIGHTLY_IN,state_machine.get_state())){
                ROS_INFO_STREAM("----------> [ [  INSERT ] ]<-------------");
                socket_policy=SOCKET_POLICY::INSERT;
            }

            ROS_INFO_STREAM_THROTTLE(1.0,"policy:   [INSERT]");
        }else if(socket_policy == SOCKET_POLICY::INSERT){
            arma_velocity(0) = -1;
            arma_velocity(1) =  0;
            arma_velocity(2) =  0;
            ROS_INFO_STREAM_THROTTLE(1.0,"policy:   [INSERT]");
        }

        arma_velocity = arma::normalise(arma_velocity);
        ROS_INFO_STREAM_THROTTLE(1.0,"socket_box.dist_edge:   " << peg_sensor_model.get_distance_edge() );

        if(peg_sensor_model.get_distance_edge() > 0.015){
            arma_velocity = (socket_pos_WF - mode_pos_WF);
            arma_velocity = arma::normalise(arma_velocity);
        }

        if(socket_policy != SOCKET_POLICY::INSERT){
            if(Planning_stack::has_state(STATES::STUCK_EDGE,state_machine.get_state())){
                force_control.get_over_edge(arma_velocity);
            }else{
                force_control.update_x(arma_velocity);
            }
        }

        break;
    }
    default:
    {
        break;
    }
    }

    force_control.force_safety(arma_velocity,10);

    arma_velocity = arma::normalise(arma_velocity);
    if(!arma_velocity.is_finite()){
        ROS_WARN_THROTTLE(1.0,"arma_velocity is not finite() [Search_policy::get_velocity]!");
        arma_velocity.zeros();
    }
    opti_rviz::type_conv::vec2tf(arma_velocity,velocity);

}

void Search_policy::get_target(FEATURE feature, arma::colvec3 &target){

    switch (feature) {
    case FEATURE::BL_CORNER:
    {
        break;
    }
    case FEATURE::ML_EDGE:
    {
        break;
    }
    case FEATURE::SOCKET:
    {
        break;
    }
    default:
        break;
    }


}


}
