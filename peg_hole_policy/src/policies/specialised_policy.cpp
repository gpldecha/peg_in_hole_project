#include "peg_hole_policy/policies/specialised_policy.h"
#include "optitrack_rviz/type_conversion.h"

namespace ph_policy{

Specialised::Specialised(wobj::WrapObject &wrap_object):
    go_edge(wrap_object),
    go_socket(wrap_object)
{
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
}

void Specialised::reset(){
    go_edge.set_target(go_edge.w_ml);
    go_socket.set_target(go_socket.s_ml);


    planning_stack.set_action(actions::GO_TO_SOCKET);
    socket_policy = SOCKET_POLICY::RIGHT_OUTER;
}

void Specialised::set_socket_policy(SOCKET_POLICY socket_policy){
    this->socket_policy = socket_policy;
}

std::string Specialised::command(const std::string& cmd){
    std::string res;
    if(cmd == "reset"){
          reset();
    }else{
        res = "no such cmd: " + cmd + " [Specialised]";
    }
    return res;
}

void Specialised::update(arma::colvec3&                 arma_velocity,
                         const arma::colvec3&           mls_WF,
                         const arma::colvec3&           mls_SF,
                         const arma::colvec3&           socket_pos_WF,
                         const arma::colvec3&           peg_origin,
                         const arma::colvec&            Y_c,
                         const std::vector<STATES>&     states,
                         Insert_peg&                    insert_peg,
                         Get_back_on&                   get_back_on,
                         Force_control&                 force_control,
                         Peg_sensor_model&              peg_sensor_model)
{

    planning_stack.update(states);
    ctrl_policy = planning_stack.get_action();
    planning_stack.print(1);

    switch(ctrl_policy)
    {
    case FIND_TABLE:
    {
        ROS_INFO_STREAM_THROTTLE(1.0,"-> FIND_TABLE");
        arma_velocity.zeros();
        arma_velocity(0) = -1;
        ROS_INFO_STREAM_THROTTLE(1.0,"arma_velocity: " << arma_velocity(0) << " " << arma_velocity(1) << " " << arma_velocity(2));
        break;
    }
    case GO_TO_EDGE:
    {
        go_edge.update(arma_velocity,mls_WF);

        if(Planning_stack::has_state(STATES::AIR,states) &&
           Planning_stack::has_state(STATES::MEDIUM_UNCERTAINTY,states) &&
            std::fabs(peg_origin(1)) > 0.49)
            {
                arma_velocity.zeros();
                arma_velocity(0) = -1;
                ROS_INFO_STREAM_THROTTLE(1.0,"-> OFF table go forward");
            }

        if(Planning_stack::has_state(STATES::STUCK_EDGE,states)){
            ROS_INFO_STREAM_THROTTLE(1.0, "-> STUCK at an edge");
            force_control.get_over_edge(arma_velocity);
        }else{
            force_control.update_x(arma_velocity);
        }

        break;
    }
    case GO_TO_SOCKET:
    {
        insert_peg.get_linear_velocity(arma_velocity,mls_WF);
        if(Planning_stack::has_state(STATES::STUCK_EDGE,states)){
            ROS_INFO_STREAM_THROTTLE(1.0, "-> STUCK at an edge");
            force_control.get_over_edge(arma_velocity);
        }else{
            force_control.update_x(arma_velocity);
        }

        break;
    }
    case GET_BACK_ONTO_TABLE:
    {
        get_back_on.update(arma_velocity,mls_SF,mls_WF,Y_c);
        if(Planning_stack::has_state(STATES::STUCK_EDGE,states)){
            ROS_INFO_STREAM_THROTTLE(1.0, "-> STUCK at an edge");
            force_control.get_over_edge(arma_velocity);
        }
        break;
    }
    case FIND_SOCKET_HOLE:
    {

        if(socket_policy == SOCKET_POLICY::LEFT_OUTER){
            left_circle_gmr.gmr(mls_SF);
            left_circle_gmr.get_ee_linear_velocity(arma_velocity);
            if(Planning_stack::has_state(STATES::TOP_SOCKET,states)){
                socket_policy=SOCKET_POLICY::TO_SOCKET;
                ROS_INFO_STREAM("----------> SWITCH GO TO SOCKET HOLE <-------------");
            }
        }else if(socket_policy == SOCKET_POLICY::RIGHT_OUTER){
            right_circle_gmr.gmr(mls_SF);
            right_circle_gmr.get_ee_linear_velocity(arma_velocity);
            if(Planning_stack::has_state(STATES::TOP_SOCKET,states)){
                socket_policy=SOCKET_POLICY::TO_SOCKET;
                ROS_INFO_STREAM("----------> SWITCH GO TO SOCKET HOLE <-------------");
            }
        }else if(socket_policy == SOCKET_POLICY::TO_SOCKET){
            ROS_INFO_STREAM_THROTTLE(1.0,"      FINAL ACTION");
            gmm_socket.gmr(mls_SF);
            gmm_socket.get_ee_linear_velocity(arma_velocity);

            if(Planning_stack::has_state(STATES::SOCKET_ENTRY,states)){
                ROS_INFO_STREAM("----------> SWITCH GO TO INSERT <-------------");
                socket_policy=SOCKET_POLICY::GO_TO_INSERT;
            }
        }else if(socket_policy == SOCKET_POLICY::GO_TO_INSERT){
           // gmm_socket.gmr(mode_pos_SF);
          //  gmm_socket.get_ee_linear_velocity(arma_velocity);

            arma_velocity = socket_pos_WF  - mls_WF;
            arma_velocity = arma::normalise(arma_velocity);

            if(Planning_stack::has_state(STATES::SLIGHTLY_IN,states)){
                ROS_INFO_STREAM("----------> [ [  INSERT ] ]<-------------");
                socket_policy=SOCKET_POLICY::INSERT;
            }

            ROS_INFO_STREAM_THROTTLE(1.0,"policy:   [GO_TO_INSERT]");
        }else if(socket_policy == SOCKET_POLICY::INSERT){
            arma_velocity(0) = -1;
            arma_velocity(1) =  0;
            arma_velocity(2) =  0;
            ROS_INFO_STREAM_THROTTLE(1.0,"policy:   [INSERT]");
        }

        break;
    }
    default:
    {
        break;
    }
    }


   arma_velocity = arma::normalise(arma_velocity);

}



}