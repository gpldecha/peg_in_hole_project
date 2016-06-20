#include "peg_hole_policy/policies/specialised_policy.h"
#include "optitrack_rviz/type_conversion.h"

namespace ph_policy{

Specialised::Specialised(wobj::WrapObject &wrap_object, SOCKET_TYPE socket_type):
    go_edge(wrap_object),
    go_socket(wrap_object),
    socket_type(socket_type)
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

    b_air = false;
}

void Specialised::reset(){
    go_edge.set_target(go_edge.w_ml);
    go_socket.set_target(go_socket.s_ml);


    planning_stack.set_action(actions::FIND_SOCKET_HOLE);
    socket_policy = SOCKET_POLICY::LEFT_OUTER;
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

  /*  switch(ctrl_policy)
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

        if(Planning_stack::has_state(STATES::AIR,states) && Planning_stack::has_state(STATES::MEDIUM_UNCERTAINTY,states) &&  std::fabs(peg_origin(1)) > 0.49)
            {
                arma_velocity.zeros();
                arma_velocity(0) = -1;
                ROS_INFO_STREAM_THROTTLE(1.0,"-> OFF table go forward");
            }
        break;
    }
    case GO_TO_SOCKET:
    {
        insert_peg.update(arma_velocity,mls_SF);
        break;
    }
    case GET_BACK_ONTO_TABLE:
    {
        get_back_on.update(arma_velocity,mls_SF,mls_WF,Y_c);
        break;
    }
    case FIND_SOCKET_HOLE:
    {
*/
        mls_SF_tmp = mls_SF;

      //  if(socket_type == SOCKET_TYPE::TWO){
            mls_SF_tmp(0) = 0.02;
      //  }

        if(socket_policy == SOCKET_POLICY::LEFT_OUTER){
            ROS_INFO_STREAM_THROTTLE(1.0,"SOCKET_POLICY == > LEFT_OUTER");
            left_circle_gmr.gmr(mls_SF_tmp);
            left_circle_gmr.get_ee_linear_velocity(arma_velocity);
           // if(Planning_stack::has_state(STATES::TOP_SOCKET,states)){
           //     socket_policy=SOCKET_POLICY::TO_SOCKET;
           //     ROS_INFO_STREAM("----------> SWITCH GO TO SOCKET HOLE <-------------");
          //  }
        }else if(socket_policy == SOCKET_POLICY::RIGHT_OUTER){
          ROS_INFO_STREAM_THROTTLE(1.0,"SOCKET_POLICY == > RIGHT_OUTER");
            right_circle_gmr.gmr(mls_SF_tmp);
            right_circle_gmr.get_ee_linear_velocity(arma_velocity);
          //  if(Planning_stack::has_state(STATES::TOP_SOCKET,states)){
          //      socket_policy=SOCKET_POLICY::TO_SOCKET;
          //      ROS_INFO_STREAM("----------> SWITCH GO TO SOCKET HOLE <-------------");
          //  }
        }/*else if(socket_policy == SOCKET_POLICY::TO_SOCKET){
            ROS_INFO_STREAM_THROTTLE(1.0,"SOCKET_POLICY ==>    FINAL ACTION");
            gmm_socket.gmr(mls_SF_tmp);
            gmm_socket.get_ee_linear_velocity(arma_velocity);

            if(Planning_stack::has_state(STATES::SOCKET_ENTRY,states)){
                ROS_INFO_STREAM("----------> SWITCH GO TO INSERT <-------------");
                socket_policy=SOCKET_POLICY::GO_TO_INSERT;
            }*/
        /*}else if(socket_policy == SOCKET_POLICY::GO_TO_INSERT){
           gmm_socket.gmr(mode_pos_SF);
           gmm_socket.get_ee_linear_velocity(arma_velocity);

           arma_velocity = socket_pos_WF  - peg_origin;
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
        }*/
  /*      break;
    }
    default:
    {
        break;
    }
    }*/


    ///  keep peg whithin socket region
    //  ROS_INFO_STREAM_THROTTLE(1.0,"socket_box.dist_edge:   " << peg_sensor_model.get_distance_edge() );

//        /

        opti_rviz::debug::tf_debuf(socket_pos_WF,"socket_pos_WF");
    if(dist_yz(socket_pos_WF,peg_origin) > 0.058){
        ROS_WARN_STREAM_THROTTLE(1.0,"TO FARE AWAY");
        arma_velocity = -mls_SF;
        arma_velocity(0) = 0;
    }

/*
    if(State_machine::has_state(STATES::AIR_HIGH,states))
    {
        b_air = true;
    }

    if(!State_machine::has_state(STATES::AIR,states)){
        b_air = false;
    }

    if(b_air){
        arma_velocity.zeros();
        arma_velocity(0) = -1;
    }*/


   arma_velocity = arma::normalise(arma_velocity);

}



}
