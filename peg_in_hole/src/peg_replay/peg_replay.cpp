#include <peg_in_hole/peg_replay.h>
#include <algorithm>
#include <optitrack_rviz/type_conversion.h>
#include <optitrack_rviz/input.h>
#include <optitrack_rviz/print.h>
#include <optitrack_rviz/listener.h>





Peg_replay::Peg_replay(ros::NodeHandle& node,
                       const std::string& plug_topic,
                       const std::string& socket_topic):
broadcaster_plug_link("world",plug_topic),broadcaster_socket("world",socket_topic),
index_subscriber(node,"index_sub")
{

    bPlayTraj       = false;
    bInit           = false;
    bListen_index   = false;
    service         = node.advertiseService("peg_replay_cmd", &Peg_replay::cmd_callback,this);
    ptr_node        = &node;

    cmds["load"]    = LOAD;
    cmds["start"]   = START;
    cmds["stop"]    = STOP;
    cmds["init"]    = INIT;
    cmds["listen"]  = LIST_INDEX;

    position_socket.setValue(0,0,0);
    plug_link_pos.setValue(0.03,-0.04,0);
    plug_link_rot.setRPY(0,0,M_PI);

    run_trajectories.set_initial_position(plug_link_pos,plug_link_rot);

    t      = 0;
    t_stop = 0;
}

void Peg_replay::set_traj_dir_path(const std::string path2dir){
    this->path2dir  = path2dir;
}

void Peg_replay::update(){

    run_trajectories.update_sensor();


    if(bPlayTraj){
        if(bListen_index){
            t = index_subscriber.index;
        }

        if(t < t_stop ){
            run_trajectories.set_pos_origin_force(t);

            if(ptr_plug_contact_model!=NULL){
                ptr_plug_contact_model->update(run_trajectories.position,run_trajectories.r_rot);
            }

            run_trajectories.update_particle_filter();
            t = t + 1;
        }
        opti_rviz::type_conv::vec2tf(run_trajectories.position,plug_link_pos);
        opti_rviz::type_conv::mat2tf(run_trajectories.r_rot,plug_link_rot);
    }


    broadcaster_plug_link.update(plug_link_pos,plug_link_rot);
    broadcaster_socket.update(position_socket,orientation_socket);
}

void Peg_replay::use_contact_model(wobj::WrapObject& wrapped_objects){
    ptr_plug_contact_model = std::unique_ptr<psm::Plug_contact_model>(new psm::Plug_contact_model(wrapped_objects));
}


bool Peg_replay::cmd_callback(peg_in_hole::String_cmd::Request& req,peg_in_hole::String_cmd::Response& res){

     std::vector<std::string> input;
     opti_rviz::Input::segment_string_space(req.cmd,input);
     if(input.size() == 0){
         res.res = "Peg_replay::cmd_callback no command given";
         return false;
     }

     std::string first = input[0];


     bool status = true;
     if(cmds.count(first) > 0){

         switch (cmds[first]) {
         case LOAD:
         {
             if(input.size() != 2){
                 res.res = "Peg_replay::cmd_callback cmd: load, however no file path given";
                 return false;
             }
             std::string file_name  = input[1];
             if(!run_trajectories.load(path2dir,file_name)){
                 std::string error_msg = "Peg_replay::cmd_callback   no such file to load: " + path2dir + file_name;
                 t      = 0;
                 t_stop = 0;

                 res.res    = error_msg;
                 std::cout<< error_msg << std::endl;
                 status = false;
             }else{
                 t      = 0;
                 t_stop = run_trajectories.traj.n_rows-1;
                 std::cout << "successfully loaded traj:" << run_trajectories.traj.n_rows << " x " << run_trajectories.traj.n_cols << std::endl;
                 std::cout<< "index_stop: " << t_stop << std::endl;
                 res.res = "sucessfully loaded: " + file_name;
             }
             break;
         }
         case START:
         {
             res.res   = "start";
             bPlayTraj = true;
             break;
         }
         case INIT:
         {
             std::cout<< "INIT start" << std::endl;
             t      = 0;
             t_stop = run_trajectories.traj.n_rows-1;
             run_trajectories.set_pos_origin_force(t);
             if(run_trajectories.ptr_particle_filter_manager != NULL){
                 run_trajectories.ptr_particle_filter_manager->initialise_prior_pdf(run_trajectories.position);
                 std::cout<< "-1-" << std::endl;

                 run_trajectories.update_sensor();
                 std::cout<< "-2-" << std::endl;
                 run_trajectories.set_pos_origin_force(t);
                 std::cout<< "-2.5-" << std::endl;
                 run_trajectories.update_particle_filter();
                 std::cout<< "-3-" << std::endl;

                 opti_rviz::type_conv::vec2tf(run_trajectories.position,plug_link_pos);
                 opti_rviz::type_conv::mat2tf(run_trajectories.r_rot,plug_link_rot);
                 bInit   = !bInit;
                 std::cout<< "INIT finished" << std::endl;
             }

             if(bInit){
                 res.res = "init: true";
             }else{
                 res.res = "init: false";
             }
             break;
         }
         case STOP:
         {
             res.res = "stop";
             bPlayTraj = false;
             break;
         }
         case LIST_INDEX:
         {

             bListen_index  = !bListen_index;
             bPlayTraj      = !bPlayTraj;

             if(bListen_index){
                 res.res = "listening to index";
             }else{
                 res.res = "not listening to index";
             }
             break;
         }
         default:
             break;
         }

     }else{
         std::string error_msg = "Peg_replay::cmd_callback   no such command: " + first;
         res.res = error_msg;
         std::cout << error_msg.c_str() << std::endl;
          status = false;
     }
    return status;
}

void Peg_replay::initalise_vision(ros::NodeHandle& node){
    if(run_trajectories.ptr_particle_filter_manager != NULL){
        run_trajectories.ptr_particle_filter_manager->init_visualise(node);
        run_trajectories.sensor_manager->init_visualise(node);
    }
}

void Peg_replay::visualise(){
    if(run_trajectories.ptr_particle_filter_manager != NULL){
        run_trajectories.ptr_particle_filter_manager->visualise();
        run_trajectories.sensor_manager->visualise();
    }
}

/*
void Peg_replay::get_transform(){
    while(!bGotTransform){
        try{

            tf_listener.lookupTransform("plug_link","link_cylinder",ros::Time(0), tf_transform);
            T =  tf_transform.getOrigin();
            R.setRotation(q);
            bGotTransform=true;
        }
        catch (tf::TransformException ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1).sleep();
        }
    }
}
*/
