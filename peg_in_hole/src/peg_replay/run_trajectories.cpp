#include <peg_in_hole/run_trajectories.h>



Run_trajectories::Run_trajectories(){
    b_use_filter            = false;
    b_use_sensor            = false;
    b_print_sensor          = false;
}
/*
void Run_trajectories::setup(const plugfilter::PF_parameters& pf_parameters,const Sensor_parameters& sensor_parameters){
    setup_pf(pf_parameters);
    setup_sensor(sensor_parameters);
    b_use_filter = true;

}
*/
void Run_trajectories::setup_pf(const plugfilter::PF_parameters& parameters){

    ptr_particle_filter_manager = ptr_pf_manager( new plugfilter::Plug_pf_manager(parameters) );
    ptr_particle_filter_manager->set_visualise_mode(parameters.visualisation_mode);
    std::cout<< "particle_filter_manager (created)" << std::endl;
    b_use_filter = true;

}

void Run_trajectories::setup_sensor(const Sensor_parameters& sensor_parameters){

    std::cout<< " Initialise real sensor" << std::endl;
    sensor_manager = ptr_sensor_manager(new psm::Sensor_manager(sensor_parameters.wrapped_objects));
    std::cout<< "sensor_parameters.t_sensor: " << sensor_parameters.t_sensor << std::endl;
    sensor_manager->initialise(sensor_parameters.t_sensor);
    b_use_sensor    = true;
    b_print_sensor  = sensor_parameters.b_print_sensor;

}




void Run_trajectories::set_initial_position(tf::Vector3& position, tf::Matrix3x3& rot){

    opti_rviz::type_conv::tf2vec(position,this->position);
    opti_rviz::type_conv::tf2mat(rot,this->r_rot);

}


bool Run_trajectories::load(const std::string& folder_path, const std::string& file_name){
    if(traj.load(folder_path + file_name)){
        error_diff = - traj(traj.n_rows-1,arma::span(0,2)).st();
        return true;
    }else{
        std::cerr<< "failed to load: " << folder_path + file_name << std::endl;
        return false;
    }
}

void Run_trajectories::run(){

    for(std::size_t i = 0; i < traj.n_rows;i++){
        set_pos_origin_force(i);
        update_particle_filter();
    }

    std::cout<< "trajectory finished!" << std::endl;
}

void Run_trajectories::set_pos_origin_force(std::size_t index){
        position = traj(index,arma::span(0,2)).st() + error_diff;
        //position(0) =  0.02;
        //position(1) = -0.04;
        //position(2) =  0.00;

        q.setX(traj(index,3));
        q.setY(traj(index,4));
        q.setZ(traj(index,5));
        q.setW(traj(index,6));
        orientation.setRotation(q);
        opti_rviz::type_conv::tf2mat(orientation,r_rot);
        force(0) = traj(index,7);
        force(1) = traj(index,8);
        force(2) = traj(index,9);
        if(index >= 1){
            position_tmp = traj(index-1,arma::span(0,2)).st()+ error_diff;
            u            = position - position_tmp;
        }

        if(reguliser != NULL){
            reguliser->update_position(position,r_rot);
        }
}

void Run_trajectories::add_reguliser(){

    if(ptr_particle_filter_manager != NULL){
        reguliser = std::shared_ptr<pf::Reguliser>(new pf::Reguliser(position,r_rot));
        ptr_particle_filter_manager->particle_filter->add_reguliser(reguliser);
    }

}

void Run_trajectories::update_sensor(){
    if(b_use_sensor){
        sensor_manager->update(Y,position,r_rot,force);
        if(b_print_sensor){
            sensor_manager->print(Y);
        }
    }
}



