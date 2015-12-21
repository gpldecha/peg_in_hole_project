#ifndef RUN_TRAJECTORIES_H_
#define RUN_TRAJECTORIES_H_

#include <ros/ros.h>

#include <tf/LinearMath/Quaternion.h>

#include <peg_filter/pf_manager.h>
#include <peg_filter/likelihood.h>

#include "peg_sensor/peg_sensor_model/distance_features.h"
#include "peg_sensor/peg_sensor_model/peg_distance_model.h"

#include <optitrack_rviz/type_conversion.h>
#include <optitrack_rviz/type_conversion.h>

#include "wrapobject.h"


#include <string>
#include <vector>
#include <armadillo>
#include <memory>


class Sensor_parameters{
public:
    Sensor_parameters(wobj::WrapObject& wrapped_objects):
        wrapped_objects(wrapped_objects)
    {
    //    t_sensor        = psm;
        b_print_sensor  = false;
    }

    psm::type_sensor      t_sensor;
    wobj::WrapObject&     wrapped_objects;
    bool                  b_print_sensor;

};


class Run_trajectories{

public:

    typedef std::shared_ptr<plugfilter::Plug_pf_manager> ptr_pf_manager;
    typedef std::shared_ptr<psm::Sensor_manager>         ptr_sensor_manager;



public:


    Run_trajectories();

    void set_initial_position(tf::Vector3& position, tf::Matrix3x3& r_rot);

    void setup_pf(const plugfilter::PF_parameters& pf_parameters);

    void setup_sensor(const Sensor_parameters& sensor_parameters);

   // void setup(const plugfilter::PF_parameters& pf_parameters,const Sensor_parameters& sensor_parameters);

    bool load(const std::string& folder_path, const std::string& file_name);

    void run();

    void inline one_update(std::size_t i){
        set_pos_origin_force(i);
        update_particle_filter();
    }

    void inline update_particle_filter(){
        if(b_use_filter){
            //std::cout<< "--- update filter --- " << std::endl;
            ptr_particle_filter_manager->update(Y,u,r_rot);
        }
    }

    void add_reguliser();


    void set_pos_origin_force(std::size_t index);

    void update_sensor();


private:




private:

    bool                        b_use_filter;
    bool                        b_use_sensor;
    bool                        b_print_sensor;

public:

    ptr_pf_manager               ptr_particle_filter_manager;
    ptr_sensor_manager           sensor_manager;
    arma::mat                    traj;

    arma::colvec3                   position, position_tmp;
    arma::colvec3                   error_diff;
    arma::mat33                     r_rot;
    arma::fcolvec3                  force;
    tf::Matrix3x3                   orientation;
    tf::Quaternion                  q;
    arma::colvec3                   u;
    arma::colvec                    Y;

    std::shared_ptr<pf::Reguliser>          reguliser;


};




class Trajectory_replay{

public:

    Trajectory_replay(wobj::WrapObject& wrapped_objects){
        p_distance_features = new psm::Distance_features(wrapped_objects);
    }

    bool load_trajectory(const std::string& path_to_trajectory){
        return traj.load(path_to_trajectory);
    }

    void run(){

        data.resize(traj.n_rows,2);
        data.zeros();

     /*   for(std::size_t i = 0; i < traj.n_rows;i++){
            set_pos_origin_force(i);
            p_distance_features->update_position(position,rot);
            p_distance_features->get_distance_features(position);
            data(i,0) = p_distance_features->min_distance_surface;
            data(i,1) = p_distance_features->min_distance_edge;
            if(data(i,0) > 10){
                data(i,0) = -0.1;
                data(i,1) = -0.1;
            }
        }*/

        std::cout<< "trajectory finished!" << std::endl;
    }

    void save(const std::string& path_folder, const std::string& file_name){
        std::string full_path = path_folder + file_name;
        data.save(full_path, arma::raw_ascii);

    }


    void set_pos_origin_force(std::size_t index){
        if(traj.n_rows != 0 && index < traj.n_rows){
            position = traj(index,arma::span(0,2)).st();
            q.setX(traj(index,3));
            q.setY(traj(index,4));
            q.setZ(traj(index,5));
            q.setW(traj(index,6));
            orientation.setRotation(q);
            opti_rviz::type_conv::tf2mat(orientation,rot);
            force(0) = traj(index,7);
            force(1) = traj(index,8);
            force(2) = traj(index,9);
        }
    }



private:

    psm::Distance_features*         p_distance_features;

    arma::mat                       data;
    arma::mat                       traj;
    arma::colvec3                   position;
    arma::mat33                     rot;
    arma::colvec3                   force;
    tf::Matrix3x3                   orientation;
    tf::Quaternion                  q;


};

#endif
