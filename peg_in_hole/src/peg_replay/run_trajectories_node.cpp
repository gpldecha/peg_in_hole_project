#include <peg_in_hole/run_trajectories.h>
#include <world_wrapper/world_wrapper.h>
#include <objects/socket_one.h>

int main(int argc,char** argv){


    ww::World_wrapper       world_wrapper;
    geo::fCVec3 origin_      = {{0,0,-0.02/2 - 0.03/2}};
    geo::fCVec3 dim_         = {{0.8,0.4,0.05}};
    geo::fCVec3 orientation_ = {{M_PI/2,0,M_PI/2}};

    wobj::WBox wsocket_wall("socket_wall",dim_,origin_,orientation_);

    /// add a socket
    tf::Vector3 origin(0,0,0);
    tf::Vector3 rpy(M_PI/2,0,M_PI/2);

    obj::Socket_one socket_one("socket_one",origin,rpy,1);
    world_wrapper.wrapped_objects.push_back_box(wsocket_wall);
    world_wrapper.wrapped_objects.push_back_socket(socket_one.wsocket);
    world_wrapper.wrapped_objects.push_back_box(socket_one.wbox);


    plugfilter::PF_parameters   pf_parameters(world_wrapper.wrapped_objects);
                                pf_parameters.number_particles    = 4000;

    Sensor_parameters   sensor_parameters(world_wrapper.wrapped_objects);
                        sensor_parameters.t_sensor = psm::SIMPLE_CONTACT_DIST;


    Run_trajectories    run_trajectories;
             //           run_trajectories.setup(pf_parameters,sensor_parameters);


    std::string folder_name =  "/home/guillaume/MatlabWorkSpace/peg_in_hole/TextData";
    std::string subject_name = "Albert";
    std::string plug_type    = "A";


    std::size_t i = 1;

    std::string folder_path = folder_name + "/" + subject_name + "/";
    std::string file_name   = subject_name + "_" + plug_type + "_" + boost::lexical_cast<std::string>(i) + "_.txt";

    run_trajectories.load(folder_path,file_name);




    return 0;
}
