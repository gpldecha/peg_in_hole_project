#ifndef PLUG_WORLD_WRAPPER_H_
#define PLUG_WORLD_WRAPPER_H_

#include <world_wrapper/world_wrapper.h>
#include <objects/socket_one.h>
#include <optitrack_rviz/listener.h>

class Plug_world_wrapper{

public:

    Plug_world_wrapper(){}

    Plug_world_wrapper(const std::string& table_urdf){
        //initialise_urdf(table_urdf);
        initialise_objects();
    }

    void initialise_urdf(const std::string& table_urdfs){

        world_wrapper.loadURDF(table_urdfs);
        world_wrapper.initialise_origin_orientation(world_wrapper,"world");
        geo::fCVec3 T = {{0,0,-0.02}};
        for(std::size_t i = 0; i < world_wrapper.wrapped_objects.wboxes.size();i++){
            world_wrapper.wrapped_objects.wboxes[i].transform(T);
        }

    }

    void initialise_objects(){


        tf::StampedTransform transform;
        opti_rviz::Listener::get_tf_once("world_frame","link_wall",transform);
        opti_rviz::Listener::print(transform);



        tf::Vector3     wall_origin = transform.getOrigin();

        geo::fCVec3 origin_      = {{(float)wall_origin.x(),(float)wall_origin.y(),(float)wall_origin.z()}};//{{0,0,-0.02/2}};
        geo::fCVec3 dim_         = {{0.02,0.8,0.4}};
        geo::fCVec3 orientation_ = {{0,0,0}};

        wobj::WBox  wsocket_wall("link_wall",dim_,origin_,orientation_);

        opti_rviz::Listener::get_tf_once("world_frame","link_socket",transform);
        opti_rviz::Listener::print(transform);

        /// add a socket
        tf::Vector3 origin = transform.getOrigin();
        //tf::Vector3 rpy(0,0,0);
        tf::Vector3 rpy(M_PI/2,0,M_PI/2);
        obj::Socket_one socket_one("link_socket","link_wall",origin,rpy,1);
/*


        geo::fCVec3 origin_      = {{0,0,-0.02/2}};
        geo::fCVec3 dim_         = {{0.8,0.4,0.02}};
        geo::fCVec3 orientation_ = {{M_PI/2,0,M_PI/2}};

        wobj::WBox wsocket_wall("socket_wall",dim_,origin_,orientation_);

        /// add a socket
        tf::Vector3 origin(0,0,0);
        tf::Vector3 rpy(M_PI/2,0,0);

        obj::Socket_one socket_one("link_socket","link_wall",origin,rpy,1);*/

        world_wrapper.wrapped_objects.push_back_box(wsocket_wall);
        world_wrapper.wrapped_objects.push_back_box(socket_one.wbox);
        world_wrapper.wrapped_objects.push_back_socket(socket_one.wsocket);



    }

    ww::World_wrapper& get_world_wrapper(){
        return world_wrapper;
    }

    wobj::WrapObject& get_wrapped_objects(){
        return world_wrapper.wrapped_objects;
    }

public:

         ww::World_wrapper  world_wrapper;
         obj::Socket_one    socket_one;

};


#endif
