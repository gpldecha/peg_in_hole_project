#include "socket_table_broadcaster/load.h"

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <fstream>

#include <array>
#include <ros/ros.h>

namespace sock_tab{


bool Load::load(tf::Vector3& origin, tf::Quaternion& q,std::string path_to_save,std::string target_frame){

    std::string folder_load = path_to_save + "/" + target_frame;

    if( !(boost::filesystem::exists(folder_load))){
        std::cerr << "FAILED to open folder: " + target_frame << "\n";
        return false;
    }else{
        ROS_INFO("Open folder: %s",folder_load.c_str());
    }


    std::string file_origin      = folder_load  + "/origin.csv";
    std::string file_orientation = folder_load  + "/orientation.csv";

    ROS_INFO("Open file_origin:      %s",file_origin.c_str());
    ROS_INFO("Open file_orientation: %s",file_orientation.c_str());



    std::ifstream file_stream_origin(file_origin);
    if(file_stream_origin.is_open()){
        std::string line;
        while(std::getline(file_stream_origin, line)){
            std::istringstream s(line);
            std::string field;
            std::size_t i = 0;
            while (getline(s, field,',')){
                origin[i] = boost::lexical_cast<float>(field);
                i++;
            }
        }
    }else{
        std::cerr<< "failed to open: " << file_origin << std::endl;
        return false;
    }

    std::ifstream file_stream_orientation(file_orientation);
    if(file_stream_origin.is_open()){
        std::string line;
        std::array<float,4> orien;
        while(std::getline(file_stream_orientation, line)){
            std::istringstream s(line);
            std::string field;
            std::size_t i = 0;
            while (getline(s, field,',')){
                orien[i] = boost::lexical_cast<float>(field);
                i++;
            }
        }
        q.setValue(orien[0],orien[1],orien[2],orien[3]);

    }else{
        std::cerr<< "failed to open: " << file_origin << std::endl;
        return false;
    }

    return true;
}


}
