#include "socket_table_broadcaster/save.h"

#include <boost/filesystem.hpp>
namespace sock_tab{

Save::Save(tf::Vector3& origin,tf::Matrix3x3& orientation,const std::string &path_to_save, const std::string &target_frame_rviz):
    origin(origin),orientation(orientation),path_to_save(path_to_save),target_frame_rviz(target_frame_rviz)
{

}


bool Save::save(){

    std::string folder_save = path_to_save + "/" + target_frame_rviz;

    if( !(boost::filesystem::exists(folder_save))){
        if (boost::filesystem::create_directory(folder_save)){
            std::cout << "new folder " + target_frame_rviz + " created" << "\n";
        }else{
            std::cerr << "FAILED to create new folder " + target_frame_rviz << "\n";
            return false;
        }
    }

    std::string file_origin      = folder_save  + "/origin.csv";
    std::string file_orientation = folder_save  + "/orientation.csv";

    std::ofstream file_stream1(file_origin);
    if(file_stream1.is_open()){

        file_stream1 << origin.x() << ","
                     << origin.y() << ","
                     << origin.z() << std::endl;

        file_stream1.close();
        std::cout<< "saved " + file_origin + " !" << std::endl;
    }else{
        std::cerr << " FAILED to open: " + file_origin << std::endl;
        return false;
    }

    std::ofstream file_stream2(file_orientation);
    if(file_stream2.is_open()){
        tf::Quaternion q;
        orientation.getRotation(q);
        file_stream2 << q.getX()  << ","
                     << q.getY()  << ","
                     << q.getZ()  << ","
                     << q.getW()  << std::endl;

        file_stream2.close();
        std::cout<< "saved " + file_orientation + " !" << std::endl;
    }else{
        std::cerr << " FAILED to open: " + file_orientation << std::endl;
        return false;
    }

    return true;
}

}
