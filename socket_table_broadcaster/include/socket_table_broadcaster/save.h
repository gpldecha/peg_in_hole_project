#ifndef SAVE_H_
#define SAVE_H_

// STL

#include <iostream>
#include <fstream>
#include <string>

// ROS

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>


namespace sock_tab{

class Save{

public:

    Save(tf::Vector3& origin,tf::Matrix3x3& orientation,const std::string& path_to_save, const std::string& target_frame_rviz);

    bool save();

private:

    const std::string path_to_save;
    const std::string target_frame_rviz;

    tf::Vector3&   origin;
    tf::Matrix3x3& orientation;

};

}

#endif


