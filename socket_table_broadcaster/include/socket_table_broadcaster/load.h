#ifndef LOAD_H_
#define LOAD_H_

// ROS

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>

// STL

#include <string>

namespace sock_tab{

class Load{

public:

   static bool load(tf::Vector3& origin, tf::Quaternion& q,std::string path_to_save,std::string target_frame_rviz);

};

}

#endif
