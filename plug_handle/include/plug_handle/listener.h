#ifndef LISTENER_H_
#define LISTENER_H_


//STL

#include <string>

namespace plugh{

class Listener{

public:

    Listener(const std::string &fixed_frame_,const std::string &target_frame_vision_);

    void update(avec3& origin,mat3& orientation);


};

}


#endif
