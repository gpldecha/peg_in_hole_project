#ifndef PEG_HOLE_POLICY_VELOCITY_CONTROLLER_H_
#define PEG_HOLE_POLICY_VELOCITY_CONTROLLER_H_

#include <ros/ros.h>
#include <tf/LinearMath/Vector3.h>
#include <control_toolbox/pid.h>
#include <boost/scoped_ptr.hpp>
#include "peg_hole_policy/gainsConfig.h"


namespace ph_policy{

class Velocity_controller{

public:

    typedef enum CTRL_TYPE{
        PID,
        CONSTANTE
    } CTRL_TYPE;

public:

    Velocity_controller(ros::NodeHandle &nh);


    /**
     * @brief update
     * @param dx_cmd : commanded velocity to robot
     * @param dx_des : wanted velocity
     * @param dx_msr : measured velocity
     */
    void update(tf::Vector3 &dx_cmd, const tf::Vector3 &dx_des, const tf::Vector3 &dx_current,const ros::Duration& ros_dt);

private:

    void gains_callback(peg_hole_policy::gainsConfig& config, uint32_t level);


private:

    std::vector<control_toolbox::Pid>  pid;

    double Kp,Ki,Kd,Ki_max,Ki_min;

    double C;

    boost::scoped_ptr< dynamic_reconfigure::Server< peg_hole_policy::gainsConfig> >    dynamic_server_gains_param;

    ros::NodeHandle nd1;


    CTRL_TYPE ctrl_type;

};

}


#endif
