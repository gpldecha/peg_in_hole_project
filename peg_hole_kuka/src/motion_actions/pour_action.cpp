#include "motion_actions/pour_action.h"
#include <cmath>

Pour_action::Pour_action(ros::NodeHandle&   nh,
            const std::string& ee_state_pos_topic,
            const std::string& ee_cmd_pos_topic,
            const std::string& ee_cmd_ft_topic)
    :Base_action(nh,ee_state_pos_topic,ee_cmd_pos_topic,ee_cmd_ft_topic)
{


    initial_config          = true;
    tf_count                = 0;
    reachingThreshold       = 0.01;  // [m]
    orientationThreshold    = 0.02;  // [rad]
    model_dt                = 0.001; // [s]
    k                       = 1;
    masterType              = CDSController::MODEL_DYNAMICS;
    slaveType               = CDSController::UTHETA;


}

void Pour_action::initialize() {
    std::string ad;
    ros::NodeHandle _nh("~");
    _nh.getParam("action_mode", ad);
    if(ad == "lasa_fixed") {
        action_mode = ACTION_LASA_FIXED;
    } else if(ad == "romeo_fixed") {
        action_mode = ACTION_ROMEO_FIXED;
    } else if(ad == "vision") {
        action_mode = ACTION_VISION;
    } else {
        ROS_ERROR_STREAM("Unrecognized action type. Must be one of 'lasa_fixed', 'romeo_fixed', 'vision'. Found '"<<ad<<"'");
        throw;
    }

    _nh.getParam("world_frame", world_frame);
    _nh.getParam("model_base_path", base_path);
    _nh.getParam("simulation", simulation);
    _nh.getParam("model_dt", model_dt);
    _nh.getParam("reachingThreshold", reachingThreshold);
    _nh.getParam("orientationThreshold", orientationThreshold);

    ROS_INFO_STREAM("Selected Action Mode: " << ad);
}

bool Pour_action::executeCB(asrv::alib_server& as_, asrv::alib_feedback& feedback_,const lasa_action_planners::PLAN2CTRLGoalConstPtr & goal){


    if(goal->action_name == "home"){
        return learned_model_execution(PHASEHOME,as_,feedback_,goal);
    }else if(goal->action_name == "back"){
        return learned_model_execution(PHASEBACK,as_,feedback_,goal);
    }else if(goal->action_name == "pour"){
        return learned_model_execution(PHASEPOUR,as_,feedback_,goal);
    }else{
        return false;
    }
}

bool Pour_action::learned_model_execution(PouringPhase                phase,
                                          asrv::alib_server&          as_,
                                          asrv::alib_feedback&        feedback_,
                                          const asrv::cptrGoal&       goal)
{

    ROS_INFO_STREAM(" Model Path "<<base_path);
    ROS_INFO_STREAM("Learned model execution with phase "<<phase);
    ROS_INFO_STREAM(" Reaching threshold "<<reachingThreshold);
    ROS_INFO_STREAM(" Orientation threshold "<<orientationThreshold);
    ROS_INFO_STREAM(" Model DT "<<model_dt);

    tf::Transform trans_obj, trans_att;
    switch (action_mode) {
    case ACTION_ROMEO_FIXED:
        break;
    case ACTION_LASA_FIXED:

    case ACTION_VISION:
        trans_obj.setRotation(tf::Quaternion(goal->object_frame.rotation.x,goal->object_frame.rotation.y,
                                             goal->object_frame.rotation.z,goal->object_frame.rotation.w));
        trans_obj.setOrigin(tf::Vector3(goal->object_frame.translation.x, goal->object_frame.translation.y,
                                        goal->object_frame.translation.z));
        trans_att.setRotation(tf::Quaternion(goal->attractor_frame.rotation.x,goal->attractor_frame.rotation.y,
                                             goal->attractor_frame.rotation.z,goal->attractor_frame.rotation.w));
        trans_att.setOrigin(tf::Vector3(goal->attractor_frame.translation.x, goal->attractor_frame.translation.y,
                                        goal->attractor_frame.translation.z));
        break;
    default:
        break;
    }

    ros::Rate wait(1);
    tf::Transform  trans_final_target;

    // convert attractor information to world frame
    trans_final_target.mult(trans_obj, trans_att);

    ROS_INFO_STREAM("Final target origin "<<trans_final_target.getOrigin().getX()<<","<<trans_final_target.getOrigin().getY()<<","<<trans_final_target.getOrigin().getZ());
    ROS_INFO_STREAM("Final target orient "<<trans_final_target.getRotation().getX()<<","<<trans_final_target.getRotation().getY()<<","<<trans_final_target.getRotation().getZ()<<","<<trans_final_target.getRotation().getW());

    if (initial_config == true)
        curr_ee_pose = ee_pose;
    else
        curr_ee_pose = des_ee_pose;

    // Initialize CDS
    CDSExecution *cdsRun = new CDSExecution;
    cdsRun->initSimple(base_path, phase);
    cdsRun->setObjectFrame(toMatrix4(trans_obj));
    cdsRun->setAttractorFrame(toMatrix4(trans_att));
    cdsRun->setCurrentEEPose(toMatrix4(curr_ee_pose));
    cdsRun->setDT(model_dt);
    cdsRun->setMotionParameters(0.5,1,1,reachingThreshold, masterType, slaveType);
    cdsRun->postInit();

    ros::Duration loop_rate(model_dt);
    tf::Pose mNextRobotEEPose = curr_ee_pose;
    tf::Transform trans_ee;
    std::vector<double> gmr_in, gmr_out;
    gmr_in.resize(1);gmr_out.resize(1);
    double pos_err, ori_err, prog_curr;
    pos_err = 0; ori_err = 0; prog_curr = 0;

    ROS_INFO("Execution started");

    static tf::TransformBroadcaster br;
    while(ros::ok()) {
        if (initial_config == true)
            curr_ee_pose = ee_pose;
        else
            curr_ee_pose = des_ee_pose;

        // Publish attractors if running in simulation or with fixed values
        trans_ee.setRotation(tf::Quaternion(curr_ee_pose.getRotation()));
        trans_ee.setOrigin(tf::Vector3(curr_ee_pose.getOrigin()));

        // To Visualize EE Frames
        if (simulation==true){
            int frame_viz = int(model_dt*1000);
            if (tf_count==0 || tf_count%frame_viz==0){
                stringstream ss;
                ss <<  "/ee_tf_" << tf_count;
                br.sendTransform(tf::StampedTransform(trans_ee, ros::Time::now(), world_frame, ss.str()));
            }
            tf_count++;
        }
        else{
            br.sendTransform(tf::StampedTransform(trans_ee, ros::Time::now(), world_frame, "/ee_tf"));
        }

        br.sendTransform(tf::StampedTransform(trans_final_target, ros::Time::now(), world_frame, "/attractor"));
        br.sendTransform(tf::StampedTransform(trans_obj, ros::Time::now(), world_frame, "/object_frame"));

        // Current progress variable (position/orientation error).
        // TODO: send this back to action client as current progress
        pos_err = (trans_final_target.getOrigin() - curr_ee_pose.getOrigin()).length();
        //Real Orientation Error qdiff = acos(dot(q1_norm,q2_norm))*180/pi
        ori_err = acos(abs(trans_final_target.getRotation().dot(curr_ee_pose.getRotation())));
        ROS_INFO_STREAM_THROTTLE(0.5,"Position Threshold : " << reachingThreshold << " ... Current Error: "<<pos_err);
        ROS_INFO_STREAM_THROTTLE(0.5,"Orientation Threshold : " << orientationThreshold << " ... Current Error: "<<ori_err);

        double att_pos_err = (trans_final_target.getOrigin() - des_ee_pose.getOrigin()).length();
        double att_ori_err = acos(abs(trans_final_target.getRotation().dot(des_ee_pose.getRotation())));

        ROS_INFO_STREAM_THROTTLE(0.5,"Des-Att Position Error: " << att_pos_err);
        ROS_INFO_STREAM_THROTTLE(0.5,"Des-Att Orientation Error: " << att_ori_err);

        // Compute Next Desired EE Pose
           cdsRun->setCurrentEEPose(toMatrix4(mNextRobotEEPose));
           toPose(cdsRun->getNextEEPose(), mNextRobotEEPose);
           des_ee_pose = mNextRobotEEPose;

        // Make next pose the current pose for open-loop simulation
        if (simulation==true)
            initial_config=false;

        // If orientation error is VERY low or nan because of qdiff take target orientation
        if (att_ori_err < 0.005 || std::isnan(att_ori_err)) //[rad] and [m]//
//			if (isnan(att_ori_err)) //[rad] and [m]
            des_ee_pose.setRotation(tf::Quaternion(trans_final_target.getRotation()));

        // Send the computed pose from one of the above phases
        if (simulation==false)
            sendPose(des_ee_pose);

        // Broadcast/view Desired EE Pose
        br.sendTransform(tf::StampedTransform(des_ee_pose, ros::Time::now(), world_frame, "/des_ee_mp"));
        ROS_INFO_STREAM_THROTTLE(1, "Sent Position: "<<des_ee_pose.getOrigin().x()<<","<<des_ee_pose.getOrigin().y()<<","<<des_ee_pose.getOrigin().z());
        tf::Quaternion q = des_ee_pose.getRotation();
        q  = q.normalized();
        ROS_INFO_STREAM_THROTTLE(1, "Sent Orientation: "<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w());

        feedback_.progress = prog_curr;
        as_.publishFeedback(feedback_);

        if(pos_err < reachingThreshold && (ori_err < orientationThreshold || std::isnan(ori_err))) {
            break;
        }
        loop_rate.sleep();
    }
    delete cdsRun;
    return ros::ok();
}

