#include "peg_filter/belief_features/mode_feature.h"

Mode_feature::Mode_feature(ros::NodeHandle &nh, const arma::mat& points, const arma::colvec& weights)
    :points(points),weights(weights),vis_points(nh,"bel_x")
{

    pub = nh.advertise<std_msgs::Float64MultiArray>("mode_feature",10);

    get_mode_feature();
    bFirst = true;
    feature_msg.data.resize(4);

    vis_pts.resize(1,3);

    vis_points.r = 0;
    vis_points.g = 0;
    vis_points.b = 1;
    vis_points.scale = 0.05;
    vis_points.alpha = 1;

    vis_points.initialise("world",vis_pts);


}

void Mode_feature::get_mode_feature(){

    max_w                    = weights.max(index);
    delta_max_threashold     = 0.95 * max_w;

    // first time the point is simply set to max
    if(bFirst){
        max_tmp              = max_w;
        index_tmp            = index;
        bFirst               = false;
    }else{
        // if max_w is smaller than threashold set mode to new max.
        if(max_tmp < delta_max_threashold){
            index_tmp = index;
            max_tmp   = max_w;
        }
    }
    mode_point  = points.row(index_tmp).st();
}

void Mode_feature::update(){

    get_mode_feature();

    if(mode_point.has_nan()){

        std::cout<< "MODE_POINT HAS NAN" << std::endl;
        exit(0);
    }

    feature_msg.data[X] = mode_point(X);
    feature_msg.data[Y] = mode_point(Y);
    feature_msg.data[Z] = mode_point(Z);
    feature_msg.data[W] = max_tmp;

    pub.publish(feature_msg);

}

void Mode_feature::visualize(){
    vis_pts(0,0) = mode_point(0);
    vis_pts(0,1) = mode_point(1);
    vis_pts(0,2) = mode_point(2);
    vis_points.update(vis_pts);
    vis_points.publish();

}
