#include "peg_features/mode_feature.h"

Mode_feature::Mode_feature(ros::NodeHandle &nh, const arma::mat& points, const arma::colvec& weights)
    :points(points),weights(weights)
{

    pub = nh.advertise<std_msgs::Float64MultiArray>("mode_feature",10);

    get_mode_feature();
    bFirst = true;
    feature_msg.data.resize(4);
}

void Mode_feature::get_mode_feature(){

    max_w                    = weights.max(index_tmp);

    // first time the point is simply set to max
    if(bFirst){
        index                = index_tmp;
        max_tmp              = max_w;
        delta_max_threashold = 0.9 * max_w;
        bFirst               = false;
    }else{
        // if max_w is smaller than threashold set mode to new max.
        if(max_w < delta_max_threashold){
            index = index_tmp;
        }
    }
    mode_point  = points.row(index).st();
}

void Mode_feature::update(){

    get_mode_feature();

    feature_msg.data[X] = mode_point(X);
    feature_msg.data[Y] = mode_point(Y);
    feature_msg.data[Z] = mode_point(Z);
    feature_msg.data[W] = max_w;

    pub.publish(feature_msg);

}
