#include "peg_filter/belief_features/belief_features.h"


Belief_features::Belief_features(ros::NodeHandle& nh, const arma::mat& points,const arma::colvec &weights)
    : mode_features(nh,points,weights)
{

}

void Belief_features::add_feature(feature_type f_type){
    feature_types.insert(f_type);
}

void Belief_features::update(){

    mode_features.update();
    mode_features.visualize();

}

