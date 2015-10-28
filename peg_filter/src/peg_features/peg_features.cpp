#include "peg_features/peg_features.h"


Peg_filter_features::Peg_filter_features(ros::NodeHandle& nh, const arma::mat& points,const arma::colvec &weights)
    : mode_features(nh,points,weights)
{

}

void Peg_filter_features::add_feature(feature_type f_type){
    feature_types.insert(f_type);
}

void Peg_filter_features::update(){

    mode_features.update();

}

