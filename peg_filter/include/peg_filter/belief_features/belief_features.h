#ifndef PEG_FILTER_FEATURES_H_
#define PEG_FILTER_FEATURES_H_

#include "peg_filter/belief_features/mode_feature.h"

#include <ros/ros.h>

#include <set>
#include <armadillo>



class Belief_features{

    typedef enum feature_type{
        moste_likely,
        mean,
        entropy
    } feature_type;

public:

    Belief_features(ros::NodeHandle& nh, const arma::mat& points,const arma::colvec& weights);

    void update();

    void add_feature(feature_type f_type);


private:

    Mode_feature                     mode_features;

    std::set<feature_type>           feature_types;
    std::set<feature_type>::iterator it;



private:


};


#endif
