#ifndef PEG_FILTER_FEATURES_H_
#define PEG_FILTER_FEATURES_H_

#include "peg_filter/belief_features/mode_feature.h"

#include <ros/ros.h>

#include <set>
#include <armadillo>
#include <boost/shared_ptr.hpp>
#include <map>
#include "base_bel_compress.h"


class Belief_compression{

public:

    Belief_compression();

    void update(const arma::colvec3 &velocity, const arma::mat &points, const arma::cube &P);

    bool set_method(const std::string& name);

    void add_bel_compress_method(BaseBeliefCompression *bel_compress_method);


private:

    //Mode_feature                     mode_features;
    std::map<std::string,BaseBeliefCompression*> methods;
    std::map<std::string,BaseBeliefCompression*>::iterator it;



};


#endif
