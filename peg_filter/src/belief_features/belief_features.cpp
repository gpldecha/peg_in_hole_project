#include "peg_filter/belief_features/belief_features.h"


Belief_compression::Belief_compression()
{
    it=methods.end();
}

bool Belief_compression::set_method(const std::string& name){

    it = methods.find(name);

    if ( methods.find(name) != methods.end() ) {
      // found
        return true;
    } else {
        it=methods.end();
      // not fount found
        return false;
    }

}

void Belief_compression::add_bel_compress_method(BaseBeliefCompression* bel_compress_method){
    methods[bel_compress_method->get_name()] = bel_compress_method;
}

void Belief_compression::update(const arma::colvec3& velocity,const arma::mat &points, const arma::cube& P){
    if(it != methods.end())
    {
        (it->second)->update(velocity,points,P);
    }
}

