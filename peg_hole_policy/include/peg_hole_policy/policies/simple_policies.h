#ifndef SIMPLE_POLICIES_H_
#define SMPLE_POLICIES_H_


#include "peg_hole_policy/stack_planner/state_machine.h"
#include <vector>
#include <armadillo>

namespace ph_policy{

class Simple_policies{

public:

    Simple_policies();

    void update(arma::colvec3&                 velocity,
                 const arma::colvec3&           mls_WF,
                 const arma::colvec3&           mls_SF,
                 const arma::colvec3&           socket_pos_WF,
                 const arma::colvec3&           peg_origin,
                 const arma::colvec&            Y_c,
                 const std::vector<STATES>&     states);


private:

    bool b_air;
    arma::colvec3 velocity_tmp;
    arma::colvec3 target;


};

class Forward_insert{

public:

    Forward_insert();

    void reset();

    void update(arma::colvec3& velocity,const arma::colvec3& peg_origin);

private:

    bool          bFirst;
    arma::colvec3 target_x;

};

}


#endif
