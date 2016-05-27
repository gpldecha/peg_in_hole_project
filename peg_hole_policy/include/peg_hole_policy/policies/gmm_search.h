#ifndef GMM_SEARCH_PEG_H_
#define GMM_SEARCH_PEG_H_

#include "robot_planners/gmmPlanner.h"
#include "peg_hole_policy/stack_planner/state_machine.h"
#include "peg_sensor/peg_world_wrapper/peg_world_wrapper.h"

namespace ph_policy{

class GMM{

public:

    enum class GMM_TYPE{NONE,GMM,QEM,GREEDY};

public:

    GMM(SOCKET_TYPE socket_type);

    void update(arma::colvec3 &velocity, const arma::colvec &belief_state_SF, const std::vector<STATES> &states);

    void set_gmm(GMM_TYPE type);


private:

    bool                     b_air;
    planners::GMR_EE_Planner gmr_greedy;
    planners::GMAPlanner     gma_gmm, gma_qem;
    arma::colvec3            velocity_tmp;
    GMM_TYPE                 type;
    SOCKET_TYPE              socket_type;
    arma::colvec             belief_state_SF_tmp;
};

}


#endif
