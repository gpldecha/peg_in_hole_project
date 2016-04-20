#ifndef GMM_SEARCH_PEG_H_
#define GMM_SEARCH_PEG_H_

#include "robot_planners/gmmPlanner.h"

namespace ph_policy{

class GMM{

public:

    enum class GMM_TYPE{NONE,GMM,QEM,GREEDY};

public:

    GMM();

    void update(arma::colvec3 &velocity, const arma::colvec &belief_state_SF);

    void set_gmm(GMM_TYPE type);


private:


    planners::GMR_EE_Planner gmr_greedy;
    planners::GMAPlanner     gma_gmm, gma_qem;
    arma::colvec3            velocity_tmp;
    GMM_TYPE    type;


};

}


#endif
