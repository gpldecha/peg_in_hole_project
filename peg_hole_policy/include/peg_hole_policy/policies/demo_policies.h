#ifndef DEMO_POLICY_PEG_H_
#define DEMO_POLICY_PEG_H_

#include "specialised_policy.h"
#include "gmm_search.h"
#include "simple_policies.h"


namespace ph_policy{

class Demo_policies{


public:

    enum class DEMO_TYPE{DEMO_1,DEMO_2,DEMO_3};


    enum class POLICY_TYPE{DEMO,GREEDY,INSERT,FORWARD};

public:

    Demo_policies(wobj::WrapObject& wrap_object,SOCKET_TYPE socket_type,ph_policy::GMM& gmm);

    void set_demo(DEMO_TYPE demo_type);

    void reset();

    void update(arma::colvec3&              velocity,
                const arma::colvec3&        mls_WF,
                const arma::colvec3&        mls_SF,
                const arma::colvec3&        socket_pos_WF,
                const arma::colvec3&        peg_origin_WF,
                const arma::colvec&         Y_c,
                const std::vector<STATES>&  states,
                Insert_peg&                 insert_peg,
                Get_back_on&                get_back_on,
                Force_control&              force_control,
                Peg_sensor_model&           peg_sensor_model,
                const arma::colvec &belief_state_SF,
                const arma::colvec3&   open_loop_x_origin_arma_WF);


private:

    inline double dist_yz(const arma::colvec3& v1, const arma::colvec3& v2){
        return std::sqrt( (v1(1) - v2(1)) * (v1(1) - v2(1)) + (v1(2) - v2(2)) * (v1(2) - v2(2))  );
    }


private:


    boost::shared_ptr<ph_policy::Specialised>  spolicy_one;
    boost::shared_ptr<ph_policy::Specialised>  spolicy_two;
    boost::shared_ptr<ph_policy::Specialised>  spolicy_three;

    ph_policy::GMM&             gmm;
    Forward_insert              forward_policy;


    bool                        in_socket;
    DEMO_TYPE                   demo_type;
    POLICY_TYPE                 policy_type;
    arma::colvec3               target_WF;

};

}

#endif
