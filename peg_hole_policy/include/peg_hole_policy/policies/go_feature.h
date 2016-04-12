#ifndef GO_FEATURE_H_
#define GO_FEATURE_H_

#include <armadillo>
#include <wrapobject.h>

namespace ph_policy{


class Go_freature{

public:

    enum class STATUS{
        RUNNING,
        FINISHED,
    };

public:


    Go_freature(wobj::WrapObject &wrap_object);

    Go_freature::STATUS get_status() const;

    void   set_status(Go_freature::STATUS status);

    void reset();

    void set_target(const arma::colvec3& pos_WF);

    void update(arma::colvec3& velocity, const arma::colvec3& mode_pos_WF);

public:

    arma::colvec3           w_tl,w_tr,w_bl,w_br;
    arma::colvec3           s_tl,s_tr,s_bl,s_br;
    arma::colvec3           w_ml,w_mr,w_mt,w_mb;
    arma::colvec3           s_ml,s_mr,s_mt,s_mb;

private:

    wobj::WBox&             wall_box;
    wobj::WBox&             socket_box;
    bool                    bSetTarget;


    arma::colvec3               target_pos;
    arma::colvec3               target_velocity;
    Go_freature::STATUS         status;

};

}

#endif
