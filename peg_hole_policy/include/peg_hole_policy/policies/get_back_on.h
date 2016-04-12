#ifndef GET_BACK_ON_H_
#define GET_BACK_ON_H_

/**
 * @brief If the robot end-effector falls of the table, this policy will safely
 *        bring back the end-effector back onto the table.
 **/

#include <armadillo>
#include <wrapobject.h>
#include <optitrack_rviz/listener.h>

namespace ph_policy{

class Get_back_on{

public:

    enum class STATUS{
        RUNNING,
        FINISHED,
    };

public:

    Get_back_on(wobj::WBox& wall_box);

    void update(arma::colvec3& velocity,const arma::colvec3 &mode_pos_SF, const arma::colvec3 &mode_pos_WF, const arma::colvec& Y_c);

    STATUS get_status() const;

    void   set_status(STATUS status);

    void reset();

private:

    void planner(const arma::colvec3 &mode_pos_SF, const arma::colvec3& mode_pos_WF, const arma::colvec &Y_c, double dist_threashod=0.01);

    void get_way_pts(const arma::colvec3 &mode_pos);

private:

    wobj::WBox& wall_box;
    arma::fcolvec3 P;
    arma::colvec3 edge_P;
    std::vector<arma::colvec3> wpts;
    arma::colvec3 target;
    arma::colvec3 middle;
    std::size_t index;

    STATUS status;


};


}

#endif
