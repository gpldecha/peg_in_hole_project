#ifndef PEG_FIND_TABLE_H_
#define PEG_FIND_TABLE_H_

#include <armadillo>
#include "peg_hole_policy/policies/base_find.h"


/**
 * @brief The Find_table class
 *
 *  Simply takes the shortest distance to the surface of a table.
 *
 *
 **/

namespace ph_policy{

class Find_table : public Base_find {

public:

    Find_table(Peg_world_wrapper &peg_world_wrapper);

    void get_linear_velocity(tf::Vector3& velocity,const tf::Vector3& peg_origin);

    void get_linear_velocity(tf::Vector3 &velocity, const tf::Vector3 &peg_origin,tf::Vector3& des_origin,tf::Quaternion& des_orient);

private:

    wobj::WBox*             ptr_table_wbox;
    bool bFirst;
    geo::fCVec3             surf_proj;

};




}


#endif
