#include "peg_hole_policy/policies/base_find.h"

namespace ph_policy{

Base_find::Base_find(Peg_world_wrapper &peg_world_wrapper):
     peg_world_wrapper(peg_world_wrapper)
{

    velocity_reguliser.set_max_speed_ms(0.025);
    velocity_reguliser.set_min_speed_ms(0.001);
    beta_vel_reg = 10;
}

}
