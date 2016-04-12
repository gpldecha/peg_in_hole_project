#ifndef PEG_PF_MANAGER_H_
#define PEG_PF_MANAGER_H_

#include <particle_filter/particle_filter_definitions.h>
#include <particle_filter/particle_filter.h>
#include <particle_filter/particle_filter_gmm.h>
#include <particle_filter/static_grid_filter.h>
#include <point_mass_filter/point_mass_filter.h>

#include <peg_filter/likelihood.h>
#include <peg_filter/motion.h>


#include <particle_filter/String_cmd.h>
#include <visualise/vis_point_cloud.h>
#include <visualise/vis_gmm.h>

#include <memory>
#include <functional>
#include <statistics/distributions/distributions.h>
#include <visualise/colormap.h>
#include <array>
#include <map>

namespace plugfilter {


typedef enum {C_LIKE,C_WEIGHTS} color_type;
typedef enum {SIR,GMM,HIST,PMF} pf_type;



class PF_parameters {

public:

    PF_parameters();


    PF_parameters(pf::Measurement_h&     measurement_h,
                  pf::likelihood_model& likelihood_f);

public:

    pf_type                                         particle_filter_type;
    std::size_t                                     number_particles;
    std::size_t                                     Y_dim;

    float                                           motion_noise;
    float                                           measurement_noise;

    pf::Measurement_h&                               measurement_h;
    pf::likelihood_model&                            likelihood_f;
    pf::Sampling_Parameters                          sampling_parameters;

    opti_rviz::Vis_point_cloud::display_mode        visualisation_mode;
    pf::color_type                                  pf_color_type;
};

class Plug_pf_manager {

public:

public:

    Plug_pf_manager();

    void add(pf::Base_particle_filter* base_particle_filter,const std::string& name);

    bool select_method(const std::string& name);

    void update(const arma::colvec& Y, const arma::colvec &u,const arma::mat33& rot, double duration_s);

    void visualise();

    void set_visualise_mode(opti_rviz::Vis_point_cloud::display_mode mode);

    void set_pf_color_type(pf::color_type color_t);

    void init_visualise(ros::NodeHandle& node);

private:

    std::map<std::string,pf::Base_particle_filter*> filter_methods;
    std::map<std::string,pf::Base_particle_filter*>::iterator it_filter_method;

};

}

#endif
