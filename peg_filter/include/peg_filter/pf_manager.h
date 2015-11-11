#ifndef PEG_PF_MANAGER_H_
#define PEG_PF_MANAGER_H_

#include <particle_filter/particle_filter_definitions.h>
#include <particle_filter/particle_filter.h>
#include <particle_filter/particle_filter_gmm.h>
#include <particle_filter/static_grid_filter.h>

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
typedef enum {SIR,GMM,HIST} pf_type;



class PF_parameters {

public:

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

    typedef std::shared_ptr<pf::Particle_filter_sir>                ptr_pf_sir;
    typedef std::shared_ptr<pf::Particle_filter_gmm>                ptr_pf_gmm;
    typedef std::shared_ptr<pf::Static_grid_filter>                 ptr_pf_grid;
    typedef std::shared_ptr<pf::Particle_filter>                    ptr_pf;
 //   typedef std::unique_ptr<likeli::Plug_likelihood_base>           uptr_like_base;


public:

    Plug_pf_manager(const PF_parameters& pf_parameters);

    void update(const arma::colvec& Y, const arma::colvec &u,const arma::mat33& rot);

    void visualise();

    void set_visualise_mode(opti_rviz::Vis_point_cloud::display_mode mode);

    void set_pf_color_type(pf::color_type color_t);

    void init_visualise(ros::NodeHandle& node);

    void initialise_prior_pdf(const arma::colvec3& Plug_position);

private:

    void initialise_motion_model();


    inline float rescale(float x,float old_min,float old_max,float new_min,float new_max){
        return (((x - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min;
    }

    void update_center_rectangle(const arma::colvec3& P,arma::colvec3 rec_pos,const arma::colvec3& dim);

    bool is_in_rectangle(const arma::colvec3& P,const arma::colvec3 rec_pos,const arma::colvec3& dim) const;

public:

    ptr_pf                                  particle_filter;
    bool                                    bUpdate;

private:

    std::unique_ptr<Plug_motion_model>      peg_motion_model;

    pf::Sampling                            sampler;

    /// The three necessary functions needed for the particle filter

    pf::likelihood_model                    peg_likelihood_f;
    pf::Measurement_h                       peg_measurement_h;
    pf::motion_model                        peg_motion_model_f;


    Uniform                                 uniform;
    pf_type                                 particle_filter_type;
    opti_rviz::Vis_point_cloud::display_mode viz_mode;
    pf::color_type                          color_t;


};

}

#endif
