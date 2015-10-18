#ifndef PLUG_PF_MANAGER_H_
#define PLUG_PF_MANAGER_H_

#include <particle_filter/particle_filter.h>
#include <particle_filter/particle_filter_gmm.h>
#include <particle_filter/static_grid_filter.h>

#include <peg_filter/plug_likelihood.h>
#include <peg_filter/motion_model.h>
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

    PF_parameters(wobj::WrapObject&  wrapped_objects);

public:

    wobj::WrapObject&                               wrapped_objects;
    pf_type                                         particle_filter_type;
    likeli::likelihood_type                         likelihood_t;
    std::size_t                                     number_particles;
    float                                           motion_noise;

    float                                           measurement_noise;

    pf::Sampling_Parameters                         sampling_parameters;

    opti_rviz::Vis_point_cloud::display_mode        visualisation_mode;
    pf::color_type                                  pf_color_type;
};

class Plug_pf_manager {

public:

    void tf2mat(const tf::Matrix3x3& m1, arma::mat& m2){

        m2(0,0)    = m1[0][0];
        m2(0,1)    = m1[0][1];
        m2(0,2)    = m1[0][2];

        m2(1,0)    = m1[1][0];
        m2(1,1)    = m1[1][1];
        m2(1,2)    = m1[1][2];

        m2(2,0)    = m1[2][0];
        m2(2,1)    = m1[2][1];
        m2(2,2)    = m1[2][2];
    }

    typedef std::shared_ptr<pf::Particle_filter_sir>                ptr_pf_sir;
    typedef std::shared_ptr<pf::Particle_filter_gmm>                ptr_pf_gmm;
    typedef std::shared_ptr<pf::Static_grid_filter>                 ptr_pf_grid;
    typedef std::shared_ptr<pf::Particle_filter>                    ptr_pf;
    typedef std::unique_ptr<likeli::Plug_likelihood_base>           uptr_like_base;


public:

    Plug_pf_manager(const PF_parameters& pf_parameters);

    void update(const arma::colvec& Y, const arma::colvec &u,const arma::mat33& rot);

    void visualise();

    void set_visualise_mode(opti_rviz::Vis_point_cloud::display_mode mode);

    void set_pf_color_type(pf::color_type color_t);

    void init_visualise(ros::NodeHandle& node);

    void initialise_prior_pdf(const arma::colvec3& Plug_position = arma::colvec3());

private:

    void initialise_motion_model();

    void initalise_likelihood_model(wobj::WrapObject& wrapped_objects, likeli::likelihood_type likelihood_t);


    inline float rescale(float x,float old_min,float old_max,float new_min,float new_max){
        return (((x - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min;
    }

    void update_center_rectangle(const arma::colvec3& P,arma::colvec3 rec_pos,const arma::colvec3& dim);

    bool is_in_rectangle(const arma::colvec3& P,const arma::colvec3 rec_pos,const arma::colvec3& dim) const;

public:

    ptr_pf    particle_filter;
    bool                                    bUpdate;

private:

    uptr_like_base                          plug_likelihood;
    std::unique_ptr<Plug_motion_model>      plug_motion_model;
    pf::Sampling                            sampler;
    pf::likelihood_model                    likelihood_f;
    pf::motion_model                        motion_model_f;
    Uniform                                 uniform;
    pf_type                                 particle_filter_type;
    opti_rviz::Vis_point_cloud::display_mode viz_mode;
    pf::color_type                          color_t;


};

}

#endif
