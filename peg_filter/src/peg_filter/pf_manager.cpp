#include <peg_filter/pf_manager.h>
#include <optitrack_rviz/type_conversion.h>
#include <optitrack_rviz/listener.h>


namespace plugfilter {

PF_parameters::PF_parameters(pf::Measurement_h& measurement_h,
                             pf::likelihood_model& likelihood_f):
    measurement_h(measurement_h),
    likelihood_f(likelihood_f)
{
    particle_filter_type  = SIR;
    number_particles      = 1000;
    visualisation_mode    = opti_rviz::Vis_point_cloud::DEFAULT;
    pf_color_type         = pf::C_LIKE;
}



/////////////////////////////////////////
/// \brief Plug_pf_manager::Plug_pf_manager
/// \param number_particles
/// \param wrapped_objects
///


Plug_pf_manager::Plug_pf_manager(const PF_parameters& pf_parameters,
                                 const std::string& fixed_frame,
                                 const std::string& target_frame):
    sampler(3)
{

    particle_filter_type             = pf_parameters.particle_filter_type;

    peg_measurement_h                = pf_parameters.measurement_h;
    peg_likelihood_f                 = pf_parameters.likelihood_f;
    std::size_t Y_dim                = pf_parameters.Y_dim;

    initialise_motion_model();

    std::size_t number_particles     = pf_parameters.number_particles;
    color_t                          = pf_parameters.pf_color_type;
    viz_mode                         = pf_parameters.visualisation_mode;


    sampler.set_covariance(pf_parameters.sampling_parameters.kernel_covariance);

    switch(particle_filter_type){
    case SIR:
    {
        pf_sir_.reset(new pf::Particle_filter_sir(peg_likelihood_f,peg_measurement_h,peg_motion_model_f,number_particles,3,Y_dim,sampler) );
        break;
    }
    case PMF:
    {
        pf::Point_mass_filter::delta  delta_ ;
        pf::Point_mass_filter::length length_;

        int init_pmf_type = 1;

        if(init_pmf_type == 1)
        {
            delta_.m  = 0.01;
            delta_.n  = 0.04;
            delta_.k  = 0.04;

            length_.m = 0.2;
            length_.n = 0.7;
            length_.k = 0.2;
        }else if(init_pmf_type == 2)
        {
            delta_.m  = 0.2;
            delta_.n  = 0.2;
            delta_.k  = 0.2;

            length_.m = 1;
            length_.n = 1;
            length_.k = 1;
        }else if(init_pmf_type == 3)
        {
            delta_.m  = 0.04;
            delta_.n  = 0.04;
            delta_.k  = 0.04;

            length_.m = 1;
            length_.n = 1;
            length_.k = 1;
        }


        ptr_pmf_.reset(new pf::Point_mass_filter(peg_likelihood_f,peg_measurement_h,delta_,length_,Y_dim));
        break;
    }
    default:
    {
        std::cerr<< "no such particle filter implementation : " << particle_filter_type << std::endl;
        break;
    }
    }

    arma::colvec3 init_position;
    init_position.zeros();

    init_position(0) = -0.9;
    init_position(1) =  0.0;
    init_position(2) = 0.345;

    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once(fixed_frame,target_frame,transform);

    opti_rviz::type_conv::tf2vec(transform.getOrigin(),init_position);

    initialise_prior_pdf(init_position);
    bUpdate             = false;

}


void Plug_pf_manager::initialise_motion_model(){
    // create motion model
    arma::mat33 motion_noise_cov;
    float st_dev = 0.005;
    motion_noise_cov    = motion_noise_cov.eye() * st_dev * st_dev;
    peg_motion_model    = std::unique_ptr<Plug_motion_model>(new Plug_motion_model(motion_noise_cov) );
    peg_motion_model_f  = std::bind(&Plug_motion_model::motion_update,*peg_motion_model,std::placeholders::_1,std::placeholders::_2);

}

void Plug_pf_manager::update(const arma::colvec& Y, const arma::colvec& u,const arma::mat33& rot, const double duration_s){
    //        std::cout<< "update " << std::endl;

    switch(particle_filter_type){
    case SIR:
        pf_sir_->set_rotation(rot);
        pf_sir_->update(u,Y);
        break;
    case  PMF:
        ptr_pmf_->set_rotation(rot);
        ptr_pmf_->update(u,Y,duration_s);
        break;
    }
}

void Plug_pf_manager::init_visualise(ros::NodeHandle &node){
    switch(particle_filter_type){
    case SIR:
        pf_sir_->init_visualise(node,"pfilter");
        pf_sir_->set_visualisation_mode(viz_mode);
        pf_sir_->set_color_mode(color_t);
        break;
    case  PMF:
        ptr_pmf_->init_visualise(node,"pfilter");
        ptr_pmf_->set_visualisation_mode(viz_mode);
        ptr_pmf_->set_color_mode(color_t);
        break;
    }
}

void Plug_pf_manager::visualise(){

    switch(particle_filter_type){
    case SIR:
        if(pf_sir_ != NULL){
            pf_sir_->visualise();
        }
        break;
    case  PMF:
        if(ptr_pmf_ != NULL){
            ptr_pmf_->visualise();
        }
        break;
    }
}


//const arma::colvec3& T = arma::colvec3()
void Plug_pf_manager::initialise_prior_pdf(const arma::colvec3 &Plug_position){

    if(particle_filter_type == PMF)
    {

        ptr_pmf_->reset(Plug_position);
        arma::cube& P = ptr_pmf_->P;
        arma::mat& points = ptr_pmf_->points;

        int pmf_init_type = 0;


        if(pmf_init_type == 1)
        {
            int i,j,k;
            for(std::size_t n = 0; n < P.n_elem;n++){
                pf::ind2sub(i,j,k,P.n_rows,P.n_cols,n);
                P(i,j,k) =  exp(-(1.0/0.02) * ( arma::sum(arma::pow(points.row(n).st() - Plug_position,2))));
            }
        }else if (pmf_init_type == 2){
            for(std::size_t n = 0; n < P.n_elem;n++)
            {
                if( points(n,0) - Plug_position(0) < 0 || points(n,1) > 0.2)
                {
                    P.at(n) = 0;
                }

            }
        }



    }else{

        tf::Matrix3x3 rot;
        rot.setEulerYPR(0,0,0);

        // arma::vec3  origin = {{0.3,0.0,0}};
        // origin(2) = Plug_position(2);
        arma::vec3 origin = Plug_position;

        arma::mat   orientation(3,3);

        double      length = 0.2;
        double      width  = 1.0;
        double      height = 0.1;

        /*   double      length = 0.4;
        double      width  = 0.5;
        double      height = 0.05;*/

        arma::colvec3 rec_dim = {{length,width,height}};

        opti_rviz::type_conv::tf2mat(rot,orientation);

        arma::mat& particles   = pf_sir_->particles;

        if(arma::sum(Plug_position) != 0){
            update_center_rectangle(Plug_position,origin,rec_dim);
        }

        origin.print("initial origin");
        orientation.print("initial orientation");
        rec_dim.print("uniform dims");

        /*   origin(0) = -0.84875;
        origin(1) =  0.034115;
        origin(2) =  0.33551;*/

        uniform = stats::Uniform(origin,orientation,rec_dim(0),rec_dim(1),rec_dim(2));

        arma::vec x;
        for(std::size_t i = 0; i < pf_sir_->particles.n_rows;i++){
            x = uniform.sample();
            //x.print("x(" + boost::lexical_cast<std::string>(i) + ")" );
            particles(i,0) = x(0);
            particles(i,1) = x(1);
            particles(i,2) = x(2);
        }

        //std::cout<< "before create cube" << std::endl;
        //arma::mat& points,float l, float w, float h, float bin_w,   const arma::colvec3 position, const arma::vec3& rpy;
        // arma::vec3 rpy;
        // pf::Static_grid_filter::create_cube(particles,length,width,height,0.006,origin,rpy);

        if(particles.has_nan()){
            std::cout<< "=== bad initialisation of particles, NaN values are present" << std::endl;
            exit(0);
        }
        pf_sir_->reinitialise(particles);
        pf_sir_->reset_weights();
    }

    std::cout<< "=== particle filter initialised === " << std::endl;

}

void Plug_pf_manager::set_visualise_mode(opti_rviz::Vis_point_cloud::display_mode mode){
    viz_mode = mode;
}

void Plug_pf_manager::set_pf_color_type(pf::color_type color_t){
    this->color_t = color_t;
}
void Plug_pf_manager::update_center_rectangle(const arma::colvec3 &P,
                                              arma::colvec3 rec_pos,
                                              const arma::colvec3& dim)
{

    while(!is_in_rectangle(P,rec_pos,dim)){
        rec_pos(0) = rec_pos(0) + (P(0) - rec_pos(0)) * 0.01;
        rec_pos(1) = rec_pos(1) + (P(1) - rec_pos(1)) * 0.01;
    }

}

bool Plug_pf_manager::is_in_rectangle(const arma::colvec3&  P,
                                      const arma::colvec3   rec_pos,
                                      const arma::colvec3&  dim) const{

    // check X

    if(abs(P(0) - rec_pos(0))  >  dim(0)/2 + 0.01){
        return false;
    }

    if(abs(P(1) - rec_pos(1))  >  dim(1)/2 + 0.01 ){
        return false;
    }

    return true;
}



}
