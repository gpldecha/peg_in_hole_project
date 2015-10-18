#include <peg_filter/pf_manager.h>


namespace plugfilter {

PF_parameters::PF_parameters(wobj::WrapObject&  wrapped_objects):
wrapped_objects(wrapped_objects)
{

    particle_filter_type = SIR;
    likelihood_t         = likeli::THREE_PIN;
    number_particles     = 4000;
    visualisation_mode   = opti_rviz::Vis_point_cloud::DEFAULT;
    pf_color_type        = pf::C_WEIGHTS;
}



/////////////////////////////////////////
/// \brief Plug_pf_manager::Plug_pf_manager
/// \param number_particles
/// \param wrapped_objects
///


Plug_pf_manager::Plug_pf_manager(const PF_parameters& pf_parameters):
                                 sampler(3)
{

   particle_filter_type             = pf_parameters.particle_filter_type;
   likeli::likelihood_type L_type   = pf_parameters.likelihood_t;
   std::size_t number_particles     = pf_parameters.number_particles;
   color_t                          = pf_parameters.pf_color_type;
   viz_mode                         = pf_parameters.visualisation_mode;


    sampler.set_covariance(pf_parameters.sampling_parameters.kernel_covariance);

    initialise_motion_model();
    initalise_likelihood_model(pf_parameters.wrapped_objects,L_type);


    switch(particle_filter_type){
    case SIR:
    {
        std::cout<< "Sample Importance Resample (particle filter)" << std::endl;
        particle_filter = ptr_pf_sir(new pf::Particle_filter_sir(likelihood_f,motion_model_f,number_particles,3,sampler) );

        break;
    }
    case GMM:
    {
        std::cout<< "Regulariezd GMM (particle filter)" << std::endl;
        mean_shift::MeanShift_Parameters mean_shift_parameters;
        mean_shift_parameters.bandwidth = 1.0/(0.01 * 0.01);
        particle_filter = ptr_pf_gmm (new pf::Particle_filter_gmm(likelihood_f,motion_model_f,number_particles,3,mean_shift_parameters,20));
        break;
    }
    case HIST:
    {
        std::cout<< "Histogram (particle filter)" << std::endl;
        particle_filter = ptr_pf_grid(new pf::Static_grid_filter(likelihood_f,motion_model_f,0,3));
        break;
    }
    default:
    {
        std::cerr<< "no such particle filter implementation : " << particle_filter_type << std::endl;
        break;
    }
    }

    initialise_prior_pdf();

    bUpdate             = false;

}


void Plug_pf_manager::initialise_motion_model(){
    // create motion model
    arma::mat33 motion_noise_cov;
    float st_dev = 0.005;
    motion_noise_cov = motion_noise_cov.eye() * st_dev * st_dev;
    plug_motion_model   = std::unique_ptr<Plug_motion_model>(new Plug_motion_model(motion_noise_cov) );
    motion_model_f      = std::bind(&Plug_motion_model::motion_update,*plug_motion_model,std::placeholders::_1,std::placeholders::_2);

}

void Plug_pf_manager::initalise_likelihood_model(wobj::WrapObject& wrapped_objects,
                                                 likeli::likelihood_type likelihood_t
                                                 ){

    switch(likelihood_t)
    {
    case likeli::THREE_PIN:
    {
        std::cout<< "initialise three pin likelihood model" << std::endl;
        plug_likelihood     = uptr_like_base(new likeli::Plug_likelihood_three_pin_distance(wrapped_objects));
        likeli::Plug_likelihood_three_pin_distance* ptr_lik_three = static_cast<likeli::Plug_likelihood_three_pin_distance*>(plug_likelihood.get());
        likelihood_f = std::bind(&likeli::Plug_likelihood_three_pin_distance::likelihood,*ptr_lik_three,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4);
        break;
    }
    case likeli::SIMPLE_CONTACT:
    {
        std::cout<< "initialise simple contact likelihood model" << std::endl;
        plug_likelihood     = uptr_like_base(new likeli::Plug_likelihood_simple_contact(wrapped_objects));
        likeli::Plug_likelihood_simple_contact* ptr_lik_three = static_cast<likeli::Plug_likelihood_simple_contact*>(plug_likelihood.get());
        likelihood_f = std::bind(&likeli::Plug_likelihood_simple_contact::likelihood,*ptr_lik_three,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4);
        break;
    }
    case likeli::FOUR_CONTACT:
    {
        std::cout<< "initialise four contact likelihood model" << std::endl;
        plug_likelihood     = uptr_like_base(new likeli::Plug_likelihood_four_contact(wrapped_objects));
        likeli::Plug_likelihood_four_contact* ptr_lik_three = static_cast<likeli::Plug_likelihood_four_contact*>(plug_likelihood.get());
        likelihood_f = std::bind(&likeli::Plug_likelihood_four_contact::likelihood,*ptr_lik_three,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4);

        break;
    }
    case likeli::FORCE_IID:
    {

        break;
    }
    default:
    {
        std::cerr<< "NO LIKELIHOOD FUNCTION SET Plug_pf_manager:initialise_likelihood_model" << std::endl;
        break;
    }
    }


}

void Plug_pf_manager::update(const arma::colvec& Y, const arma::colvec &u,const arma::mat33& rot){
        particle_filter->set_rotation(rot);
        particle_filter->update(u,Y);
}

void Plug_pf_manager::init_visualise(ros::NodeHandle &node){
    particle_filter->init_visualise(node,"pfilter");
    particle_filter->set_visualisation_mode(viz_mode);
    particle_filter->set_color_mode(color_t);

}

void Plug_pf_manager::visualise(){
    if(particle_filter != NULL){
        particle_filter->visualise();
    }
}


//const arma::colvec3& T = arma::colvec3()
void Plug_pf_manager::initialise_prior_pdf(const arma::colvec3 &Plug_position){

    if(particle_filter_type == HIST){

        arma::mat points_sparse;
        arma::mat points_dense;
        arma::colvec3 origin = {{0.2,0.0,0}};
        arma::vec orient = {{0,0,0}};

        origin(2) = Plug_position(2);

        float length = 0.2;
        float width  = 1.2;
        float height = 0.05;
        arma::colvec3 rec_dim = {{length,width,height}};

        origin(0) = origin(0) + length/2;

        if(arma::sum(Plug_position) != 0){
            update_center_rectangle(Plug_position,origin,rec_dim);
        }

        pf::Static_grid_filter::create_cube(points_sparse,rec_dim(0),rec_dim(1),rec_dim(2),0.02,origin,orient);

        length = 0.05;
        width  = 0.15;
        height = 0.15;
        origin.zeros();
        origin(0) = origin(0) + 0.025;
        pf::Static_grid_filter::create_cube(points_dense,length,width,height,0.0025,origin,orient);

        arma::mat points = points_dense;// arma::join_vert(points_sparse,points_dense);

        particle_filter->reinitialise(points);
       // particle_filter->print();

    }else{


        tf::Matrix3x3 rot;
        rot.setEulerYPR(0,0,0);

        arma::vec3  origin = {{0.3,0.0,0}};
        origin(2) = Plug_position(2);

        arma::mat   orientation(3,3);
        double      length = 0.2;
        double      width  = 1.2;
        double      height = 0.05;
        arma::colvec3 rec_dim = {{length,width,height}};

        tf2mat(rot,orientation);

        arma::mat& particles   = particle_filter->particles;

        if(arma::sum(Plug_position) != 0){
            update_center_rectangle(Plug_position,origin,rec_dim);
        }

        uniform = Uniform(origin,orientation,rec_dim(0),rec_dim(1),rec_dim(2));
        arma::vec x;
        for(std::size_t i = 0; i < particle_filter->particles.n_rows;i++){
            x = uniform.sample();
            particles(i,0) = x(0);
            particles(i,1) = x(1);
            particles(i,2) = x(2);
        }

        particle_filter->reinitialise(particles);
        particle_filter->reset_weights();
   }

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
