#include <peg_filter/pf_manager.h>
#include <optitrack_rviz/type_conversion.h>
#include <optitrack_rviz/listener.h>

int init_pmf_type = 3;

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




Plug_pf_manager::Plug_pf_manager(){}

void Plug_pf_manager::add(pf::Base_particle_filter* base_particle_filter,const std::string& name){
    filter_methods[name] = base_particle_filter;
}

bool Plug_pf_manager::select_method(const std::string& name){
    it_filter_method = filter_methods.find(name);
    if(it_filter_method != filter_methods.end())
    {
        return true;
    }else{
        return false;
    }
}

void Plug_pf_manager::update(const arma::colvec& Y, const arma::colvec& u,const arma::mat33& rot, const double duration_s){
    if(it_filter_method != filter_methods.end()){
        (it_filter_method->second)->set_rotation(rot);
        (it_filter_method->second)->update(u,Y);

    }else{
        ROS_ERROR("it_filter_method == filter_methods.end()");
    }
}

void Plug_pf_manager::init_visualise(ros::NodeHandle &node){

    if(it_filter_method != filter_methods.end()){
        (it_filter_method->second)->init_visualise(node,"pfilter");
    }else{
        ROS_ERROR("it_filter_method == filter_methods.end() init_visualise");
    }
}

void Plug_pf_manager::visualise(){

    if(it_filter_method != filter_methods.end()){
        (it_filter_method->second)->visualise();
    }else{
        ROS_ERROR("it_filter_method == filter_methods.end() visualise");
    }
}


void Plug_pf_manager::set_visualise_mode(opti_rviz::Vis_point_cloud::display_mode mode){
    if(it_filter_method != filter_methods.end()){
        (it_filter_method->second)->set_visualisation_mode(mode);
    }else{
        ROS_ERROR("set_visualise_mode == filter_methods.end() visualise");
    }
}

void Plug_pf_manager::set_pf_color_type(pf::color_type color_t){
    if(it_filter_method != filter_methods.end()){
        (it_filter_method->second)->set_color_mode(color_t);
    }else{
        ROS_ERROR("set_pf_color_type == filter_methods.end() visualise");
    }
}

}
