#include "peg_filter/likelihood.h"
#include <functional>
#include "statistics/distributions/distributions.h"

namespace likeli{

Gaussian_likelihood::Gaussian_likelihood(const arma::colvec& variance)
{

    // (1 x M), where M is the dimension of Y
    one_div_var = -0.5 * (1.0 / variance);

}

void Gaussian_likelihood::gaussian_likelihood(double *L, const arma::colvec &Y, const arma::mat &hY){
    for(std::size_t i = 0; i < hY.n_rows;i++){

        L[i] =     exp(  one_div_var(0) * (hY(i,0) - Y(0)) * (hY(i,0) - Y(0))
                         + one_div_var(1) * (hY(i,1) - Y(1)) * (hY(i,1) - Y(1))
                         + one_div_var(2) * (hY(i,2) - Y(2)) * (hY(i,2) - Y(2))
                         );

    }
}

Binary_likelihood::Binary_likelihood(){
    ptr_delta_ == NULL;
}

void Binary_likelihood::set_resolution(const pf::Point_mass_filter::delta *delta_){
    ptr_delta_ = delta_;
    const pf::Point_mass_filter::delta& delta__ = *ptr_delta_;
    range = std::sqrt(delta__.k*delta__.k + delta__.m*delta__.m + delta__.n * delta__.n);
}

bool Binary_likelihood::set_sense(double Y){
    if((int)Y == 0){
        return false;
    }else{
        return true;
    }
}

double Binary_likelihood::binary_surf_likelihood(const double dist_surface,const double range,bool bSense_SURF){
    if(bSense_SURF)
    {
        if(dist_surface <= range + 0.01)
        {
            return 1;
        }else{
            return 0;
        }
    }else{
        if(dist_surface <= 0.001)
        {
            return 0;
        }else{
            return 1;
        }
    }
}

double Binary_likelihood::binary_edge_likelihood(const double dist_edge, const double range,bool bSense_EDGE){
    if(bSense_EDGE)
    {
        if(dist_edge <=  range + 0.02)
        {
            return 1;
        }else{
            return 0;//exp(-1000.0 * (dist_edge-range) * (dist_edge-range) );
        }
    }else{

       /* if(dist_edge <= 0.005){
            return 0;
         }else{
            return 1.0;// - exp(-1000.0 * (dist_edge-range) * (dist_edge-range) );
        }*/
        return 1;
    }
}



void Binary_likelihood::likelihood(double *L, const arma::colvec &Y, const arma::mat &hY){

   // std::cout<< "==== Y = " << Y(0) << "   ===== " << std::endl;
    bool bSense_SURF, bSense_EDGE, bSense_SOCKET;

    bSense_SURF     = set_sense(Y(0));
    bSense_EDGE     = set_sense(Y(1));
   // bSense_SOCKET   = set_sense(Y(2));

    double sum_L = 0;

    if(ptr_delta_ == NULL)
    {
        ROS_ERROR("ptr_delta_ == NULL, Binary_likelihood::likelihood");
        return;
    }

    const pf::Point_mass_filter::delta& delta__ = *ptr_delta_;
    range = std::sqrt(delta__.k*delta__.k + delta__.m*delta__.m + delta__.n * delta__.n);
    ROS_INFO_STREAM_THROTTLE(1.0,"range:                " << range);
    ROS_INFO_STREAM_THROTTLE(1.0,"Y:   " << Y(0) << " " << Y(1) << " " << Y(2));



    for(std::size_t i = 0; i < hY.n_rows;i++){

        // if point is index table or socket (but still ok if inside hole boxes)
        if(hY(i,0) == -1)
        {
            L[i] = 0;
        }else{

           L[i] = binary_surf_likelihood(hY(i,0),range,bSense_SURF) * binary_edge_likelihood(hY(i,1),range,bSense_EDGE);
           sum_L = sum_L + L[i];

            if(std::isnan(L[i])){
                std::cout<< "L has NAN" << std::endl;
                exit(0);
            }
        }

    }

    if(sum_L == 0){
        ROS_INFO("sum_L : 0");
        for(std::size_t i = 0; i < hY.n_rows;i++){
            L[i] = 1;
        }
    }

}

}
