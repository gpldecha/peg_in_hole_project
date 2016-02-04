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

    /* assert(L.n_elem == hY.n_elem);
    double value = 0;
    double one_var = (1.0/(0.02*0.02));
    //Y.print("Y");

    bool bSense;
    if(Y(0) >= 0.5){
        bSense = true;
    }else{
        bSense = false;
    }
   // std::cout<< "bSense: " << bSense << std::endl;

    for(std::size_t i = 0; i < hY.n_rows;i++){


        if(!bSense){     /// Case when there is no sensing dist >= 0.02 [m]

           // L(i) = 1 - exp(-0.5 * one_var * (hY(i,0) - Y(0)) * (hY(i,0) - Y(0)) );
            L(i) = 1 - exp(-0.5 * one_var * hY(i,0)*hY(i,0));
           // std::cout<< "no sense" << std::endl;
       }else{          /// Case when there is sensing
            L(i) =     exp(  one_div_var(0) * (hY(i,0) - Y(0)) * (hY(i,0) - Y(0))
                            + one_div_var(1) * (hY(i,1) - Y(1)) * (hY(i,1) - Y(1))
                            + one_div_var(2) * (hY(i,2) - Y(2)) * (hY(i,2) - Y(2))
                             );



      }

       if(hY(i,0) == -1){
            L(i) = 0;
        }

        //  if(Y(0) == 1){

      //

        /// Case when the peg tip is very close to the surface <= 0.02 [m]
     //   }else{
       //     L(i) =  exp(-0.5 * one_var * (hY(i,0) - Y(0)) * (hY(i,0) - Y(0))  );
      //  }
      //  L(i) = 1 - exp(-0.5 * one_var* hY(i,0));

    }

    if(arma::sum(L) == 0){
        L = arma::ones(L.n_elem);
    }

    L = L / (arma::sum(L)) ;

    if(L.has_nan()){
       std::cout<< "L has NAN" << std::endl;
        exit(0);
    }*/

}


void Hellinger_likelihood::likelihood(double *L, const arma::colvec &Y, const arma::mat &hY){


    // hY are distances to closest featuers


   // std::cout<< "==== Y = " << Y(0) << "   ===== " << std::endl;
    bool bSense;
    if((int)Y(0) == 0){
        bSense = false;
    }else{
        bSense = true;
    }
 //   std::cout<< "=== bSense (" << bSense << ") ===" << std::endl;

    for(std::size_t i = 0; i < hY.n_rows;i++){

        if(hY(i,0) == -1)
        {
            L[i] = 0;

        }else{

            // Y(0) == 1
            if(bSense)
            {
                if(hY(i,0) <= 0.03)
                {
                    L[i] = 1;
                }else{
                    L[i] = 0;
                }
            }else{

                if(hY(i,0) < 0.01)
                {
                    L[i] = 0;
                }else{
                    L[i] = 1;
                }

            }


            // P = hY.row(i).st();
            // P = P / (arma::norm(P,2) + std::numeric_limits<double>::min() );

            /*     if( std::fabs(Y[i] - hY(i,0)) < 0.01)
            {
                L[i] = 0.0;
            }else{
                L[i] = 1.0;
            }*/


            /* L[i] =     exp( -(1.0/0.1) *   ((hY(i,0) - Y(0)) * (hY(i,0) - Y(0))
                                    + (hY(i,1) - Y(1)) * (hY(i,1) - Y(1))
                                    + (hY(i,2) - Y(2)) * (hY(i,2) - Y(2)))
                            );*/

            // sum_L = sum_L + L[i];

            if(std::isnan(L[i])){
                std::cout<< "L has NAN" << std::endl;
                exit(0);
            }

            //            L(i) = stats::Distance::Hellinger(Q,P);


            /*if(arma::sum(arma::abs(hY.row(i).st() - Y)) != 0){
                L(i) = 0;
            }else{
                L(i) = 1;
            }*/
        }

    }


    /*if(sum_L == 0){
        double val = 1.0/static_cast<double>(hY.n_rows);
        for(std::size_t i = 0; i < hY.n_rows;i++){
            L[i] = val;
        }
    }*/


}


}

/*
Plug_likelihood_three_pin_distance::Plug_likelihood_three_pin_distance(wobj::WrapObject& wrap_object)
    :three_pin_distance_mode(wrap_object)
{
    scale = 100;
    double three_sd_range = 5.0;
    double three_sd_angle = M_PI/4;
    one_div_var_range = 1.0/(three_sd_range * three_sd_range);//((three_sd_range/3.0) * (three_sd_range/3.0));
    one_div_var_angle = 1.0/(three_sd_angle * three_sd_angle);
}

void Plug_likelihood_three_pin_distance::likelihood(arma::colvec &L, const arma::colvec& Y, const arma::mat& X,const arma::mat33& Rot){

    hY.resize(2 * 3 * 3);

    std::cout<< " Plug_likelihood_three_pin_distance::likelihood" << std::endl;
    std::cout<< "L : (" << L.n_elem << " x 1)" << std::endl;
    std::cout<< "X : (" << X.n_rows << " x " << X.n_cols <<  ") " << std::endl;
    std::cout<< "Y : (" << Y.n_elem << " x 1)" << std::endl;

    for(std::size_t i = 0; i < X.n_rows;i++){

        std::cout<< "i: " <<i << std::endl;
        three_pin_distance_mode.update(hY,X.row(i).st(),Rot);

        std::cout<< "convert" << std::endl;
        convert(Y,hY);

        sum_range = 0;
        sum_angle = 0;
        for(std::size_t j = 0; j < 3;j++){

            sum_range = sum_range  + (surf_norm[j] - surf_norm_h[j]) * (surf_norm[j] - surf_norm_h[j])
                    + (edge_norm[j] - edge_norm_h[j]) * (edge_norm[j] - edge_norm_h[j]);



            sum_angle = sum_angle +  std::acos(arma::dot(surf_vector[j],surf_vector_h[j]))
                    +  std::acos(arma::dot(edge_vector[j],edge_vector_h[j]));

        }
        // std::cout<< "sum_angle("<<i<< "): " << sum_angle << std::endl;
        // std::cout<< "sum_range("<<i<< "): " << sum_range << std::endl;

        // L(i) =  exp(- 0.5 * one_div_var_angle * sum_angle -0.5 * one_div_var_range * sum_range);
        L(i) =  - 0.5 * one_div_var_angle * sum_angle - 0.5 * one_div_var_range * sum_range; // log likelihood
        //  L(i) =  - 0.5 * one_div_var_range * sum_range;
        //   std::cout<< "L(" << i << "): " << L(i) << std::endl;

        //

    }

    std::cout<< "nearly at end" << std::endl;
    double largest_LL = arma::max(L);
    double numerical_precision = log(0.000001) - largest_LL;
    for(std::size_t i = 0; i < L.n_elem;i++){
        if(L(i)-largest_LL >= numerical_precision){
            L(i) =  exp(L(i)-largest_LL);
        }else{
            L(i) = 0;
        }
    }
}
*/

/*
Gaussian_noise_likelihood::Gaussian_noise_likelihood(wobj::WrapObject& wrap_object):
    contact_distance_model(wrap_object)
{

    surf =     psm::Contact_distance_model::C_SURF;
    edge =     psm::Contact_distance_model::C_EDGE;

    sd = 0.005;
    min_half_one_div_var = - 0.5 * 1.0 / (sd * sd);

}

void Gaussian_noise_likelihood::likelihood(      arma::colvec &L,
                                                      const arma::colvec& Y,
                                                      const arma::mat&    X,
                                                      const arma::mat33& Rot){

    hY.resize(2);

    assert(hY.n_elem == Y.n_elem);

    for(std::size_t i = 0; i < X.n_rows;i++){
        contact_distance_model.update(hY,X.row(i).st(),Rot);
        if(hY(0) == -1){
            L(i) = 0;
        }else{
            L(i) = exp(min_half_one_div_var * (  (hY(surf) - Y(surf)) * (hY(surf) - Y(surf))
                                                 + (hY(edge) - Y(edge)) * (hY(edge) - Y(edge)))
                       );
        }
    }
}
*/

/*
Plug_likelihood_four_contact::Plug_likelihood_four_contact(wobj::WrapObject& wrap_object):
    four_distance_model(wrap_object)

{

    surf =     psm::Contact_distance_model::C_SURF;
    edge =     psm::Contact_distance_model::C_EDGE;
    sd = 0.001;

    double sd_angle = M_PI/4;
    min_half_one_div_var        = - 0.5 * 1.0 / (sd * sd);
    min_half_one_div_var_angle  = 1.0/(sd_angle * sd_angle);


}

void Plug_likelihood_four_contact::likelihood(arma::colvec &L, const arma::colvec& Y, const arma::mat& X, const arma::mat33& Rot){

    hY.resize(5);
    double sum_dist  = 0;
    double angle     = 0;

    for(std::size_t i = 0; i < X.n_rows;i++){
        four_distance_model.update(hY,X.row(i).st(),Rot);
        if(hY(0) < 0){
            L(i) = 0;
        }else{
            sum_dist =  (hY(surf) - Y(surf)) * (hY(surf) - Y(surf)) + (hY(edge) - Y(edge)) * (hY(edge) - Y(edge));
            angle    =  std::acos(hY(2)*Y(2) + hY(3)*Y(3) + hY(4)*Y(4));
            L(i)     = exp(min_half_one_div_var * sum_dist);// +  min_half_one_div_var_angle * angle);
        }
    }

}



Plug_likelihood_force_iid::Plug_likelihood_force_iid(wobj::WrapObject &wrap_object){
    //:{

}
*/
/*
void Plug_likelihood_force_iid::likelihood(arma::colvec &L, const arma::colvec& Y, const arma::mat& X, const arma::mat33& Rot){

    for(std::size_t i = 0; i < X.n_rows;i++){

        if(Y(0) == 1){

        }else{

        }

    }

}



Plug_likelihood_preset::Plug_likelihood_preset()
    :Plug_likelihood_base()
{

}

void Plug_likelihood_preset::likelihood(arma::colvec &L, const arma::colvec& Y, const arma::mat& X, const arma::mat33& Rot){




}



}
*/
/*
Plug_likelihood::Plug_likelihood(psm::Plug_base_model& plug_measuremen_model):
    plug_measuremen_model(plug_measuremen_model)
{

    noise_var   = std::pow(0.01/3,2);

}
*/

//                               arma::fcolvec& L, const arma::fcolvec& Y,const arma::fmat& X
/*void Plug_likelihood::likelihood(arma::colvec& L, const arma::colvec& Y, const arma::mat& X){

   // omp_set_dynamic(0);     // Explicitly disable dynamic teams
   // omp_set_num_threads(4);
    //#pragma omp parallel for
    hY.resize(Y.n_elem);
    for(std::size_t i = 0; i < X.n_rows;i++){
        // hY : [-1, 1 - 0]
        plug_measuremen_model.update(hY,X.row(i).st(),R);

        if(hY(0) < 0){
            L(i) = 0;
        }else{
            L(i) = 0;
            for(std::size_t j = 0; j < hY.n_elem;j++){
                 L(i) = L(i) + (hY(j) - Y(j)) * (hY(j) - Y(j)) ;
            }
            L(i) = exp(-0.5 * (1.0/noise_var) * L(i));
        }

       // if(min_distance_surface < 0){
       //     L(i) = 0;
       // }else{
       //     L(i) = lik_contact(Y(0),min_distance_surface,beta_surface);// * lik_edge(Y(1),min_distance_edge,beta_edge);
       // }

    }


}
*/

