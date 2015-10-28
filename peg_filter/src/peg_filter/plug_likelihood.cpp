#include <peg_filter/plug_likelihood.h>

namespace likeli{

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
    std::cout<< "Y : (" << Y.n_cols << " x 1)" << std::endl;

    for(std::size_t i = 0; i < X.n_rows;i++){

        three_pin_distance_mode.update(hY,X.row(i).st(),Rot);

        convert(Y,hY);

        sum_range = 0;
        sum_angle = 0;
        for(std::size_t j = 0; j < 3;j++){

            sum_range = sum_range  + (surf_norm[j] - surf_norm_h[j]) * (surf_norm[j] - surf_norm_h[j])
                    + (edge_norm[j] - edge_norm_h[j]) * (edge_norm[j] - edge_norm_h[j]);

            /* std::cout << " ===== X(" << i << ") === " << std::endl;

            surf_vector[j].print("surf_vector[" + boost::lexical_cast<std::string>(j) + "]");
            surf_vector_h[j].print("surf_vector_h[" + boost::lexical_cast<std::string>(j) + "]");
            std::cout<< "n surf_vector["<<j<<"]: " << arma::norm(surf_vector[j]) << std::endl;
            std::cout<< "n surf_vector_h["<<j<<"]: " << arma::norm(surf_vector_h[j]) << std::endl;
            std::cout<< "dot:  " << arma::dot(surf_vector[j],surf_vector_h[j]) << std::endl;
            std::cout<< "acos: "<< std::acos( arma::dot(surf_vector[j],surf_vector_h[j])) << std::endl;*/

            sum_angle = sum_angle +  std::acos(arma::dot(surf_vector[j],surf_vector_h[j]))
                    +  std::acos(arma::dot(edge_vector[j],edge_vector_h[j]));

            /* std::cout<<  "surf_norm["<< j<<"]:   " << surf_norm[j] <<std::endl;
            std::cout<<  "surf_norm_h["<< j<<"]: " << surf_norm_h[j] <<std::endl;
            std::cout<<  "edge_norm["<< j<<"]:   " << edge_norm[i] <<std::endl;
            std::cout<<  "edge_norm_h["<< j<<"]: " << edge_norm_h[i] <<std::endl;*/


            /* L(i) = L(i) - angle_noise * std::acos(arma::dot(surf_vector[j],surf_vector_h[j]))
                      - angle_noise * std::acos(arma::dot(edge_vector[j],edge_vector_h[j]))
                      - range_noise * (surf_norm[j] - surf_norm_h[j]) * (surf_norm[j] - surf_norm_h[j])
                      - range_noise * (edge_norm[j] - edge_norm_h[j]) * (edge_norm[j] - edge_norm_h[j]);*/
        }
        // std::cout<< "sum_angle("<<i<< "): " << sum_angle << std::endl;
        // std::cout<< "sum_range("<<i<< "): " << sum_range << std::endl;

        // L(i) =  exp(- 0.5 * one_div_var_angle * sum_angle -0.5 * one_div_var_range * sum_range);
        L(i) =  - 0.5 * one_div_var_angle * sum_angle - 0.5 * one_div_var_range * sum_range; // log likelihood
        //  L(i) =  - 0.5 * one_div_var_range * sum_range;
        //   std::cout<< "L(" << i << "): " << L(i) << std::endl;

        //

    }


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


Plug_likelihood_simple_contact::Plug_likelihood_simple_contact(wobj::WrapObject& wrap_object):
    contact_distance_model(wrap_object)
{

    surf =     psm::Contact_distance_model::C_SURF;
    edge =     psm::Contact_distance_model::C_EDGE;

    sd = 0.005;
    min_half_one_div_var = - 0.5 * 1.0 / (sd * sd);

}

void Plug_likelihood_simple_contact::likelihood(      arma::colvec &L,
                                                      const arma::colvec& Y,
                                                      const arma::mat&    X,
                                                      const arma::mat33& Rot){

    hY.resize(2);
    for(std::size_t i = 0; i < X.n_rows;i++){
        contact_distance_model.update(hY,X.row(i).st(),Rot);
        L(i) = exp(min_half_one_div_var * (  (hY(surf) - Y(surf)) * (hY(surf) - Y(surf))
                                             + (hY(edge) - Y(edge)) * (hY(edge) - Y(edge)))
                   );

    }
}


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

