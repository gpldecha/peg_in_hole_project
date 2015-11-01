#include "peg_filter/motion.h"


Plug_motion_model::Plug_motion_model(const arma::mat33& motion_noise)
    :noise_cov(motion_noise)
{
    try{
        A = arma::chol(noise_cov);
    }catch(std::runtime_error e){
        std::cout<< "chol did not work" << std::endl;
        arma::fmat33 I;
        I.eye();
        noise_cov = noise_cov + I * 0.00001;
        A = arma::chol(noise_cov);
    }
}

void Plug_motion_model::motion_update(arma::mat& X, const arma::colvec3 &u){

 //
 /*  noise_cov.print("noise_cov");
   A.print("A");
   z.print("motion noise");
   std::cout<< "||u||: " << arma::norm(u) << std::endl;
   std::cout<< "||z||: " << arma::norm(z) << std::endl;
   */
    for(std::size_t i = 0; i < X.n_rows;i++){
            z = A * arma::randn<arma::fcolvec>(3);
           // X(i,0) = X(i,0) + u(0) + z(0);
           // X(i,1) = X(i,1) + u(1) + z(1);
           // X(i,2) = X(i,2) + u(2) + z(2);
            X(i,0) = X(i,0) + u(0);//s + z(0);
            X(i,1) = X(i,1) + u(1);// + z(1);
            X(i,2) = X(i,2) + u(2);// + z(2);
    }

}
