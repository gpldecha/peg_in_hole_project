#ifndef PEG_MOTION_MODEL_H_
#define PEG_MOTION_MODEL_H_

#include <armadillo>
#include <statistics/distributions/gaussian.h>


class Plug_motion_model{

public:

    Plug_motion_model(const arma::mat33& noise_cov);

    void motion_update(arma::mat& X,const arma::colvec3& u);

private:

    arma::mat33            noise_cov; // Covariance of motion noise
    arma::mat33            A;
    arma::colvec3          z;

};

#endif
