#ifndef PEG_FILTER_LIKELIHOOD_H_
#define PEG_FILTER_LIKELIHOOD_H_

#include "peg_sensor/peg_sensor_model/distance_model.h"
#include "peg_sensor/peg_sensor_model/peg_distance_model.h"

/**
 *      Likelihood functions
 *
 *      Base and derived implementations of the pegs sensor likelihood model.
 *
 *      These functions take multivariate sensor input $Y \in R^{n}$ and compare it
 *      with a measurement model $\hat{Y} = h(X)$ where X is the cartesian position of
 *      the peg.
 *
 *      The likelihood of a measurement is then: N(Y - h(X)|0,Sigma), where N is a multivariate
 *      Gaussian function with mean zero and unit covariance.
 */

namespace likeli{

typedef enum {SIMPLE_CONTACT,FOUR_CONTACT,THREE_PIN,FORCE_IID} likelihood_type;

class Plug_likelihood_base{

public:

    virtual void likelihood(arma::colvec &L, const arma::colvec& Y, const arma::mat& X,const arma::mat33& Rot) = 0;

public:

    arma::colvec          hY;

};




class Plug_likelihood_three_pin_distance: public Plug_likelihood_base{

public:

    Plug_likelihood_three_pin_distance(wobj::WrapObject& wrap_object);

    virtual void likelihood(arma::colvec &L, const arma::colvec& Y, const arma::mat& X, const arma::mat33& Rot);

private:

    inline void convert(const arma::colvec& Y,const arma::colvec& hY){

       surf_vector[0] =  Y(arma::span(0,2));
       edge_vector[0] =  Y(arma::span(3,5));

       surf_vector[1] =  Y(arma::span(6,8));
       edge_vector[1] =  Y(arma::span(9,11));

       surf_vector[2] =  Y(arma::span(12,14));
       edge_vector[2] =  Y(arma::span(15,17));

       surf_norm[0]     = arma::norm(surf_vector[0]);
       surf_norm[1]     = arma::norm(surf_vector[1]);
       surf_norm[2]     = arma::norm(surf_vector[2]);

       edge_norm[0]     = arma::norm(edge_vector[0]);
       edge_norm[1]     = arma::norm(edge_vector[1]);
       edge_norm[2]     = arma::norm(edge_vector[2]);

       surf_vector[0]   =   surf_vector[0]/surf_norm[0];
       surf_vector[1]   =   surf_vector[1]/surf_norm[1];
       surf_vector[2]   =   surf_vector[2]/surf_norm[2];

       edge_vector[0]   =   edge_vector[0]/edge_norm[0];
       edge_vector[1]   =   edge_vector[1]/edge_norm[1];
       edge_vector[2]   =   edge_vector[2]/edge_norm[2];


       surf_vector_h[0] =  hY(arma::span(0,2));
       edge_vector_h[0] =  hY(arma::span(3,5));

       surf_vector_h[1] =  hY(arma::span(6,8));
       edge_vector_h[1] =  hY(arma::span(9,11));

       surf_vector_h[2] =  hY(arma::span(12,14));
       edge_vector_h[2] =  hY(arma::span(15,17));

       surf_norm_h[0]   = arma::norm(surf_vector_h[0]);
       surf_norm_h[1]   = arma::norm(surf_vector_h[1]);
       surf_norm_h[2]   = arma::norm(surf_vector_h[2]);

       edge_norm_h[0]   = arma::norm(edge_vector_h[0]);
       edge_norm_h[1]   = arma::norm(edge_vector_h[1]);
       edge_norm_h[2]   = arma::norm(edge_vector_h[2]);

       surf_vector_h[0]   =   surf_vector_h[0]/surf_norm_h[0];
       surf_vector_h[1]   =   surf_vector_h[1]/surf_norm_h[1];
       surf_vector_h[2]   =   surf_vector_h[2]/surf_norm_h[2];

       edge_vector_h[0]   =   edge_vector_h[0]/edge_norm_h[0];
       edge_vector_h[1]   =   edge_vector_h[1]/edge_norm_h[1];
       edge_vector_h[2]   =   edge_vector_h[2]/edge_norm_h[2];

       surf_norm_h[0]    = surf_norm_h[0] * scale;
       surf_norm_h[1]    = surf_norm_h[1] * scale;
       surf_norm_h[2]    = surf_norm_h[2] * scale;

       surf_norm[0]    = surf_norm[0] * scale;
       surf_norm[1]    = surf_norm[1] * scale;
       surf_norm[2]    = surf_norm[2] * scale;

       edge_norm[0]    = edge_norm[0] * scale;
       edge_norm[1]    = edge_norm[1] * scale;
       edge_norm[2]    = edge_norm[2] * scale;

       edge_norm_h[0] = edge_norm_h[0] * scale;
       edge_norm_h[1] = edge_norm_h[1] * scale;
       edge_norm_h[2] = edge_norm_h[2] * scale;


    }

public:

    float           angle_noise;
    float           range_noise;

private:

    psm::Three_pin_distance_model three_pin_distance_mode;

    std::array<arma::colvec3,3>    surf_vector;
    std::array<arma::colvec3,3>    edge_vector;
    std::array<double,3>             surf_norm;
    std::array<double,3>             edge_norm;

    std::array<arma::colvec3,3>    surf_vector_h;
    std::array<arma::colvec3,3>    edge_vector_h;
    std::array<double,3>             surf_norm_h;
    std::array<double,3>             edge_norm_h;
    double                           one_div_var_range;
    double                           one_div_var_angle;
    double                           sum_range;
    double                           sum_angle;
    double                           scale;


};

class Plug_likelihood_simple_contact : public Plug_likelihood_base{

public:

    Plug_likelihood_simple_contact(wobj::WrapObject& wrap_object);

    virtual void likelihood(arma::colvec &L, const arma::colvec& Y, const arma::mat& X, const arma::mat33& Rot);

private:

    psm::Contact_distance_model contact_distance_model;

    int edge;
    int surf;
    double min_half_one_div_var;
    double sd;


};

class Plug_likelihood_four_contact : public Plug_likelihood_base{

public:

    Plug_likelihood_four_contact(wobj::WrapObject& wrap_object);

    virtual void likelihood(arma::colvec &L, const arma::colvec& Y, const arma::mat& X, const arma::mat33& Rot);

private:

    psm::Four_contact_distance_model four_distance_model;

    int edge;
    int surf;
    double min_half_one_div_var;
    double min_half_one_div_var_angle;

    double sd;


};

class Plug_likelihood_force_iid : public Plug_likelihood_base{

public:

    Plug_likelihood_force_iid(wobj::WrapObject& wrap_object);

    virtual void likelihood(arma::colvec &L, const arma::colvec& Y, const arma::mat& X, const arma::mat33& Rot);

private:


};

class Plug_likelihood_preset : public Plug_likelihood_base{

public:

    Plug_likelihood_preset();

    virtual void likelihood(arma::colvec &L, const arma::colvec& Y, const arma::mat& X, const arma::mat33& Rot);

private:

};


}





#endif
