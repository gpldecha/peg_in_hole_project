#include "peg_filter/likelihood.h"
#include <functional>
#include "statistics/distributions/distributions.h"
#include "peg_sensor/peg_sensor_model/distance_model.h"
#include <optitrack_rviz/type_conversion.h>
#include <optitrack_rviz/debug.h>
#include <limits>

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


/// ----------------------------  Mixded Likelihood -----------------------------------------

Mixed_likelihood::Mixed_likelihood(ros::NodeHandle& nh, wobj::WrapObject &wrapped_world, const arma::colvec3 &peg_origin)
    :wrapped_world(wrapped_world),
      wall_box(             wrapped_world.get_wbox("link_wall")             ),
      socket_box(           wrapped_world.get_wbox("wbox_socket")           ),
      box_h1(               wrapped_world.get_wbox("box_hole_1")            ),
      box_h2(               wrapped_world.get_wbox("box_hole_2")            ),
      box_h3(               wrapped_world.get_wbox("box_hole_3")            ),
      socket_top_edge(      wrapped_world.get_wbox("socket_top_edge")       ),
      socket_bottom_edge(   wrapped_world.get_wbox("socket_bottom_edge")    ),
      socket_left_edge(     wrapped_world.get_wbox("socket_left_edge")      ),
      socket_right_edge(    wrapped_world.get_wbox("socket_right_edge")     ),
      wsocket(wrapped_world.wsocket),
      vis_vectors(nh,"force_vector"),
      peg_origin(peg_origin)
{
    ptr_delta_ == NULL;
    Rot_tmp.setRPY(0,-M_PI/2,M_PI);
    opti_rviz::type_conv::tf2mat(Rot_tmp,rot_tmp);

    arrows.resize(1);
    arrows[0] = opti_rviz::Arrow(arrow_origin,arrow_direction,"force_vector");
    arrows[0].set_rgba(0, 0, 1, 1);
    arrows[0].set_scale(0.005, 0.005, 0.005);

    vis_vectors.initialise("world",arrows);


}

void Mixed_likelihood::set_resolution(const pf::Point_mass_filter::delta* delta_){
    ptr_delta_ = delta_;
    const pf::Point_mass_filter::delta& delta__ = *ptr_delta_;
    range = std::sqrt(delta__.k*delta__.k + delta__.m*delta__.m + delta__.n * delta__.n);
}

void Mixed_likelihood::likelihood(double *L, const arma::colvec &Y, const arma::mat &X, const arma::mat33& Rot){
    /*
 *  Y : Measurement from the robot.
 *
 *  Y(0) : Contact / No contact
 *
 *  Y(1) : Fx
 *  Y(2) : Fy
 *  Y(3) : Fz
 *
 *  Y(4) : prob top   contact
 *  Y(5) : prob bot   contact
 *  Y(6) : prob left  contact
 *  Y(7) : prob right contact
 *
 * */





    /*
     *
     *  C_SURF      =0
     *  C_EDGE_DIST =1
     *
     *  C_EDGE_LEFT =2
     *  C_EDGE_RIGHT=3
     *  C_EDGE_TOP  =4
     *  C_EDGE_BOT  =5
     *
     *  C_RING      =6
     *  C_S_HOLE    =7
     *  C_SOCKET    =8
     *  C_EDGE_V1   =9
     *  C_EDGE_V2   =10
     *  C_EDGE_V3  = 11
     */


    double sum_L         = 0;
    bool bSense_SURF     = set_sense(Y(0));
    bool bSense_edge     = false;

    /*    Y_edge(0) = Y(6);
    Y_edge(1) = Y(7);
    Y_edge(2) = Y(8);

    double Y_dist_edge  = arma::norm(Y_edge);
    Y_edge              = Y_edge / (Y_dist_edge  + std::numeric_limits<double>::min());
    double hY_dist_edge = 0;


    if(Y_dist_edge < 0.01){
        bSense_edge = true;
    }


    if(bSense_edge){
        ROS_INFO_STREAM_THROTTLE(0.5,"bSense_edge: TRUE");
    }else{
        ROS_INFO_STREAM_THROTTLE(0.5,"bSense_edge: FALSE");
    }
*/

    double lik_one;


    geo::fCVec3 origin_i;
    geo::fCVec3 vec_peg; vec_peg.zeros();
    geo::fCVec3 forward_FR_proj;
    vec_peg(0) = 0.015;

    rot = arma::conv_to<arma::fmat>::from(Rot);

    F(0) = Y(1)/6;
    F(1) = Y(2)/6;
    F(2) = Y(3)/6;

    F_n         = F;
    force_norm  = arma::norm(F_n);
    F_n         = F_n / (force_norm + std::numeric_limits<double>::min());


    if( sqrt(F(1) * F(1) + F(2)* F(2)) > 0.5 )
    {
        bSense_edge = true;
        ROS_INFO_STREAM_THROTTLE(1.0,"------> bSense_edge");
    }


    arrow_direction[0]  = F(0);
    arrow_direction[1]  = F(1);
    arrow_direction[2]  = F(2);



    arrow_direction =  0.05 * (arrow_direction);


    opti_rviz::type_conv::vec2tf(peg_origin,arrow_origin);

    arrows[0].set_pos_dir(arrow_origin,arrow_direction);
    vis_vectors.update(arrows);
    vis_vectors.publish();



    if(Y(6) > 0.8)
    {
        std::cout<< "TOP EDGE " << std::endl;
    }


    for(std::size_t i = 0; i < X.n_rows;i++){

        lik_one = 1;


        /// Check Contact no Contact.

        point_i        =  arma::conv_to<arma::fmat>::from(X.row(i).st());
        forward_FR_proj = (rot * rot_tmp * vec_peg) + point_i;

        if(i == 0){
            opti_rviz::debug::tf_debuf(forward_FR_proj,"forward_FR_proj");
        }


        wall_box.distance_to_features(forward_FR_proj);

       /* if(wall_box.is_inside())
        {
            L[i] = 0;

        }else{*/


            /// Check Circle

            wsocket.distance_to_features(point_i);

            plat_dir        = wsocket.plate.C - wsocket.get_surface_projection();
            plat_dir_radius = arma::norm(plat_dir);
            plat_dir_n      = plat_dir/(plat_dir_radius + std::numeric_limits<double>::min());

            center_dir      = wsocket.plate.C - point_i;
            dist_center     = std::sqrt(center_dir(1) *center_dir(1) +  center_dir(2) *center_dir(2));


            // ROS_INFO_STREAM_THROTTLE(0.1,"force_norm: " << force_norm);
            ROS_INFO_STREAM_THROTTLE(0.1,"plat_dir_radius:      " << plat_dir_radius);


           // if(force_norm > 0.6 && sgmf( plat_dir_n(1) * F_n(1) + plat_dir_n(2) * F_n(2) +1,4,1) < 0.1){
           //     lik_one=0;
           // }

            // force cirlce

            in_circle=false;
            if(bSense_edge && dist_center < 0.015 && dist_center > 0.025){
                in_circle=true;
            }


            /// Check Block Edges


            if(Y(6) > 0.2) // prob top   contact
            {
                // check if point is inside block top
                socket_top_edge.distance_to_features(point_i);
                if(socket_top_edge.is_inside())
                {
                    in_ts=true;
                }else{
                    in_bs=false;
                }
            }

            if(Y(7) > 0.5){ // prob bottom   contact

                // check if point is bottom block top
                socket_bottom_edge.distance_to_features(point_i);
                if(socket_bottom_edge.is_inside())
                {
                    in_bs=true;
                }else{
                    in_bs=false;
                }
            }

            if(Y(5) > 0.5) // prob left  contact
            {
                // check if point is in left block
                socket_left_edge.distance_to_features(point_i);
                if(socket_left_edge.is_inside())
                {
                    in_ls=true;
                }else{
                    in_ls=false;
                }
            }

            if(Y(4) > 0.5) // prob right  contact
            {
                socket_right_edge.distance_to_features(point_i);
                if(socket_right_edge.is_inside())
                {
                    in_rs=true;
                }else{
                    in_rs=false;
                }
            }

            // if I sense, I should either have

            if(bSense_edge)
            {
                // if an edge is sensed I should be in all of these regions.
                if(in_circle ||  in_rs || in_ls || in_ts || in_bs)
                {
                    lik_one = 1;
                }else{
                    lik_one = 0;
                }


            }




            L[i] = lik_one;
       // }
        sum_L = sum_L + L[i];

        /*  hY_edge(0)   = hY(i,psm::Contact_distance_model::C_EDGE_V1);
          hY_edge(1)   = hY(i,psm::Contact_distance_model::C_EDGE_V2);
          hY_edge(2)   = hY(i,psm::Contact_distance_model::C_EDGE_V3);
          hY_dist_edge = arma::norm(hY_edge);
          hY_edge      = hY_edge / (hY_dist_edge + std::numeric_limits<double>::min());*/

        // if point is index table or socket (but still ok if inside hole boxes)
        /*   if(hY(i,0) == -1)
        {
            L[i] = 0;
        }else{*/

        /*  lik_one = binary_surf_likelihood(hY(i,0),range,bSense_SURF);

            if(bSense_edge)
            {

                if          (Y(5) == 1 && Y(4) == 0 && Y(3) == 0 && Y(2) == 0)
                {
                    lik_one =  lik_one * binary_likelihood(hY(i,5),Y(5));

                }else if    (Y(5) == 0 && Y(4) == 1 && Y(3) == 0 && Y(2) == 0){

                    lik_one =  lik_one * binary_likelihood(hY(i,4),Y(4));

                }else if    (Y(5) == 0 && Y(4) == 0 && Y(3) == 1 && Y(2) == 0){

                    lik_one =  lik_one * binary_likelihood(hY(i,3),Y(3));

                }else if    (Y(5) == 0 && Y(4) == 0 && Y(3) == 0 && Y(2) == 1){

                    lik_one =  lik_one * binary_likelihood(hY(i,2),Y(2));
                }

               // if(sgmf(arma::dot(hY_edge,Y_edge)+1,4,1) < 0.2){
               //     lik_one = 0;
//               /  }

            }else{
                // if we do not sense the edge, anything that is super close to edge is set to zero.
               if(hY_dist_edge < 0.0001)
                {
                    lik_one =  0;
                }
            }

            L[i]  = lik_one;
            sum_L = sum_L + L[i];

            if(std::isnan(L[i])){
                std::cout<< "L has NAN" << std::endl;
                exit(0);
            }
       // }*/


    }

    if(sum_L == 0){
        ROS_INFO_STREAM_THROTTLE(1.0,"-----------__>sum_L : 0");
        for(std::size_t i = 0; i < X.n_rows;i++){
            L[i] = 1;
        }
    }else{

        // normalise likelihood

        for(std::size_t i = 0; i < X.n_rows;i++){
            L[i] = L[i] / sum_L;
        }
    }

}

/// ----------------------------  Binary Likelihood -----------------------------------------


Binary_likelihood::Binary_likelihood(){
    ptr_delta_ == NULL;
}

void Binary_likelihood::set_resolution(const pf::Point_mass_filter::delta *delta_){
    ptr_delta_ = delta_;
    const pf::Point_mass_filter::delta& delta__ = *ptr_delta_;
    range = std::sqrt(delta__.k*delta__.k + delta__.m*delta__.m + delta__.n * delta__.n);
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
