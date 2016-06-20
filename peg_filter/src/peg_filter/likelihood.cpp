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

    Y_type = Y_TYPE::VIRTUAL;


}

void Mixed_likelihood::set_resolution(const pf::Point_mass_filter::delta* delta_){
    ptr_delta_ = delta_;
    const pf::Point_mass_filter::delta& delta__ = *ptr_delta_;
    range = std::sqrt(delta__.k*delta__.k + delta__.m*delta__.m + delta__.n * delta__.n);
}

void Mixed_likelihood::likelihood(double *L, const arma::colvec &Y, const arma::mat &X, const arma::mat33& Rot){
 /**
 *  Y : Measurement from the robot.
 *
 *  FT Sensor ---------------------
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
 *  FT Virtual Sensor ------------
 *
 *  Y(0) : Contact / No contact
 *  Y(1) : Distance to Edge
 *  Y(2) : Edge_vy
 *  Y(3) : Edge_vz
 *
 *  Y(4) : prob left  contact
 *  Y(5) : prob right contact
 *  Y(6) : prob top   contact
 *  Y(7) : prob bot   contact
 *
 *  Y(8) : bool / is inside socket or not
 *  Y(9) : real : closet distance to ring
 *
 **/


    sum_L              = 0;
    bSense_SURF        = set_sense(Y(0));
    bSense_edge        = false;

    arma::colvec Y_      = Y;

    geo::fCVec3 vec_peg; vec_peg.zeros();
    geo::fCVec3 forward_FR_proj;
    vec_peg(0) = 0.015;

    rot = arma::conv_to<arma::fmat>::from(Rot);



    if (Y_type == Y_TYPE::FT)
    {
        force_yz = std::sqrt(F(1) * F(1) + F(2) * F(2));
        F(0)        = Y(1)/6;
        F(1)        = Y(2)/6;
        F(2)        = Y(3)/6;
        F_n         = F;
        force_norm  = arma::norm(F_n);

        F_n         = F_n / (force_norm + std::numeric_limits<double>::min());
        ROS_INFO_STREAM_THROTTLE(1.0,"------> force_yz:   " << force_yz);
        if( force_yz > 0.5 )
        {
            bSense_edge = true;
        }
        arrow_direction[0]  = F(0);
        arrow_direction[1]  = F(1);
        arrow_direction[2]  = F(2);
        arrow_direction =  0.05 * (arrow_direction);
        opti_rviz::type_conv::vec2tf(peg_origin,arrow_origin);
        arrows[0].set_pos_dir(arrow_origin,arrow_direction);
        vis_vectors.update(arrows);
        vis_vectors.publish();

    }else{

        if(Y_(1) < 0.01)
        {
            bSense_edge = true;
        }
    }



    b_t_c       = false;
    b_b_c       = false;
    b_l_c       = false;
    b_r_c       = false;
    b_insert    = false;
    b_ring      = false;

    if(Y_(4) > 0.2){b_l_c=true;}
    if(Y_(5) > 0.2){b_r_c=true;}
    if(Y_(6) > 0.2){b_t_c=true;}
    if(Y_(7) > 0.2){b_b_c=true;}
    if(Y_(8) == 1){b_insert=true;}
    if(Y_(9) < 0.006){b_ring=true;}


    if(b_insert){
        ROS_INFO_STREAM_THROTTLE(1.0,"b_insert: true");
    }

    average_value = 1.0/static_cast<double>(X.n_rows);




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

        /// Check if close to peg origin

        if( std::sqrt( (peg_origin(1) - point_i(1)) * (peg_origin(1) - point_i(1)) +  (peg_origin(2) - point_i(2)) * (peg_origin(2) - point_i(2)) ) < 0.005)
        {
            close_peg = true;
        }else{
            close_peg = false;
        }

        /// Check Plug Ring Circle

        wsocket.distance_to_features(point_i);

        plat_dir        = wsocket.plate.C - wsocket.get_surface_projection();
        plat_dir_radius = arma::norm(plat_dir);
        plat_dir_n      = plat_dir/(plat_dir_radius + std::numeric_limits<double>::min());

        center_dir      = wsocket.plate.C - point_i;
        dist_center     = std::sqrt(center_dir(1) *center_dir(1) +  center_dir(2) *center_dir(2));
        dist_socket_c   = std::sqrt(center_dir(1) *center_dir(1) +  (center_dir(2) + 0.005) * (center_dir(2) + 0.005 ));

        // if(force_norm > 0.6 && sgmf( plat_dir_n(1) * F_n(1) + plat_dir_n(2) * F_n(2) +1,4,1) < 0.1){
        //     lik_one=0;
        // }

        if(dist_center > 0.015 && dist_center < 0.03){
            in_circle=true;
        }else{
            in_circle=false;
        }

        /// Likelihood inserted
        if(b_insert){
            if(dist_socket_c < 0.005)
            {
                in_s = true;
                //   std::cout<< "in_s: true" << std::endl;
            }else{
                in_s = false;
            }
        }

        /// Check Block Edges

        if(b_t_c) // prob top   contact
        {
            // check if point is inside block top
            socket_top_edge.distance_to_features(point_i);
            if(socket_top_edge.is_inside())
            {
                in_ts=true;
            }else{
                in_ts=false;
            }
        }

        if(b_b_c){ // prob bottom   contact

            // check if point is bottom block top
            socket_bottom_edge.distance_to_features(point_i);
            if(socket_bottom_edge.is_inside())
            {
                in_bs=true;
            }else{
                in_bs=false;
            }
        }

        if(b_l_c) // prob left  contact
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

        if(b_r_c) // prob right  contact
        {
            socket_right_edge.distance_to_features(point_i);
            if(socket_right_edge.is_inside())
            {
                in_rs=true;
            }else{
                in_rs=false;
            }
        }

      /*  bSense_edge = true;
        b_insert    = false;
        b_ring      = false;
        b_l_c       = true;
        b_t_c       = true;
        in_circle   = false;*/


        if(b_insert){

            if(!in_s){ lik_one  = 0;}

        }else if(b_ring){
            if(!in_circle){
                lik_one = 0;
            }
        }else if(bSense_edge){

            if(b_t_c && !b_l_c && !b_r_c  && !b_b_c){            // only    top
                if(!in_ts && (!in_circle)){
                    lik_one = 0;
                }
            }else if (b_l_c && !b_t_c && !b_b_c && !b_r_c){      // only    left
                if(!in_ls && (!in_circle)){
                    lik_one = 0;
                }
            }else if (b_r_c && !b_t_c && !b_b_c && !b_l_c){      // only    right
                if(!in_rs && (!in_circle))
                {
                    lik_one = 0;
                }
            }else if (b_b_c && !b_l_c && !b_r_c && !b_t_c){      // only    bottom
                if(!in_bs  && (!in_circle)){
                    lik_one = 0;
                }
            }else if (b_t_c && b_l_c && !b_r_c  && !b_b_c)       // top     left
            {
                if( (!in_ts) && (!in_ls)  && (!in_circle)){
                    lik_one = 0;
                }
            }else if (b_t_c && b_r_c && !b_l_c && !b_b_c)        // top     right
            {
                if( (!in_ts) && (!in_rs)  && (!in_circle)){
                    lik_one = 0;
                }
            }else if (b_b_c && b_l_c && !b_r_c && !b_t_c){       // bottom  left
                if( (!in_bs) && (!in_ls)  && (!in_circle)){
                    lik_one = 0;
                }
            }else if (b_b_c && b_r_c && !b_l_c && !b_t_c){       // bottom  right
                if( (!in_bs) && (!in_rs) && (!in_circle)){
                    lik_one = 0;
                }
            }else if(!b_t_c && !b_l_c && !b_r_c  && !b_b_c){
                //  if(!in_circle){
                //     lik_one = 0;
                //  }
            }else{
                /// something weired happened

                // if( (!in_bs) && (!in_rs) && (!in_ts) && (!in_ls)){
                //     lik_one = 0;
                // }
            }
        }



        L[i] = lik_one;

        if(close_peg && (L[i] == 0))
        {
            L[i] = 1;
        }


        // }
        sum_L = sum_L + L[i];
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


void Mixed_likelihood::likelihood_stwo(double* L, const arma::colvec& Y, const arma::mat &X, const arma::mat33 &Rot){
    /**
    *  Y : Measurement from the robot.
    *
    *  FT Sensor ---------------------
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
    *  FT Virtual Sensor ------------
    *
    *  Y(0) : Contact / No contact
    *  Y(1) : Distance to Edge
    *  Y(2) : Edge_vy
    *  Y(3) : Edge_vz
    *
    *  Y(4) : prob left  contact
    *  Y(5) : prob right contact
    *  Y(6) : prob top   contact
    *  Y(7) : prob bot   contact
    *
    *  Y(8) : bool / is inside socket or not
    *  Y(9) : real : closet distance to ring
    *
    **/


       sum_L              = 0;
       bSense_SURF        = set_sense(Y(0));
       bSense_edge        = false;

       arma::colvec Y_      = Y;


       geo::fCVec3 vec_peg; vec_peg.zeros();
       geo::fCVec3 forward_FR_proj;
       vec_peg(0) = 0.015;

       rot = arma::conv_to<arma::fmat>::from(Rot);


       if (Y_type == Y_TYPE::FT)
       {
           force_yz = std::sqrt(F(1) * F(1) + F(2) * F(2));
           F(0)        = Y(1)/6;
           F(1)        = Y(2)/6;
           F(2)        = Y(3)/6;
           F_n         = F;
           force_norm  = arma::norm(F_n);

           F_n         = F_n / (force_norm + std::numeric_limits<double>::min());
           ROS_INFO_STREAM_THROTTLE(1.0,"------> force_yz:   " << force_yz);
           if( force_yz > 0.5 )
           {
               bSense_edge = true;
           }
           arrow_direction[0]  = F(0);
           arrow_direction[1]  = F(1);
           arrow_direction[2]  = F(2);
           arrow_direction =  0.05 * (arrow_direction);
           opti_rviz::type_conv::vec2tf(peg_origin,arrow_origin);
           arrows[0].set_pos_dir(arrow_origin,arrow_direction);
           vis_vectors.update(arrows);
           vis_vectors.publish();

       }else{

           if(Y_(1) < 0.01)
           {
               bSense_edge = true;
           }
       }


       bool bdebug = true;

       b_t_c       = false;
       b_b_c       = false;
       b_l_c       = false;
       b_r_c       = false;
       b_insert    = false;
       b_ring      = false;

       if(Y_(4) > 0.2){b_l_c=true;}
       if(Y_(5) > 0.2){b_r_c=true;}
       if(Y_(6) > 0.2){b_t_c=true;}
       if(Y_(7) > 0.2){b_b_c=true;}
       if(Y_(8) == 1){b_insert=true;}
       //if(Y_(9) < 0.006){b_ring=true;}

       if(b_insert){
           ROS_INFO_STREAM_THROTTLE(1.0,"b_insert: true");
       }

       average_value = 1.0/static_cast<double>(X.n_rows);


       for(std::size_t i = 0; i < X.n_rows;i++){

           lik_one = 1;


           /// Check Contact no Contact.

           point_i        =  arma::conv_to<arma::fmat>::from(X.row(i).st());
           forward_FR_proj = (rot * rot_tmp * vec_peg) + point_i;

           if(i == 0){
               opti_rviz::debug::tf_debuf(forward_FR_proj,"forward_FR_proj");
           }


           wall_box.distance_to_features(forward_FR_proj);

           socket_box.distance_to_features(point_i);

           if(wall_box.is_inside())
           {
              lik_one = 0;

           }

           if(socket_box.is_inside())
           {
               lik_one = 0;
           }


           /// Check if close to peg origin

           if( std::sqrt( (peg_origin(1) - point_i(1)) * (peg_origin(1) - point_i(1)) +  (peg_origin(2) - point_i(2)) * (peg_origin(2) - point_i(2)) ) < 0.005)
           {
               close_peg = true;
           }else{
               close_peg = false;
           }

           /// Check Plug Ring Circle

           wsocket.distance_to_features(point_i);

           plat_dir        = wsocket.plate.C - wsocket.get_surface_projection();
           plat_dir_radius = arma::norm(plat_dir);
           plat_dir_n      = plat_dir/(plat_dir_radius + std::numeric_limits<double>::min());

           center_dir      = wsocket.plate.C - point_i;
           dist_center     = std::sqrt(center_dir(1) *center_dir(1) +  center_dir(2) *center_dir(2));
           dist_socket_c   = std::sqrt(center_dir(1) *center_dir(1) +  (center_dir(2) + 0.005) * (center_dir(2) + 0.005 ));

           // if(force_norm > 0.6 && sgmf( plat_dir_n(1) * F_n(1) + plat_dir_n(2) * F_n(2) +1,4,1) < 0.1){
           //     lik_one=0;
           // }

          // if(dist_center > 0.015 && dist_center < 0.03){
          //     in_circle=true;
          // }else{
            in_circle=false;
          // }

           /// Likelihood inserted
           if(b_insert){
               if(dist_socket_c < 0.005)
               {
                   in_s = true;
                   std::cout<< "in_s: true" << std::endl;
               }else{
                   in_s = false;
               }
           }
           /// Check Block Edges

           if(b_t_c) // prob top   contact
           {
               // check if point is inside block top
               socket_top_edge.distance_to_features(point_i);
               if(socket_top_edge.is_inside())
               {
                   in_ts=true;
               }else{
                   in_ts=false;
               }
           }

           if(b_b_c){ // prob bottom   contact

               // check if point is bottom block top
               socket_bottom_edge.distance_to_features(point_i);
               if(socket_bottom_edge.is_inside())
               {
                   in_bs=true;
               }else{
                   in_bs=false;
               }
           }

           if(b_l_c) // prob left  contact
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

           if(b_r_c) // prob right  contact
           {
               socket_right_edge.distance_to_features(point_i);
               if(socket_right_edge.is_inside())
               {
                   in_rs=true;
               }else{
                   in_rs=false;
               }
           }

           if(b_insert){

               if(!in_s){ lik_one  = 0;}

           }else if(b_ring){
               if(!in_circle){
                   lik_one = 0;
               }
           }else if(bSense_edge){

               if(b_t_c && !b_l_c && !b_r_c  && !b_b_c){            // only    top
                   if(!in_ts){
                       lik_one = 0;
                   }
               }else if (b_l_c && !b_t_c && !b_b_c && !b_r_c){      // only    left
                   if(!in_ls){
                       lik_one = 0;
                   }
               }else if (b_r_c && !b_t_c && !b_b_c && !b_l_c){      // only    right
                   if(!in_rs && (!in_circle))
                   {
                       lik_one = 0;
                   }
               }else if (b_b_c && !b_l_c && !b_r_c && !b_t_c){      // only    bottom
                   if(!in_bs){
                       lik_one = 0;
                   }
               }else if (b_t_c && b_l_c && !b_r_c  && !b_b_c)       // top     left
               {
                   if( (!in_ts) && (!in_ls)){
                       lik_one = 0;
                   }
               }else if (b_t_c && b_r_c && !b_l_c)        // top     right
               {
                   if( (!in_ts) && (!in_rs)){
                       lik_one = 0;
                   }
               }else if (b_b_c && b_l_c && !b_r_c && !b_t_c){       // bottom  left
                   if( (!in_bs) && (!in_ls)){
                       lik_one = 0;
                   }
               }else if (b_b_c && b_r_c && !b_l_c && !b_t_c){       // bottom  right
                   if( (!in_bs) && (!in_rs)){
                       lik_one = 0;
                   }
               }else if(!b_t_c && !b_l_c && !b_r_c  && !b_b_c){
                   //  if(!in_circle){
                   //     lik_one = 0;
                   //  }
               }else{
                   /// something weired happened

                   // if( (!in_bs) && (!in_rs) && (!in_ts) && (!in_ls)){
                   //     lik_one = 0;
                   // }
               }
           }



           L[i] = lik_one;

           if(close_peg && (L[i] == 0))
           {
               L[i] = 1;
           }


           // }
           sum_L = sum_L + L[i];
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


void Mixed_likelihood::likelihood_sthree(double* L, const arma::colvec& Y, const arma::mat &X, const arma::mat33 &Rot){
    /**
    *  Y : Measurement from the robot.
    *
    *  FT Sensor ---------------------
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
    *  FT Virtual Sensor ------------
    *
    *  Y(0) : Contact / No contact
    *  Y(1) : Distance to Edge
    *  Y(2) : Edge_vy
    *  Y(3) : Edge_vz
    *
    *  Y(4) : prob left  contact
    *  Y(5) : prob right contact
    *  Y(6) : prob top   contact
    *  Y(7) : prob bot   contact
    *
    *  Y(8) : bool / is inside socket or not
    *  Y(9) : real : closet distance to ring
    *
    **/


       sum_L              = 0;
       bSense_SURF        = set_sense(Y(0));
       bSense_edge        = false;

       arma::colvec Y_      = Y;


       geo::fCVec3 vec_peg; vec_peg.zeros();
       geo::fCVec3 forward_FR_proj;
       vec_peg(0) = 0.015;

       rot = arma::conv_to<arma::fmat>::from(Rot);


       if (Y_type == Y_TYPE::FT)
       {
           force_yz = std::sqrt(F(1) * F(1) + F(2) * F(2));
           F(0)        = Y(1)/6;
           F(1)        = Y(2)/6;
           F(2)        = Y(3)/6;
           F_n         = F;
           force_norm  = arma::norm(F_n);

           F_n         = F_n / (force_norm + std::numeric_limits<double>::min());
           ROS_INFO_STREAM_THROTTLE(1.0,"------> force_yz:   " << force_yz);
           if( force_yz > 0.5 )
           {
               bSense_edge = true;
           }
           arrow_direction[0]  = F(0);
           arrow_direction[1]  = F(1);
           arrow_direction[2]  = F(2);
           arrow_direction =  0.05 * (arrow_direction);
           opti_rviz::type_conv::vec2tf(peg_origin,arrow_origin);
           arrows[0].set_pos_dir(arrow_origin,arrow_direction);
           vis_vectors.update(arrows);
           vis_vectors.publish();

       }else{

           if(Y_(1) < 0.01)
           {
               bSense_edge = true;
           }
       }



       b_t_c       = false;
       b_b_c       = false;
       b_l_c       = false;
       b_r_c       = false;
       b_insert    = false;
       b_ring      = false;

       if(Y_(4) > 0.2){b_l_c=true;}
       if(Y_(5) > 0.2){b_r_c=true;}
       if(Y_(6) > 0.2){b_t_c=true;}
       if(Y_(7) > 0.2){b_b_c=true;}
       if(Y_(8) == 1){b_insert=true;}
       if(Y_(9) < 0.006){b_ring=true;}


       if(b_insert){
           ROS_INFO_STREAM_THROTTLE(1.0,"b_insert: true");
       }

       average_value = 1.0/static_cast<double>(X.n_rows);


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

           /// Check if close to peg origin

           if( std::sqrt( (peg_origin(1) - point_i(1)) * (peg_origin(1) - point_i(1)) +  (peg_origin(2) - point_i(2)) * (peg_origin(2) - point_i(2)) ) < 0.005)
           {
               close_peg = true;
           }else{
               close_peg = false;
           }

           /// Check Plug Ring Circle

           wsocket.distance_to_features(point_i);

           plat_dir        = wsocket.plate.C - wsocket.get_surface_projection();
           plat_dir_radius = arma::norm(plat_dir);
           plat_dir_n      = plat_dir/(plat_dir_radius + std::numeric_limits<double>::min());

           center_dir      = wsocket.plate.C - point_i;
           dist_center     = std::sqrt(center_dir(1) *center_dir(1) +  center_dir(2) *center_dir(2));
           dist_socket_c   = std::sqrt(center_dir(1) *center_dir(1) +  (center_dir(2) + 0.005) * (center_dir(2) + 0.005 ));

           // if(force_norm > 0.6 && sgmf( plat_dir_n(1) * F_n(1) + plat_dir_n(2) * F_n(2) +1,4,1) < 0.1){
           //     lik_one=0;
           // }

           if(dist_center > 0.015 && dist_center < 0.03){
               in_circle=true;
           }else{
               in_circle=false;
           }

           /// Likelihood inserted
           if(b_insert){
               if(dist_socket_c < 0.005)
               {
                   in_s = true;
                   //   std::cout<< "in_s: true" << std::endl;
               }else{
                   in_s = false;
               }
           }

           /// Check Block Edges

           if(b_t_c) // prob top   contact
           {
               // check if point is inside block top
               socket_top_edge.distance_to_features(point_i);
               if(socket_top_edge.is_inside())
               {
                   in_ts=true;
               }else{
                   in_ts=false;
               }
           }

           if(b_b_c){ // prob bottom   contact

               // check if point is bottom block top
               socket_bottom_edge.distance_to_features(point_i);
               if(socket_bottom_edge.is_inside())
               {
                   in_bs=true;
               }else{
                   in_bs=false;
               }
           }

           if(b_l_c) // prob left  contact
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

           if(b_r_c) // prob right  contact
           {
               socket_right_edge.distance_to_features(point_i);
               if(socket_right_edge.is_inside())
               {
                   in_rs=true;
               }else{
                   in_rs=false;
               }
           }


           if(b_insert){

               if(!in_s){ lik_one  = 0;}

           }else if(b_ring){
               if(!in_circle){
                   lik_one = 0;
               }
           }else if(bSense_edge){

               if(b_t_c && !b_l_c && !b_r_c  && !b_b_c){            // only    top
                   if(!in_ts && (!in_circle)){
                       lik_one = 0;
                   }
               }else if (b_l_c && !b_t_c && !b_b_c && !b_r_c){      // only    left
                   if(!in_ls && (!in_circle)){
                       lik_one = 0;
                   }
               }else if (b_r_c && !b_t_c && !b_b_c && !b_l_c){      // only    right
                   if(!in_rs && (!in_circle))
                   {
                       lik_one = 0;
                   }
               }else if (b_b_c && !b_l_c && !b_r_c && !b_t_c){      // only    bottom
                   if(!in_bs  && (!in_circle)){
                       lik_one = 0;
                   }
               }else if (b_t_c && b_l_c && !b_r_c  && !b_b_c)       // top     left
               {
                   if( (!in_ts) && (!in_ls)  && (!in_circle)){
                       lik_one = 0;
                   }
               }else if (b_t_c && b_r_c && !b_l_c && !b_b_c)        // top     right
               {
                   if( (!in_ts) && (!in_rs)  && (!in_circle)){
                       lik_one = 0;
                   }
               }else if (b_b_c && b_l_c && !b_r_c && !b_t_c){       // bottom  left
                   if( (!in_bs) && (!in_ls)  && (!in_circle)){
                       lik_one = 0;
                   }
               }else if (b_b_c && b_r_c && !b_l_c && !b_t_c){       // bottom  right
                   if( (!in_bs) && (!in_rs) && (!in_circle)){
                       lik_one = 0;
                   }
               }else if(!b_t_c && !b_l_c && !b_r_c  && !b_b_c){
                   //  if(!in_circle){
                   //     lik_one = 0;
                   //  }
               }else{
                   /// something weired happened

                   // if( (!in_bs) && (!in_rs) && (!in_ts) && (!in_ls)){
                   //     lik_one = 0;
                   // }
               }
           }



           L[i] = lik_one;

           if(close_peg && (L[i] == 0))
           {
               L[i] = 1;
           }


           // }
           sum_L = sum_L + L[i];
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
