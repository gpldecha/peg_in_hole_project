#include "peg_filter/belief_features/mode_feature.h"

ModeBeliefCompress::ModeBeliefCompress(ros::NodeHandle& nh,const std::string& name)
    :BaseBeliefCompression(name),vis_points(nh,"bel_x"),vis_gmm(nh,"Sigma")
{

    pub     = nh.advertise<std_msgs::Float64MultiArray>("belief_features",10);
    sub_    = nh.subscribe<std_msgs::Bool>("belief_features/reset",1,&ModeBeliefCompress::reset,this);

    bFirst = true;
    feature_msg.data.resize(4);

    vis_pts.resize(1,3);

    vis_points.r = 1;
    vis_points.g = 1;
    vis_points.b = 1;
    vis_points.scale = 0.02;
    vis_points.alpha = 1;

    vis_gmm.a = 0.1;
    vis_gmm.r = 1;
    vis_gmm.g = 1;
    vis_gmm.b = 0;

    vis_gmm.initialise("world",mode_point,Sigma);


    argmax_w = 0;

    std::cout<< "Inside ModeBeliefCompress constructor" << std::endl;

    bFristModeFeature = true;

    vis_points.initialise("world",vis_pts);

    mode_point(0) = arma::datum::inf;
    mode_point_tmp(0) = arma::datum::inf;
    bReset            = true;


}

void ModeBeliefCompress::arg_max_x(arma::colvec3& pos,double& w,const arma::colvec3& pos_tmp, const arma::mat& points, const arma::cube& P){
    /* [pos,w,b_resample] = argmax_x_bel(XYZW,pos_tmp)
    %ARGMAX_X_BEL find the most likely state under the belief distribution XYZW
    %
    %   input ---------------------------------------------------------------
    %
    %       o XYZW: (N x [4 or 3]), discrete probability density function with N
    %                                samples.
    %
    %       o pos_tmp (1 x [3 or 2]), position of the most likely state at the
    %                          previous time setp after motion dynamics have
    %                          been applied
    %
*/

    int r,c,k;
    r=0;
    c=0;
    k=0;
    arma::uword idx;

    //  % if empty then first time the function is called
    if(pos_tmp.has_inf() || bReset){
        bReset = false;
        // randomely generate index (r,c,k)
       /* r = arma::randi(1,arma::distr_param(0,P.n_rows-1))(0);
        c = arma::randi(1,arma::distr_param(0,P.n_cols-1))(0);
        k = arma::randi(1,arma::distr_param(0,P.n_slices-1))(0);
        assert(r < P.n_rows);
        assert(c < P.n_cols);
        assert(k < P.n_slices);*/
        //idx = pf::sub2ind_col_major(r,c,k,P.n_rows,P.n_cols);
        multinomial = std::discrete_distribution<int>(P.begin(),P.end());
        idx         = multinomial(generator);
    }else{

        // find the idx of the current chosen moste likely state
        arma::sqrt(arma::sum(   arma::pow(points - arma::repmat(pos_tmp.st(),points.n_rows,1),2) ,1)   ).min(idx);

        // get the weight of the point which is the closest to the max.
        double w_idx = P.at(idx);

       // ROS_INFO_STREAM_THROTTLE(1.0,"w_idx/max_w : " << w_idx/max_w );


        if(w_idx/max_w < 0.8){
            // sample

            multinomial = std::discrete_distribution<int>(P.begin(),P.end());
            idx         = multinomial(generator);
        }
    }


    assert(idx < P.n_elem);
    assert(idx < points.n_rows);

    pos = points.row(idx).st();
    w   = P.at(idx);
}

void ModeBeliefCompress::reset(const std_msgs::BoolConstPtr& msg){
    bReset = true;
}

void ModeBeliefCompress::update(const arma::colvec3& velocity, const arma::mat &points, const arma::cube& P){

    if(!mode_point_tmp.has_inf()){
        mode_point_tmp = mode_point_tmp + velocity;
    }

    // compute covariance
   // std::cout<< "ModeBeliefCompress::update" << std::endl;
   // std::cout<< "P: " <<    P.n_rows << " x " << P.n_cols << " x " << P.n_slices << std::endl;
   // std::cout<< "points: " <<    points.n_rows << " x " << points.n_cols << std::endl;

    max_w = P.max();
    Sigma.zeros();

    double sum_P = arma::sum(arma::vectorise(P));
    mean.zeros();
    for(std::size_t i = 0; i < points.n_rows;i++){
        mean = mean + P.at(i) * points.row(i);
    }
    mean = mean / (sum_P + std::numeric_limits<double>::min());


    for(std::size_t i = 0; i < points.n_rows;i++){
        Sigma = Sigma + P.at(i) * (points.row(i) - mean).st() * (points.row(i) - mean);
    }

    Sigma = Sigma / (sum_P + std::numeric_limits<double>::min());

    Sigma +=  arma::eye(3,3) * 1e-05;
    Entropy = 0.5 * log(4.9822e+03 * arma::det(Sigma));


   arg_max_x(mode_point,argmax_w,mode_point_tmp,points,P);


    mode_point_tmp = mode_point;


    if(mode_point.has_nan()){

        std::cout<< "MODE_POINT HAS NAN" << std::endl;
        exit(0);
    }

    //mode_point.print("mode_point");

    feature_msg.data[hX] = mode_point(hX);
    feature_msg.data[hY] = mode_point(hY);
    feature_msg.data[hZ] = mode_point(hZ);
    feature_msg.data[H]  = Entropy;

    vis_gmm.update(mean.st(),Sigma);
    vis_gmm.publish();

    pub.publish(feature_msg);
    visualize();

}

arma::colvec3 ModeBeliefCompress::get_mode(){
    return mode_point;
}

void ModeBeliefCompress::visualize(){
    vis_pts(0,0) = mode_point(0);
    vis_pts(0,1) = mode_point(1);
    vis_pts(0,2) = mode_point(2);
    vis_points.update(vis_pts);
    vis_points.publish();

}
