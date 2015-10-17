// STL

#include <map>
#include <string>

// Boost

#include <boost/lexical_cast.hpp>

// catkin package

#include "optitrack_rviz/listener.h"
#include "optitrack_rviz/broadcaster.h"

#include "socket_table_broadcaster/services.h"
#include "socket_table_broadcaster/load.h"


int find_index(int argc,const std::vector<std::string>& argv,std::string str){
    for(int i = 0; i < argc;i++){
        if(argv[i] == str){
            return i;
        }
    }
    return -1;
}

bool process_input(int argc, char **argv,std::map<std::string,std::string>& input,
                   std::array<float,3>& origin,std::array<float,4>& orientation,std::array<float,3>& offset){

    if(argc < static_cast<int>(input.size())){
        std::string error_msg = "options ";
        for(auto it = input.begin(); it != input.end();it++){
            error_msg += it->first + " ";
        }
        error_msg += "not defined!";
        ROS_ERROR("%s",error_msg.c_str());
        return false;
    }else{

        int index = 0;
        std::string empty = "";

        std::vector<std::string> input_args(argc);
        for(int i = 0; i < argc;i++){
            input_args[i] = std::string(argv[i]);
        }


        for(auto it = input.begin(); it != input.end();it++){
                index = find_index(argc,input_args,it->first);

                if(index == -1 && (it->second == empty)){
                    ROS_ERROR("%s [arg] not specified",it->first.c_str());
                    return false;
                }else if(index != -1){
                    if(it->first == "-origin"){
  //                      std::cout<< "origin" << std::endl;
//                        std::cout<< "index+1: " << index + 1 << std::endl;
                        origin[0] = boost::lexical_cast<float>(input_args[index+1]);
                        origin[1] = boost::lexical_cast<float>(input_args[index+2]);
                        origin[2] = boost::lexical_cast<float>(input_args[index+3]);

                        (it->second) = input_args[index+1] + " " + input_args[index+2] + " " + input_args[index+3];
                    }else if(it->first == "-orientation"){
                       // std::cout<< "orientation" << std::endl;
                        //std::cout<< "index+1: " << index + 1 << std::endl;
                        orientation[0] = boost::lexical_cast<float>(input_args[index+1]);
                        orientation[1] = boost::lexical_cast<float>(input_args[index+2]);
                        orientation[2] = boost::lexical_cast<float>(input_args[index+3]);
                        orientation[3] = boost::lexical_cast<float>(input_args[index+4]);
                        (it->second) = input_args[index+1] + " " + input_args[index+2] + " " + input_args[index+3] + " " + input_args[index+4];
                    }else if(it->first == "-offset"){
                        offset[0] = boost::lexical_cast<float>(input_args[index+1]);
                        offset[1] = boost::lexical_cast<float>(input_args[index+2]);
                        offset[2] = boost::lexical_cast<float>(input_args[index+3]);
                        (it->second) = input_args[index+1] + " " + input_args[index+2] + " " + input_args[index+3];

                    }else{
                        (it->second) =  std::string(argv[index + 1]);
                    }
                }
        }

        return true;
    }
}

void print_input_options(const std::map<std::string,std::string>& input){
    for(auto it = input.begin(); it != input.end();it++){
        ROS_INFO("%s\t%s",it->first.c_str(),it->second.c_str());
    }
}

std::string num2str(float num){
    return boost::lexical_cast<std::string>(num);
}

int main(int argc,char** argv)
{

    std::map<std::string,std::string> input;

    input["-fixed_frame"]          = "";
    input["-target_frame_listener"]  = "";
    input["-target_frame_broadcaster"]    = "";
    input["-origin"]               = "0 0 0";
    input["-orientation"]          = "0 0 0 1";
    input["-offset"]               = "0 0 0";
    input["-save"]                 = "/home/guillaume/";
    input["-load"]                 = "False";
    input["-rate"]                 = "100";

    std::array<float,3> origin_input,offset;
    std::array<float,4> orientation_input;
    offset = {{0,0,0}};


    if(!process_input(argc,argv,input,origin_input,orientation_input,offset)){
        return -1;
    }

    tf::Vector3     origin;
    tf::Matrix3x3   orientation;
    tf::Quaternion  q;

    tf::Matrix3x3 Rot;
    Rot.setRPY(0,0,M_PI);

    bool bLoad = false;

    origin.setValue(origin_input[0],origin_input[1],origin_input[2]);
    q.setX(orientation_input[0]);  q.setY(orientation_input[1]); q.setZ(orientation_input[2]); q.setW(orientation_input[3]);
    orientation.setRotation(q);

    if(input["-load"] == "True"){
        std::cout<< "== load == " << std::endl;
        bLoad=true;
        sock_tab::Load::load(origin,q,input["-save"],input["-target_frame_broadcaster"]);
        //orientation.setRotation(q);
        //orientation = Rot * orientation;

        input.at("-origin") = num2str(origin.x()) + " " + num2str(origin.y()) + " " + num2str(origin.z());
        input.at("-orientation") = num2str(q.getX()) + " " + num2str(q.getY()) + " " + num2str(q.getZ()) + " "
                 + num2str(q.getW());
    }



    print_input_options(input);


    ros::init(argc, argv,"~",ros::init_options::AnonymousName);
    ros::NodeHandle node;

    opti_rviz::Listener     listener(input["-fixed_frame"],input["-target_frame_listener"]);
    opti_rviz::Broadcaster  broadcaster(input["-fixed_frame"],input["-target_frame_broadcaster"]);

    sock_tab::Save     save(origin,orientation,input["-save"],input["-target_frame_broadcaster"]);
    sock_tab::Services services(node,save);

    tf::Matrix3x3 tmp;

    int Hz = boost::lexical_cast<int>(input["-rate"]);
    ros::Rate rate(Hz);
    while(node.ok()){

        // listen to plug and set table
        if(!bLoad){
            listener.update(origin,orientation);
            tmp[0][0] = -orientation[0][0];
            tmp[1][0] = -orientation[1][0];
            tmp[2][0] = -orientation[2][0];

            tmp[0][1] = -orientation[0][1];
            tmp[1][1] = -orientation[1][1];
            tmp[2][1] = -orientation[2][1];
            orientation = tmp;

             //orientation = Rot * orientation;
             //orientation.setRotation(q);
        }

        broadcaster.update(origin,orientation);


        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
