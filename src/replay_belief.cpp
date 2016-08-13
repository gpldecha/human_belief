#include <ros/ros.h>
#include <table_wrapper/table_wrapper.h>
#include <optitrack_rviz/input.h>
#include "human_belief/human_belief.h"

#include <optitrack_rviz/listener.h>
#include <optitrack_rviz/type_conversion.h>
#include <optitrack_rviz/debug.h>
#include <visualise/vis_points.h>


int main(int argc, char** argv){

    // -------------- Get node input paramters -------------- //

    std::map<std::string,std::string> input;
    input["-fixed_frame"]       = "/world";
    input["-rate"]              = "100";
    input["-table_frame"]       = "/table_link";
    input["-hand_frame"]        = "";

    if(!opti_rviz::Input::process_input(argc,argv,input)){
        ROS_ERROR("failed to load input");
        return 0;
    }
    opti_rviz::Input::print_input_options(input);

    double      rate_hz               = boost::lexical_cast<double>(input["-rate"]);
    std::string fixed_frame           = input["-fixed_frame"];
    std::string table_frame           = input["-table_frame"];
    std::string hand_frame            = input["-hand_frame"];


    // -------------- Initialise node --------------- //

    ros::init(argc, argv, "replay_belief");
    ros::NodeHandle nh;
    ros::Rate rate(rate_hz);


    // -------------- Table wrapper ------------------ //


    Table_wrapper table_wrapper(nh,table_frame,true,fixed_frame);


    // -------------- Initialise Belief ----------------//

    Human_belief human_belief(nh,table_wrapper.get_wrapped_objects());
    human_belief.init_visualise(nh);
    human_belief.set_pf_color_type(pf::C_WEIGHTS);
    human_belief.set_visualise_mode(opti_rviz::Vis_point_cloud::ONLY_HIGH_WEIGHTS);

    // -------------- Initialise Virtual Hand Sensor ---//

    Human_measurement human_measurement(nh,table_wrapper.get_wrapped_objects(),true);


    // -------------- Human hand listener --------------//

    opti_rviz::Listener listener(fixed_frame,hand_frame);
    opti_rviz::Vis_points vis_point(nh,"hand_vis");
    std::vector<tf::Vector3> points(1);
    vis_point.r = 1; vis_point.g = 1; vis_point.b = 1; vis_point.alpha = 1;
    vis_point.scale = 0.05;
    vis_point.initialise(fixed_frame,points);


    tf::Vector3   hand_pos, hand_pos_tmp;
    tf::Matrix3x3 hand_rot;

    arma::colvec   Y; Y.resize(4);
    arma::colvec3  hand_pos_arma, u;
    arma::mat33    hand_rot_arma;


    tf::Vector3  hand_vel;

    ROS_INFO_STREAM("   Start ROS FOR LOOP");
    while(nh.ok()){

        hand_pos_tmp = hand_pos;
        listener.update(hand_pos,hand_rot);
        hand_vel = hand_pos - hand_pos_tmp;
        opti_rviz::type_conv::tf2vec(hand_vel,u);

        // compute hand measurement
        opti_rviz::type_conv::tf2vec(hand_pos,hand_pos_arma);
        opti_rviz::type_conv::tf2mat(hand_rot,hand_rot_arma);
        human_measurement.measurement_one(Y,hand_pos_arma,hand_rot_arma);

        // update belief
        human_belief.update(Y,u,hand_rot_arma);


        // visualise hand
        points[0] = hand_pos;
        vis_point.update(points);
        vis_point.publish();


        human_belief.visualise();


        table_wrapper.update();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
