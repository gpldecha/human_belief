#include "human_belief/human_belief.h"


Human_belief::Human_belief(ros::NodeHandle &nh, wobj::WrapObject& wrap_object):
human_measurement(wrap_object)
{

    lik_func  =  std::bind(&Human_likelihood::likelihood,   &human_likelihood, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4);
    hY_func   =  std::bind(&Human_measurement::measurement, &human_measurement,std::placeholders::_1,std::placeholders::_2,std::placeholders::_3);

    init_pmf_param = 0;

    pf::Point_mass_filter::delta       delta;
    pf::Point_mass_filter::length      length;
    init_delta_length(init_pmf_param,delta,length);

    pmf.reset( new pf::Point_mass_filter(lik_func,hY_func,delta,length,4));
    reset();

    service = nh.advertiseService("pmf",&Human_belief::service_callback,this);

    bRun  = false;


    ROS_INFO_STREAM("   PMF   INITIALISED  ");
}


void Human_belief::reset(){

    ROS_INFO_STREAM("   PMF RESET  ");


    pf::Point_mass_filter::delta       delta;
    pf::Point_mass_filter::length      length;
    init_delta_length(init_pmf_param,delta,length);

    pmf->reset(init_pos,delta,length);
}


void Human_belief::update(const arma::colvec& Y, const arma::colvec& u,const arma::mat33& rot){
    if(bRun){
        pmf->set_rotation(rot);
        pmf->update(u,Y);
    }
}

void Human_belief::init_visualise(ros::NodeHandle &node){
    pmf->init_visualise(node,"pfilter");
}

void Human_belief::visualise(){
    pmf->visualise();
}

void Human_belief::set_visualise_mode(opti_rviz::Vis_point_cloud::display_mode mode){
    pmf->set_visualisation_mode(mode);
}

void Human_belief::set_pf_color_type(pf::color_type color_t){
    pmf->set_color_mode(color_t);
}


void Human_belief::init_delta_length(int init_pmf_type, pf::Point_mass_filter::delta &delta_,pf::Point_mass_filter::length& length_){

    std::cout<< "INIT DELTA LENGTH: " << init_pmf_type << std::endl;

    if(init_pmf_type == 0)
    {
        delta_.m  = 0.02;
        delta_.n  = 0.03;
        delta_.k  = 0.02;

        length_.m = 0.5;
        length_.n = 1.2;
        length_.k = 0.15;
    }
    else if(init_pmf_type == 1)
    {
        delta_.m  = 0.01;
        delta_.n  = 0.04;
        delta_.k  = 0.04;

        length_.m = 0.2;
        length_.n = 0.7;
        length_.k = 0.2;
    }
}



bool Human_belief::service_callback(human_belief::String_cmd::Request& req, human_belief::String_cmd::Response& res){

    std::string cmd = req.cmd;

    if(cmd == "run")
    {
        bRun = !bRun;

        if(bRun)
        {
            res.res = "pmf set to RUN!";
        }else{
            res.res = "pmf set to STOP!";
        }

    }else if (cmd == "reset")
    {
        reset();
    }else{

    }

    return true;
}
