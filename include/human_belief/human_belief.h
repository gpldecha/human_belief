#ifndef HUMAN_BELIEF_H_
#define HUMAN_BELIEF_H_

#include "table_wrapper/table_wrapper.h"

#include <particle_filter/particle_filter.h>
#include <point_mass_filter/point_mass_filter.h>
#include "human_belief/human_measurement.h"
#include "human_belief/human_likelihood.h"
#include <memory>


class Human_belief{

public:

    Human_belief(wobj::WrapObject& wrap_object);

    void update(const arma::colvec& Y, const arma::colvec& u,const arma::mat33& rot);

    void reset();

    void init_visualise(ros::NodeHandle &node);

    void visualise();

    void set_visualise_mode(opti_rviz::Vis_point_cloud::display_mode mode);

    void set_pf_color_type(pf::color_type color_t);

private:

    void init_delta_length(int init_pmf_type, pf::Point_mass_filter::delta &delta_,pf::Point_mass_filter::length& length_);

    void set_initial_pos();


private:


    Human_measurement human_measurement;
    Human_likelihood  human_likelihood;

    pf::likelihood_model lik_func;
    pf::Measurement_h     hY_func;

    std::shared_ptr<pf::Point_mass_filter> pmf;


    int             init_pmf_param;
    arma::colvec3   init_pos;


};

#endif
//     pmf(lik_func,hY_func,delta_,length_,dim_Y);


