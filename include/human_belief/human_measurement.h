#ifndef HUMAN_MEASUREMENT_H_
#define HUMAN_MEASUREMENT_H_

#include <armadillo>
#include <wrapobject.h>
#include <visualise/vis_points.h>

/**
 * @brief The Human_measurement class
 *
 *  Probability of contact
 *
 *  Y : p_contact_edge
 *      p_contact_surf
 *      p_contact_corner
 *      p_in_object
 */



class Human_measurement{

public:

    enum class MEASURE_IDX : std::size_t {SURF_ID=0,EDGE_ID=1,CORNER_ID=2,IN_ID=3};


public:

    Human_measurement(ros::NodeHandle& nh, wobj::WrapObject& wrap_object,bool bVisualise);


    void measurement_one(arma::colvec& Y, const arma::colvec x_, const arma::mat33& Rot);


    /**
     * @brief measurement_h : measurement function h(X) prototype, Y = h(X)
     *                        given state information X, produces a virtual sensation.
     */
    void measurement(arma::mat& hY,const arma::mat& points, const arma::mat33& Rot);

private:

    wobj::WrapObject& wrap_object;
    geo::fCVec3 x;

    bool bVisualise;
    boost::shared_ptr<opti_rviz::Vis_points> vis_proj_sur,vis_proj_edge, vis_proj_corner;
    std::vector<tf::Vector3> v_surf,v_edge,v_corner;

};

#endif
