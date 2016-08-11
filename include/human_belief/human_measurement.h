#ifndef HUMAN_MEASUREMENT_H_
#define HUMAN_MEASUREMENT_H_

#include <armadillo>
#include <wrapobject.h>

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

    Human_measurement(wobj::WrapObject& wrap_object);


    void measurement_one(arma::colvec& Y, const arma::colvec x_, const arma::mat33& Rot);


    /**
     * @brief measurement_h : measurement function h(X) prototype, Y = h(X)
     *                        given state information X, produces a virtual sensation.
     */
    void measurement(arma::mat& hY,const arma::mat& points, const arma::mat33& Rot);

private:

    wobj::WrapObject& wrap_object;

    geo::fCVec3 x;

};

#endif
