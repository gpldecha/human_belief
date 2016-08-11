 #include "human_belief/human_measurement.h"

Human_measurement::Human_measurement(wobj::WrapObject &wrap_object):
wrap_object(wrap_object)
{

}

void Human_measurement::measurement_one(arma::colvec& Y, const arma::colvec x_, const arma::mat33& Rot){
    assert(Y.n_elem == 4);

    x = arma::conv_to<arma::fmat>::from( x_);

    wrap_object.distance_to_features(x);

    Y(0)   = wrap_object.get_distance_to_surface();
    Y(1)   = wrap_object.get_distance_to_edge();
    Y(2)   = wrap_object.get_distance_to_corner();
    Y(3)   = wrap_object.is_inside_box();

}



void Human_measurement::measurement(arma::mat& hY,const arma::mat& points, const arma::mat33& Rot)
{

    for(std::size_t i = 0; i < points.n_rows;i++){

        x = arma::conv_to<arma::fmat>::from( points.row(i).st() );

        wrap_object.distance_to_features(x);

        hY(i,0)   = wrap_object.get_distance_to_surface();
        hY(i,1)   = wrap_object.get_distance_to_edge();
        hY(i,2)   = wrap_object.get_distance_to_corner();
        hY(i,3)   = wrap_object.is_inside_box();

    }


}
