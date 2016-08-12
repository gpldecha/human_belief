#include "human_belief/human_likelihood.h"


/*
 *      hY(i,0)   = wrap_object.get_distance_to_surface();
        hY(i,1)   = wrap_object.get_distance_to_edge();
        hY(i,2)   = wrap_object.get_distance_to_corner();
        hY(i,3)   = wrap_object.is_inside_box();
 *
*/

Human_likelihood::Human_likelihood(wobj::WrapObject &wrap_object)
    :wrap_object(wrap_object)
{

    max_dist = 0.04;

}


void Human_likelihood::likelihood(double* L, const arma::colvec& Y, const arma::mat& points, const arma::mat33& Rot){



    Y_ =  Y;

  //  Y_(SURF) = 0.01;


    Y_(SURF)   = bel_function(Y_(SURF),max_dist);
    Y_(EDGE)   = bel_function(Y_(EDGE),max_dist);
    Y_(CORN)   = bel_function(Y_(CORN),max_dist);

    for(std::size_t i = 0; i < points.n_rows;i++)
    {


        x = arma::conv_to<arma::fmat>::from( points.row(i).st() );
        wrap_object.distance_to_features(x);


        L[i] = 1;

        if(wrap_object.is_inside_box())
        {
            L[i] = 0;

        }else{


            hY(SURF)   = wrap_object.get_distance_to_surface();
            hY(EDGE)   = wrap_object.get_distance_to_edge();
            hY(CORN)   = wrap_object.get_distance_to_corner();

            hY(SURF)   = bel_function(hY(SURF),max_dist);
            hY(EDGE)   = bel_function(hY(EDGE),max_dist);
            hY(CORN)   = bel_function(hY(CORN),max_dist);


            // likelihood edge
            if(  std::fabs(hY(EDGE) - Y_(EDGE)) > 0.05)
            {
              L[i] = 0;
            }

            // likelihood corner
            if(  std::fabs(hY(CORN) - Y_(CORN)) > 0.05)
            {
              L[i] = 0;
            }

            // likelihood surf
            if(  std::fabs(hY(SURF) - Y_(SURF)) > 0.05)
            {
              L[i] = 0;
            }


        }



    }





}
