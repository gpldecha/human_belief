 #include "human_belief/human_measurement.h"
#include <optitrack_rviz/type_conversion.h>

Human_measurement::Human_measurement(ros::NodeHandle& nh,wobj::WrapObject &wrap_object, bool bVisualise):
wrap_object(wrap_object),
  bVisualise(bVisualise)
{
    if(bVisualise){
        v_surf.resize(1);
        v_edge.resize(1);
        v_corner.resize(1);

        std::string fixed_frame = "world";

        vis_proj_sur.reset(new opti_rviz::Vis_points(nh,"proj_surf"));
        vis_proj_sur->scale = 0.01;
        vis_proj_sur->alpha = 1;
        vis_proj_sur->r     = 1;
        vis_proj_sur->g     = 0;
        vis_proj_sur->b     = 0;
        vis_proj_sur->initialise(fixed_frame,v_surf);

        vis_proj_edge.reset(new opti_rviz::Vis_points(nh,"proj_edge"));
        vis_proj_edge->scale = 0.01;
        vis_proj_edge->alpha = 1;
        vis_proj_edge->r     = 0;
        vis_proj_edge->g     = 0;
        vis_proj_edge->b     = 1;
        vis_proj_edge->initialise(fixed_frame,v_edge);

        vis_proj_corner.reset(new opti_rviz::Vis_points(nh,"proj_corner"));
        vis_proj_corner->scale = 0.01;
        vis_proj_corner->alpha = 1;
        vis_proj_corner->r     = 0;
        vis_proj_corner->g     = 1;
        vis_proj_corner->b     = 1;
        vis_proj_corner->initialise(fixed_frame,v_corner);


    }
}

void Human_measurement::measurement_one(arma::colvec& Y, const arma::colvec x_, const arma::mat33& Rot){
    assert(Y.n_elem == 4);

    x = arma::conv_to<arma::fmat>::from( x_);

    wrap_object.distance_to_features(x);

    Y(0)   = wrap_object.get_distance_to_surface();
    Y(1)   = wrap_object.get_distance_to_edge();
    Y(2)   = wrap_object.get_distance_to_corner();
    Y(3)   = wrap_object.is_inside_box();

    if(bVisualise)
    {

        // visualise closest edge
        geo::fCVec3& c_edge = wrap_object.get_closest_point_edge();
        opti_rviz::type_conv::vec2tf(c_edge,v_edge[0]);
        vis_proj_edge->update(v_edge);

        // visualise closest surf
        geo::fCVec3& c_surf = wrap_object.get_closest_point_surface();
        opti_rviz::type_conv::vec2tf(c_surf,v_surf[0]);
        vis_proj_sur->update(v_surf);

        // visualise closest corner
        geo::fCVec3& c_corner = wrap_object.get_closest_point_corner();
        opti_rviz::type_conv::vec2tf(c_corner,v_corner[0]);
        vis_proj_corner->update(v_corner);

        vis_proj_edge->publish();
        vis_proj_sur->publish();
        vis_proj_corner->publish();

    }

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
