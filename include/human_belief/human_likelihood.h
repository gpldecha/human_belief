#ifndef HUMAN_LIKELIHOOD_H_
#define HUMAN_LIEKLIHOOD_H_

#include <armadillo>
#include <wrapobject.h>


class Human_likelihood{

    enum {SURF=0,EDGE=1,CORN=2};

public:

    Human_likelihood(wobj::WrapObject& wrap_object);

    // L.memptr(),Y,points,Rot

  void likelihood(double* L, const arma::colvec& Y, const arma::mat& points, const arma::mat33& Rot);



  inline float bel_function(const double dist,const double max_dist){

      if(dist >= max_dist)
      {
          return 0;
      }else
      {
          return 1;
      }


  }

private:

  wobj::WrapObject& wrap_object;
  arma::colvec3 hY;
  geo::fCVec3 x;
 arma::colvec Y_;
 double max_dist;



};



#endif
