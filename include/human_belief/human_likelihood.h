#ifndef HUMAN_LIKELIHOOD_H_
#define HUMAN_LIEKLIHOOD_H_

#include <armadillo>


class Human_likelihood{

public:

  void likelihood(double* L, const arma::colvec& Y, const arma::mat& hY, const arma::mat33& Rot);

private:

};



#endif
