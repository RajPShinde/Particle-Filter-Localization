#ifndef INCLUDE_MODEL_HPP_
#define INCLUDE_MODEL_HPP_

#include <cmath>
#include <iostream>

class Model
{
    public:

        Model();

        ~Model();

        Eigen::vecotr2d motionModel(double u, double xPrev);

        double measurementModel();

    private:

};

#endif  //  INCLUDE_MODEL_HPP_