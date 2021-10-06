#ifndef INCLUDE_MODEL_HPP_
#define INCLUDE_MODEL_HPP_

#include <cmath>
#include <iostream>
#include <stdlib.h>

class Model
{
    public:

        Model();

        ~Model();

        Eigen::vector3d motionModel(double u, Eigen::vector3d xPrev);

        double measurementModel(Particle p, sensor_msgs::LaserScan scan, MapData map);

    private:
        double alpha1_, alpha2_, alpha3_, alpha4_;

};

#endif  //  INCLUDE_MODEL_HPP_