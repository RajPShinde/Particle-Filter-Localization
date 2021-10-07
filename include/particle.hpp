#ifndef INCLUDE_PARTICLE_HPP_
#define INCLUDE_PARTICLE_HPP_

#include <Eigen/Dense>

class Particle
{
    public:
        Eigen::Vector3d pose;
        double weight;
};

#endif  //  INCLUDE_PARTICLE_HPP_