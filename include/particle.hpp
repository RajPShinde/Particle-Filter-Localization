#ifndef INCLUDE_PARTICLE_HPP_
#define INCLUDE_PARTICLE_HPP_

class Particle
{
    public:
        Eigen::vector3d pose;
        double weight;
};

#endif  //  INCLUDE_PARTICLE_HPP_