#ifndef INCLUDE_PARTICLEFILTER_HPP_
#define INCLUDE_PARTICLEFILTER_HPP_

#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>
#include <particle.hpp>

class ParticleFilter
{
    public:

        ParticleFilter();

        ~ParticleFilter();

        void odomCallback(const nav_msgs::Odometry msg);

        void scanCallback();

        void drawParticles(std::vector<Particle> particles);

    private:
        std::vector<Particle> particles_;
        double numberOfParticles;

};

#endif  //  INCLUDE_PARTICLEFILTER_HPP_