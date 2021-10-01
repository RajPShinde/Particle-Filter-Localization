#ifndef INCLUDE_PARTICLEFILTER_HPP_
#define INCLUDE_PARTICLEFILTER_HPP_

#include <vector>
#include <cmath>
#include <iostream>
#include <string>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>

class ParticleFilter
{
    public:

        ParticleFilter();

        ~ParticleFilter();

        void odomCallback();

        void scanCallback();

    private:

};

#endif  //  INCLUDE_PARTICLEFILTER_HPP_