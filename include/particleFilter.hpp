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
#include <tf/transform_datatypes.h>

class ParticleFilter
{
    public:

        ParticleFilter();

        ~ParticleFilter();

        void odomCallback(const nav_msgs::Odometry msg);

        void scanCallback(const sensor_msgs::LaserScan msg);

        void drawParticles();

        void localize();

    private:
        ros::Subscriber odomSub_;
        ros::Subscriber scanSub_;
        ros::Publisher particlePub_;

        std::vector<Particle> particles_;
        double numberOfParticles;
        nav_msgs::Odometry odomData_;
        sensor_msgs::LaserScan scanData_;
        geometry_msgs::PoseArray particlePoses_;
        double totalWeight_ = 0;


};

#endif  //  INCLUDE_PARTICLEFILTER_HPP_