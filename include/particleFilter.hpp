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
#include <mapData.hpp>
#include <model.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <chrono>
#include <thread>

class ParticleFilter
{
    public:       

        ParticleFilter(ros::NodeHandle &nh, int n);

        ~ParticleFilter();

        void odomCallback(const nav_msgs::Odometry msg);

        void scanCallback(const sensor_msgs::LaserScan msg);

        void mapCallback(const nav_msgs::OccupancyGrid msg);

        void fromPiToMinusPi(double &angle);

        void normalize(double totalWeight);

        void distances();

        void initializeParticles();

        void publishPose();

        void resample();

        void localize();

    private:
        ros::NodeHandle filter_;
        ros::Subscriber odomSub_;
        ros::Subscriber scanSub_;
        ros::Subscriber mapSub_;
        ros::Publisher particlePub_;
        ros::Publisher scanPub_;
        MapData map_;
        std::vector<Particle> particles_;
        double noOfParticles_;
        nav_msgs::Odometry odomData_;
        sensor_msgs::LaserScan scanData_;
        geometry_msgs::PoseArray particlePoses_;
        bool initialized_ = false;
        bool odomReceived_ = false;
        bool scanReceived_ = false;
        double pi_ = 3.14159;
        double alpha1_ = 0.03;
        double alpha2_ = 0.03;
        double alpha3_ = 0.4;
        double alpha4_ = 0.4;
        double zHit_ = 0.95;
        double zShort_ = 0.2;
        double zRand_ = 0;
        double zMax_ = 0;
        double sigmaHit_ = 2;
        double lambdaShort_ = 1.5;
        std::vector<std::pair<int,int>> occupied_;
        std::vector<std::pair<int,int>> free_;
        std::vector<std::vector<double>> distances_;
};

#endif  //  INCLUDE_PARTICLEFILTER_HPP_