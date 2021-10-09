#ifndef INCLUDE_MODEL_HPP_
#define INCLUDE_MODEL_HPP_

#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <particle.hpp>
#include <mapData.hpp>
#include <Eigen/Dense>
#include <model.hpp>

class Model
{
    public:

        Model(double alpha1, double alpha2, double alpha3, double alpha4,
              double zHit, double zShort, double zRand, double zMax,
              double sigmaHit, double lambdaShort);

        ~Model();

        Eigen::Vector3d motionModel(std::vector<std::vector<double>> u, Eigen::Vector3d xPrev);

        double measurementModel(Particle p, sensor_msgs::LaserScan scan, MapData map);

        double measurementModelLikelihoodField(Particle p, sensor_msgs::LaserScan, MapData map, std::vector<std::vector<double>> distances_);

        double sampleNormalDistribution(double sigma);

        double gaussianDistribution(double mean, double standardDeviation, double x);

    private:
        // double alpha1_, alpha2_, alpha3_, alpha4_;
        // double zHit_, zShort_, zRand_, zMax_;
        // double sigmaHit_, lambdaShort_;
        double sensorOffsetX = 0.75;
        double sensorOffsetY = -0.23;
        double sensorRangeMax_ = 9;
        double sensorRangeMin_ = 0.699999988079;
        double sensorStep_ = 0.00873877573758;
        double pi_ = 3.14159;
        double alpha1_ = 0.01;
        double alpha2_ = 0.01;
        double alpha3_ = 0.01;
        double alpha4_ = 0.01;
        double zHit_ = 0.8;
        double zShort_ = 0.2;
        double zRand_ = 0;
        double zMax_ = 0;
        double sigmaHit_ = 2;
        double lambdaShort_ = 1.5;


};

#endif  //  INCLUDE_MODEL_HPP_