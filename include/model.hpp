#ifndef INCLUDE_MODEL_HPP_
#define INCLUDE_MODEL_HPP_

#include <cmath>
#include <iostream>
#include <stdlib.h>
#include <sensor_msgs/LaserScan.h>
#include <particle.hpp>
#include <mapData.hpp>
#include <Eigen/Dense>
#include <model.hpp>

class Model
{
    public:

        Model(double alpha1_, double alpha2_, double alpha3_, double alpha4_,
              double zHit_, double zShort_, double zRand_, double zMax_,
              double sigmaHit_, double lambdaShort_);

        ~Model();

        Eigen::Vector3d motionModel(std::vector<std::vector<double>> u, Eigen::Vector3d xPrev);

        double measurementModel(Particle p, sensor_msgs::LaserScan scan, MapData map);

        double sampleNormalDistribution(double sigma);

    private:
        double alpha1_, alpha2_, alpha3_, alpha4_;
        double zHit_, zShort_, zRand_, zMax_;
        double sigmaHit_, lambdaShort_;
        double sensorOffsetX = 0.75;
        double sensorOffsetY = -0.23;
        double sensorRangeMax_ = 9;
        double sensorRangeMin_ = 0.699999988079;
        double sensorStep_ = 0.00873877573758;
        double pi_ = 3.14159;


};

#endif  //  INCLUDE_MODEL_HPP_