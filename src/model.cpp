#include <model.hpp>

Model::Model(double alpha1, double alpha2, double alpha3, double alpha4,
             double zHit, double zShort, double zRand, double zMax,
             double sigmaHit, double lambdaShort){// : alpha1_(alpha1), alpha2_(alpha2), alpha3_(alpha3), alpha4_(alpha4){
}

Model::~Model(){
}

Eigen::Vector3d Model::motionModel(std::vector<std::vector<double>> u, Eigen::Vector3d xPrev){
	// ROS_INFO_STREAM("Odometry");
	double deltaRot1, delatTrans, deltaRot2;
	double deltaRot1Sd, delatTransSd, deltaRot2Sd;
	double deltaRot1Hat, delatTransHat, deltaRot2Hat;
	Eigen::Vector3d x;
	x  << xPrev(0), xPrev(1), xPrev(2);

	deltaRot1 = std::atan2(u[1][1]-u[0][1],u[1][0]-u[0][0]) - u[0][2];
	delatTrans = std::sqrt(std::pow(u[1][0]-u[0][0],2)+std::pow(u[1][1]-u[0][1],2));
	deltaRot2 = u[1][2] - u[0][2] - deltaRot1;

	deltaRot1Sd = alpha1_*deltaRot1 + alpha2_*delatTrans;
	delatTransSd = alpha3_*delatTrans + alpha4_*(deltaRot1 + deltaRot2);
	deltaRot2Sd = alpha1_*deltaRot2 + alpha2_*delatTrans;;

	deltaRot1Hat = deltaRot1 - sampleNormalDistribution(deltaRot1Sd);
	delatTransHat = delatTrans - sampleNormalDistribution(delatTransSd);
	deltaRot2Hat = deltaRot2 - sampleNormalDistribution(deltaRot2Sd);

	x(0) += delatTransHat*std::cos(x(2)+deltaRot1Hat);
	x(1) += delatTransHat*std::sin(x(2)+deltaRot1Hat);
	x(2) += deltaRot1Hat + deltaRot2Hat;

	return x;
}

double Model::measurementModel(Particle p, sensor_msgs::LaserScan scan, MapData map){
	// ROS_INFO_STREAM("Measurement");
	double weight = 0;
	double sensorX = p.pose(0) + sensorOffsetX*cos(p.pose(2));
	double sensorY = p.pose(1) + sensorOffsetY*sin(p.pose(2));
	double sensorTheta = p.pose(2);

	if(map.data[(int)(p.pose(1)/map.resolution)][(int)(p.pose(0)/map.resolution)]!=0 || map.data[(int)(sensorY/map.resolution)][(int)(sensorX/map.resolution)]!=0)
		return 0;

	for(int i = 0; i<720; i++){
		double zkt = scan.ranges[i];

		if(zkt>sensorRangeMax_ || zkt<sensorRangeMin_)
			continue;

		double zktStep = sensorTheta - pi_ + i*sensorStep_;

		double zktX = sensorX + zkt*std::cos(zktStep);
		double zktY = sensorY + zkt*std::sin(zktStep);
		if(zktX<map.xMin*map.resolution || zktX>map.xMax*map.resolution || zktY<map.yMin*map.resolution || zktY>map.yMax*map.resolution)
			continue;
		// ROS_INFO_STREAM(zktX<<" "<<zktY);
		weight += map.data[(int)(zktY/map.resolution)][(int)(zktX/map.resolution)] ==0 ? 1 : 0;
	}

	return weight;
}

double Model::sampleNormalDistribution(double sigma){
	// Box-Muller Transform
	double x1, x2, w, r;
	do{

		// Generate a uniform distribution [0, 1]
		// Scale the Uniform distribution by 2 and left shift by 1 i.e Uniform Distribution [-1,1]
		do{
			// Generate a random number between [0,1]
			r = drand48();
		}while(r==0);
		// Project the number in distribution [-1, 1]
		x1 = 2*r -1;
		do{
			// Generate a random number between [0,1]
			r = drand48();
		}while(r==0);
		// Project the number in distribution [-1, 1]
		x2 = 2*r -1;
		w = x1*x1 + x2*x2;
	}while(w>1 || w==0);

	return sigma * x2 * std::sqrt(-2.0*std::log(w)/w);
}