#include <model.hpp>

Model::Model(double alpha1, double alpha2, double alpha3, double alpha4) : alpha1_(alpha1), alpha2_(alpha2), alpha3_(alpha3), alpha4_(alpha4){
}

Model::~Model(){
}

Eigen::vector3d Model::motionModel(std::vector<std::vector<double>> u, Eigen::vector3d xPrev){

	double deltaRot1, delatTrans, deltaRot2;
	double deltaRot1Sd, delatTransSd, deltaRot2Sd;
	double deltaRot1Hat, delatTransHat, deltaRot2Hat;
	Eigen::vector3d x << xPrev(0), xPrev(1), xPrev(2);

	deltaRot1 = std::atan2(u[1][1]-u[0][1]/u[1][0]-u[0][0]) - u[0][2];
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

	double weight = 0;
	lidarX = p.pose(0) + lidarOffsetX*cos(pose(2));
	lidarY = p.pose(1) + lidarOffsetY*sin(pose(2));
	lidarTheta = pose(2);
	for(int i = 0; i<noOfBeams; i++){


		weight += ;
	}
	return weight;
}

void Model::sampleNormalDistribution(double sigma){
	// Box-Muller Transform
	double x1, x2, w, r;
	do{

		// Generate a uniform distribution [0, 1]
		// Scale the Uniform distribution by 2 and left shift by 1 i.e Uniform Distribution [-1,1]
		do{
			// Generate a random number between [0,1]
			r = rand48();
		}while(r==0);
		// Project the number in distribution [-1, 1]
		x1 = 2*r -1;
		do{
			// Generate a random number between [0,1]
			r = rand48();
		}while(r==0)
		// Project the number in distribution [-1, 1]
		x2 = 2*r -1;
		w = x1*x1 + x2*x2;
	}while(w>1 || w==0);

	return sigma * x2 * std::sqrt(-2.0*std::log(w)/w);
}