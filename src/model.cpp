#include <model.hpp>

Model::Model(){
	
}

Model::~Model(){

}

Eigen::vecotr2d Model::motionModel(double u, double xPrev){
	return x;
}

double Model::measurementModel(){
	return weight;
}