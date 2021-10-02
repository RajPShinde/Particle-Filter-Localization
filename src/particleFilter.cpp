#include <particleFilter.hpp>

ParticleFilter::ParticleFilter(double n): noOfParticles_(n){
	// Create Particles
	while(n!=noOfParticles_){
	}
	drawParticles(particles);
}

ParticleFilter::~ParticleFilter(){
}

void ParticleFilter::odomCallback(const nav_msgs::Odometry msg){
	// To store last odom pose
	static double xPrev, yPrev, yawPrev;
	
	// Get yaw from odometry, quaternion-euler
	tf::Quaternion q;
	tf::quaternionMsgToTF(msg.pose.pose.orientation , q);
	q.normalize();
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	if(std::sqrt(std::pow(x-xPrev) + std::pow(y-yPrev))>0.01){
		// position
		double x = msg.pose.pose.position.x;
		double y = msg.pose.pose.position.y;
		
		// Prepare control for motion model
		std::vector<std::vector<double>> u = {{xPrev, yPrev, yawPrev}, {x, y, yaw}};
		
		// Store current pose
		xPrev = x;
		yPrev = y;
		yawPrev = yaw;

		// Propogate particles using motion model
		for(Particle p:particles_){
			Eigen::vector3d xPrev, x;
			xPrev = p.pose;
			x = model.motionModel(u, xPrev);
		}
	}
}

void ParticleFilter::scanCallback(){

}

void ParticleFilter::drawParticles(std::vector<Particle> particles){

}