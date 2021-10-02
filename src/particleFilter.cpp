#include <particleFilter.hpp>

ParticleFilter::ParticleFilter(double n): noOfParticles_(n){

	odomSub_ = ;
	scanSub_ = ;
	particlePub_ = ;
	
	// Create the motion and measurement models
	Model model();
	
	// Create Particles
	particlePoses_.resize(noOfParticles_);
	while(n!=noOfParticles_){
	}
	
	// Draw initial Particles
	drawParticles();
	
	// Run Algorithm
	localize();
}

ParticleFilter::~ParticleFilter(){
}

void ParticleFilter::odomCallback(const nav_msgs::Odometry msg){
	odomData_ = msg;
}

void ParticleFilter::scanCallback(const sensor_msgs::LaserScan msg){
	scanData_ = msg;
}

void ParticleFilter::normalize(double totalWeight){
	for(Particle &p:particles_){
		p.weight /= totalWeight;
	}
}

void ParticleFilter::drawParticles(){
	geometry_msgs::pose pose;
	int i = 0;

	for(Particle p:particles_){
		pose.x = p.pose(0);
		pose.y = p.pose(1);
		pose.z = 0;

		tf::Quaternion q;
		q.setRPY(0, 0, p.pose(2));
		q.normalize();
		tf::quaternionTFToMsg(qNew, pose.orientation);

		particlePoses_[i] = pose;
		i++;
	}
	particlePub_.publish(particlePoses_);
}

void ParticleFilter::localize(){

	while(true){

		// To store last odom pose
		static double xPrev, yPrev, yawPrev;
		
		// Get yaw from odometry, quaternion-euler
		tf::Quaternion q;
		tf::quaternionMsgToTF(odomData_.pose.pose.orientation , q);
		q.normalize();
		double roll, pitch, yaw;
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		if(std::sqrt(std::pow(x-xPrev) + std::pow(y-yPrev))>0.01){
			// position
			double x = odomData_.pose.pose.position.x;
			double y = odomData_.pose.pose.position.y;
			
			// Prepare control for motion model
			std::vector<std::vector<double>> u = {{xPrev, yPrev, yawPrev}, {x, y, yaw}};
			
			// Store current pose
			xPrev = x;
			yPrev = y;
			yawPrev = yaw;

			// Propogate particles using motion model
			std::vector<Particle> tempParticles;
			for(Particle p:particles_){
				Eigen::vector3d xPrev, x;
				Particle temp;
				xPrev = p.pose;
				x = model.motionModel(u, xPrev);
				temp.pose = x;
				tempParticles.push_back(temp);
			}

			particles_ = tempParticles;


		}

		for(Particle p:particles_){
			weight = model.measurementModel();
			totalWeight_ += weight;
		}

		normalize();

		drawParticles();
	}
}