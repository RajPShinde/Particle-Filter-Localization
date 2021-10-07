#include <particleFilter.hpp>

ParticleFilter::ParticleFilter(ros::NodeHandle &nh, int n): filter_(nh), noOfParticles_(n){

	odomSub_ = filter_.subscribe("/odom", 1, &ParticleFilter::odomCallback, this);
	scanSub_ = filter_.subscribe("/scan", 1, &ParticleFilter::scanCallback, this);
	mapSub_ = filter_.subscribe("/map", 1, &ParticleFilter::mapCallback, this);
	particlePub_ = filter_.advertise<geometry_msgs::PoseArray>("/particles", 1);
	scanPub_ = filter_.advertise<sensor_msgs::LaserScan>("/scan2", 1);
}

void ParticleFilter::initializeParticles(){
	// Create Particles
	particlePoses_.header.frame_id = "map";
	particlePoses_.header.stamp = ros::Time::now();
	particlePoses_.poses.resize(noOfParticles_);
	int n = 0;
	while(n!=noOfParticles_){
		double x = (rand()/(double)RAND_MAX)*(map_.xWidth*map_.resolution) + map_.xMin*map_.resolution;
		double y = (rand()/(double)RAND_MAX)*(map_.yWidth*map_.resolution) + map_.yMin*map_.resolution;
		ROS_INFO_STREAM(x<<" "<<std::round(x)/map_.resolution);
		// double x = (rand()/(double)RAND_MAX)*map_.xWidth + map_.xMin;
		// double y = (rand()/(double)RAND_MAX)*map_.yWidth + map_.yMin;
		double yaw = (rand()/(double)RAND_MAX)*(2*pi_);
		fromPiToMinusPi(yaw);
		if((int)(x/map_.resolution) == map_.xMax || (int)(y/map_.resolution) == map_.yMax)
			continue;
		if(map_.data[(int)(y/map_.resolution)][(int)(x/map_.resolution)]!=0)
			continue;
		// if(map_.data[x][y]<=0)
		// 	continue;
		Particle p;
		p.pose << x, y, yaw;
		p.weight = 1/noOfParticles_;
		particles_.push_back(p);
		n++;
	}
	
	// Draw initial Particles
	drawParticles();

	initialized_ = true;

}

ParticleFilter::~ParticleFilter(){
}

void ParticleFilter::odomCallback(const nav_msgs::Odometry msg){
	ROS_INFO_STREAM("Odom");
	odomData_ = msg;
	odomReceived_ = true;
}

void ParticleFilter::scanCallback(const sensor_msgs::LaserScan msg){
	ROS_INFO_STREAM("Scan");
	scanData_ = msg;
	scanReceived_ = true;
	sensor_msgs::LaserScan temp = msg;
	temp.header.stamp = ros::Time::now();
	scanPub_.publish(temp);
}

void ParticleFilter::mapCallback(const nav_msgs::OccupancyGrid msg){
	map_.resolution = msg.info.resolution;
	map_.xWidth = msg.info.width;
	map_.yWidth = msg.info.height;
	map_.xMin = 0;
	map_.yMin = 0;
	map_.xMax = map_.xWidth;
	map_.yMax = map_.yWidth;

	int i = 0;
	for(int x=0; x<map_.xMax; x++){
		std::vector<int> row;
		for(int y=0; y<map_.yMax; y++){
			row.push_back((int)msg.data[i]);
			i++;
		}
		map_.data.push_back(row);
	}
	ROS_INFO_STREAM("MAP");
	initializeParticles();
}

void ParticleFilter::fromPiToMinusPi(double &angle){
	if(angle > pi_)
		angle += -2*pi_ ;
	else if(angle < -pi_)
		angle += 2*pi_;
}

void ParticleFilter::normalize(double totalWeight){
	for(Particle &p:particles_){
		p.weight /= totalWeight;
	}
}

void ParticleFilter::drawParticles(){
	geometry_msgs::Pose pose;
	int i = 0;

	for(Particle p:particles_){
		pose.position.x = p.pose(0);
		pose.position.y = p.pose(1);
		pose.position.z = 0;

		tf::Quaternion q;
		q.setRPY(0, 0, p.pose(2));
		q.normalize();
		tf::quaternionTFToMsg(q, pose.orientation);

		particlePoses_.poses[i] = pose;
		i++;
	}
	particlePub_.publish(particlePoses_);
}

void ParticleFilter::publishPose(){

	double x, y, yaw;
	for(Particle p:particles_){
		x += p.pose(0);
		y += p.pose(1);
		yaw += p.pose(2);
	}
	x /= noOfParticles_;
	y /= noOfParticles_;
	yaw /= noOfParticles_;

	tf::Quaternion q;
	q.setRPY(0, 0, yaw);

	static tf::TransformBroadcaster br;
	tf::Transform t;
	t.setOrigin(tf::Vector3(x, y, 0));
	t.setRotation(q);
	br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "/map", "/base_link"));
}

void ParticleFilter::localize(){


	Model model_(alpha1_, alpha2_, alpha3_, alpha4_, zHit_, zShort_, zRand_, zMax_, sigmaHit_, lambdaShort_);
	if(initialized_ && odomReceived_ && scanReceived_){
		ros::spinOnce();
		ROS_INFO_STREAM("Localizing");
		// To store last odom pose
		static double xPrev, yPrev, yawPrev;
		
		// Get yaw from odometry, quaternion-euler
		tf::Quaternion q;
		tf::quaternionMsgToTF(odomData_.pose.pose.orientation , q);
		q.normalize();
		double roll, pitch, yaw;
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		// Get x, y (position) from odometry
		double x = odomData_.pose.pose.position.x;
		double y = odomData_.pose.pose.position.y;

		if(std::sqrt(std::pow(x-xPrev, 2) + std::pow(y-yPrev, 2))>0.005){
			ROS_INFO_STREAM("Localizing In"); 
			
			// Prepare control for motion model
			std::vector<std::vector<double>> u = {{xPrev, yPrev, yawPrev}, {x, y, yaw}};
			
			// Store current pose
			xPrev = x;
			yPrev = y;
			yawPrev = yaw;

			// Propogate particles using motion model
			std::vector<Particle> tempParticles;
			for(Particle p:particles_){
				Eigen::Vector3d xPrev, x;
				Particle temp;
				xPrev = p.pose;
				x = model_.motionModel(u, xPrev);
				temp.pose = x;
				tempParticles.push_back(temp);
			}

			particles_ = tempParticles;

			double totalWeight = 0;
			for(Particle &p:particles_){
				double weight = model_.measurementModel(p, scanData_, map_);
				p.weight = weight;
				totalWeight += weight;
			}

			normalize(totalWeight);

			resample();

			publishPose();

			drawParticles();
		}
	}
}

void ParticleFilter::resample(){

	std::vector<Particle> particlesTemp = particles_;
	double r = (rand()/(double)RAND_MAX) * (1/noOfParticles_);
	double c = particles_[0].weight;
	int i = 0;

	for(int p=0; p<noOfParticles_; p++){
		double u = r + (double)p/noOfParticles_;
		while(u>c && p<noOfParticles_){
			i++;
			c += particlesTemp[i].weight;
		}
		particles_[p] = particlesTemp[i];
		particles_[p].weight = 1/noOfParticles_;
	}

}