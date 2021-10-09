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
		double yaw = (rand()/(double)RAND_MAX)*(2*pi_);
		fromPiToMinusPi(yaw);
		if((int)(x/map_.resolution) == map_.xMax || (int)(y/map_.resolution) == map_.yMax)
			continue;
		if(map_.data[(int)(y/map_.resolution)][(int)(x/map_.resolution)]!=0)
			continue;
		Particle p;
		p.pose << x, y, yaw;
		// p.pose << 13, 25, 0;
		p.weight = 1/noOfParticles_;
		particles_.push_back(p);
		n++;
	}

	std::chrono::milliseconds timespan(2000);
	std::this_thread::sleep_for(timespan);
	
	// Draw initial Particles
	publishPose();

	initialized_ = true;

}

ParticleFilter::~ParticleFilter(){
}

void ParticleFilter::odomCallback(const nav_msgs::Odometry msg){
	odomData_ = msg;
	odomReceived_ = true;
}

void ParticleFilter::scanCallback(const sensor_msgs::LaserScan msg){
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
	for(int y=0; y<map_.yMax; y++){
		std::vector<int> row;
		for(int x=0; x<map_.xMax; x++){
			row.push_back((int)msg.data[i]);
			if((int)msg.data[i]==100)
				occupied_.push_back(std::make_pair(y,x));
			else if((int)msg.data[i]==0)
				free_.push_back(std::make_pair(y,x));
			i++;
		}
		map_.data.push_back(row);
	}
	distances();
	initializeParticles();
}

void ParticleFilter::distances(){
	std::vector<std::vector<double>> temp(map_.yMax, std::vector<double>(map_.xMax, INT_MAX));
	distances_ = temp;

	for(int i=0; i<free_.size(); i++){
		for(int j=0; j<occupied_.size(); j++){
			double distance = std::pow(free_[i].first-occupied_[j].first,2) + std::pow(free_[i].second-occupied_[j].second,2);
			if(distances_[free_[i].second][free_[i].first] > distance){
				distances_[free_[i].second][free_[i].first] = distance;
			}
		}
	}

	ROS_INFO_STREAM(distances_[24][12]);
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

void ParticleFilter::publishPose(){
	geometry_msgs::Pose pose;
	int i = 0;
	double x, y, yaw;
	for(Particle p:particles_){

		x += p.pose(0);
		y += p.pose(1);
		yaw += p.pose(2);

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

		if(std::sqrt(std::pow(x-xPrev, 2) + std::pow(y-yPrev, 2))>0.01){
			
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
				// double weight = model_.measurementModel(p, scanData_, map_);
				double weight = model_.measurementModelLikelihoodField(p, scanData_,map_, distances_);
				p.weight = weight;
				totalWeight += weight;
			}

			normalize(totalWeight);

			resample();

			publishPose();
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
		// particles_[p].weight = 1/noOfParticles_;
	}

}