#include <particleFilter.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "particleFilter");
    ros::NodeHandle nh;
    ParticleFilter localize(nh, 2000);
    ros::Rate loop(20);
    while(ros::ok()){
        ros::spinOnce();
        localize.localize();
        loop.sleep();
    }
    return 0;
}