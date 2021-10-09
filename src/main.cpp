#include <particleFilter.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "particleFilter");
    ros::NodeHandle nh;
    ParticleFilter localize(nh, 3000);
    ros::Rate loop(10);
    while(ros::ok()){
        ros::spinOnce();
        localize.localize();
        loop.sleep();
    }
    return 0;
}