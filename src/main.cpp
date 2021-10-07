#include <particleFilter.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "particleFilter");
    ros::NodeHandle nh;
    ParticleFilter localize(nh, 3000);
    localize.localize();
    ros::Rate loop(50);
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}