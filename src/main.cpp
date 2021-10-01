#include <particleFilter.hpp>

int main(int argc, char **argv){
    ros::init(argc, argv, "particleFilter")
    ros::NodeHandle nh;
    while(ros::ok()){
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}