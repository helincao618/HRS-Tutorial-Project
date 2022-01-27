#include <detection/detection.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection");
    ros::NodeHandle nh;
    ros::NodeHandle nh_2;
    detectionSpace::Detection node(nh, nh_2);
    ros::spin();
    return 0;
}