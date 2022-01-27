#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <using_markers/obstacleDetect.h>
#include <using_markers/dest.h>
#include <geometry_msgs/Pose2D.h>

double x1,Y1,x2,y2,dx,dy;  // x1,Y1,x2,y2 ---- position of wypoints   dx,dy ---- position of destination
bool flag=0;
bool my_serviceServer1(using_markers::dest::Request &req,using_markers::dest::Response &res);
// receive the position of waypoints and destination position