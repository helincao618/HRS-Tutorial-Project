#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <using_markers/obstacleDetect.h>
#include <geometry_msgs/Pose2D.h>

#define PI 3.1415926

bool launchflag=0; 
double ox[3],oy[3]; // obstacle position wrt world
double naox,naoy,naotheta; // nao position wrt world
double x_dest,y_dest, dist_dest, theta_dest; // destination position wrt world

bool my_serviceServer1(using_markers::obstacleDetect::Request &req,using_markers::obstacleDetect::Response &res);
// receive the nao position and obstacles position

void groundbroadcast();
// broadcast the tf named /ground, which is the basis of the whole map

void obstaclepublisher();
// draw the obstales and ground, for the first time it recieve localiztion and obstacles infos from service 

