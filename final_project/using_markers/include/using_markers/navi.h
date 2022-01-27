#include <ros/ros.h>

#include <math.h>
#include <cmath>
#include <detection/mode1.h>
#include <speech/walk_polar.h>
#include <using_markers/localization.h>
#include <using_markers/obstacleDetect.h>
#include <using_markers/destination.h>
#include <using_markers/dest.h>

#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#define PI 3.1415926
#define LENGTH 1
#define WIDTH 1.2
#define r0 0.20
#define r1 0.20
#define r2 0.20
#define NOTEXIST 1000000

bool launchflag=0; //navigation part only works, while launchflag=1
double ox[3],oy[3]; // obstacle position wrt world
double naox,naoy,naotheta; // nao position wrt world
double x_dest,y_dest, dist_dest, theta_dest; // destination position wrt world


double distanceToLine(double pX, double pY, double x0, double y0, double x1, double y1);
// distance between point and segment

bool ifobstcale(double px[3],double py[3],double destx,double desty, double localx,double localy,double localtheta);
// detect if there is obstacle on the way to target

double *navi(double px[3],double py[3],double destx,double desty, double localx,double localy,double localtheta);
// trajectory planning

bool my_serviceServer1(using_markers::obstacleDetect::Request &req,using_markers::obstacleDetect::Response &res);
// receive the position of obstacles and the NAO's position wrt world

bool my_serviceServer2(using_markers::destination::Request &req,using_markers::destination::Response &res);
// receive the position of target wrt NAO