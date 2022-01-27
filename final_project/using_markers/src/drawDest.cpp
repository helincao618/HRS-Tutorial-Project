//***************************Yansong Wu************************************
// This cpp aiming at drawing the target and the trajectory on the rviz. The
// trajectory is displayed by waypoints.
//***************************Yansong Wu************************************

#include <using_markers/drawDest.h>

class destDrawer
{
  ros::NodeHandle nh_;
  ros::NodeHandle it_;
  ros::Publisher marker_pub1;
  ros::Publisher marker_pub2;
  ros::Publisher marker_pub3;
  ros::Subscriber sub; 

  public:
  visualization_msgs::Marker marker1;
  visualization_msgs::Marker marker2;
  visualization_msgs::Marker marker3;
  
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  int i;
  

  destDrawer()
    : it_(nh_)
  {
    sub = it_.subscribe("naoposition", 5, &destDrawer::localizationCallback,this); // sub NAO's current position
    marker_pub1 = nh_.advertise<visualization_msgs::Marker>("visualization_marker8", 10); // draw destination
    marker_pub2 = nh_.advertise<visualization_msgs::Marker>("visualization_marker9", 10); // draw the 1st waypoint
    marker_pub3 = nh_.advertise<visualization_msgs::Marker>("visualization_marker10", 10); // draw the 2nd waypoint
    
    ROS_INFO_STREAM("+++++++++++++++++++++++++++++"); 
  }

  ~destDrawer()
  {
    ROS_INFO_STREAM("-----------------------------");
  }

  void localizationCallback(const geometry_msgs::Pose2D::ConstPtr &my_message)
  {
    
    //**************8
    sleep(0.01);
    if(flag)
    {
        // draw destination
        marker1.header.frame_id = "/ground";
        marker1.type = shape;        
        marker1.id = 8;
        marker1.action = visualization_msgs::Marker::ADD;
        marker1.pose.position.x = dx-0.6;
        marker1.pose.position.y = dy-0.5;
        marker1.pose.position.z = 0.025;
        marker1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
        
        marker1.scale.x = 0.05;
        marker1.scale.y = 0.05;
        marker1.scale.z = 0.05;

        marker1.color.r = 1.0f;
        marker1.color.g = 0.0f;
        marker1.color.b = 1.0f;
        marker1.color.a = 1.0;
        
        marker1.lifetime = ros::Duration();
        marker_pub1.publish(marker1);

        // draw the 1st waypoint
        marker2.header.frame_id = "/ground";       
        marker2.type = shape;
        marker2.header.stamp = ros::Time::now();
        marker2.ns = "basic_shape9";
        marker2.id = 9;
        marker2.action = visualization_msgs::Marker::ADD;
        marker2.pose.position.x = x1-0.6;
        marker2.pose.position.y = Y1-0.5;
        marker2.pose.position.z = 0.015;
        marker2.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
        
        marker2.scale.x = 0.03;
        marker2.scale.y = 0.03;
        marker2.scale.z = 0.03;

        marker2.color.r = 1.0f;
        marker2.color.g = 0.0f;
        marker2.color.b = 0.0f;
        marker2.color.a = 1.0;
        
        marker1.lifetime = ros::Duration();
        marker_pub2.publish(marker2); 


        // draw the 2nd waypoint
        marker3.header.frame_id = "/ground";       
        marker3.type = shape;
        marker3.header.stamp = ros::Time::now();
        marker3.ns = "basic_shape10";
        marker3.id = 10;
        marker3.action = visualization_msgs::Marker::ADD;
        marker3.pose.position.x = x2-0.6;
        marker3.pose.position.y = y2-0.5;
        marker3.pose.position.z = 0.06;
        marker3.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
        
        marker3.scale.x = 0.03;
        marker3.scale.y = 0.03;
        marker3.scale.z = 0.03;

        marker3.color.r = 1.0f;
        marker3.color.g = 0.0f;
        marker3.color.b = 0.0f;
        marker3.color.a = 1.0;
        
        marker3.lifetime = ros::Duration();
        marker_pub3.publish(marker3);

    }
    
    
    
      
        
    
  }


};

// receive the position of waypoints and destination position
bool my_serviceServer1(using_markers::dest::Request &req,using_markers::dest::Response &res)
{
    dx=req.dx;
    dy=req.dy;
    x1=req.x1;
    x2=req.x2;
    Y1=req.y1;
    y2=req.y2;
    flag=1;
    
    res.ret=1;
    return 1;
}



int main(int argc, char** argv)
{
    ros::init(argc,argv,"drawDest");
    ros::NodeHandle n;
    //posebroadcast();

    ros::ServiceServer my_service1 = n.advertiseService("drawDestt",my_serviceServer1);
    destDrawer de;
    ros::spin();    
    
    return 0;
    
}
