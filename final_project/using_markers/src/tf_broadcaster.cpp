//*************************** Yansong Wu ************************************
// This cpp aiming at drawing the ground and obstacles on the rviz. A tf 
// named /ground is broadcasted here, which works as the repesct frame of the 
// whole map.
//*************************** Yansong Wu ************************************

#include <using_markers/tfObstacle.h>


class obDrawer
{
  ros::NodeHandle nh_;
  ros::NodeHandle it_;
  ros::Publisher marker_pub1;
  ros::Publisher marker_pub2;
  ros::Publisher marker_pub3;
  ros::Publisher marker_pub4;
  ros::Subscriber sub; 

  public:
  visualization_msgs::Marker marker1;
  visualization_msgs::Marker marker2;
  visualization_msgs::Marker marker3;
  visualization_msgs::Marker marker4;
  uint32_t shape = visualization_msgs::Marker::CUBE;
  int i;
  

  obDrawer()
    : it_(nh_)
  {
    sub = it_.subscribe("naoposition", 5, &obDrawer::localizationCallback,this);
    marker_pub1 = nh_.advertise<visualization_msgs::Marker>("visualization_marker1", 10); // draw the 1st obstacle
    marker_pub2 = nh_.advertise<visualization_msgs::Marker>("visualization_marker2", 10); // draw the 2nd obstacle
    marker_pub3 = nh_.advertise<visualization_msgs::Marker>("visualization_marker3", 10); // draw the 3rd obstacle
    marker_pub4 = nh_.advertise<visualization_msgs::Marker>("visualization_marker4", 10); // draw the ground
    ROS_INFO_STREAM("+++++++++++++++++++++++++++++");
  }

  ~obDrawer()
  {
    ROS_INFO_STREAM("-----------------------------");
  }

  // draw the obstales and ground, whenever the localization infos is subscribed
  void localizationCallback(const geometry_msgs::Pose2D::ConstPtr &my_message)
  {
    //ROS_INFO_STREAM(*my_message);
    naox=my_message->x;
    naoy=my_message->y;
    naotheta=my_message->theta;
    // ROS_INFO_STREAM(ox[0]);
    // ROS_INFO_STREAM(ox[1]);
    // ROS_INFO_STREAM(ox[2]);


    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(naox-0.6,naoy-0.5,0));
    tf::Quaternion q;
    q.setRPY(0,0,-naotheta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/base_footprint","/ground"));

    
    //draw the 1st obstacle
    uint32_t shape = visualization_msgs::Marker::CUBE;
    marker1.header.frame_id = "/ground";       
    marker1.type = shape;
    marker1.header.stamp = ros::Time::now();
    marker1.ns = "basic_shape4";
    marker1.id = 4;
    marker1.action = visualization_msgs::Marker::ADD;
    marker1.pose.position.x = ox[0]-0.6;
    marker1.pose.position.y = oy[0]-0.5;
    marker1.pose.position.z = 0.075;
    marker1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    
    marker1.scale.x = 0.07;
    marker1.scale.y = 0.10;
    marker1.scale.z = 0.15;

    marker1.color.r = 0.0f;
    marker1.color.g = 1.0f;
    marker1.color.b = 0.0f;
    marker1.color.a = 1.0;
    
    marker1.lifetime = ros::Duration();
    marker_pub1.publish(marker1);

    // draw the 2nd obstacle
    marker2.header.frame_id = "/ground";       
    marker2.type = shape;
    marker2.header.stamp = ros::Time::now();
    marker2.ns = "basic_shape5";
    marker2.id = 5;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.position.x = ox[1]-0.6;
    marker2.pose.position.y = oy[1]-0.5;
    marker2.pose.position.z = 0.06;
    marker2.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    
    marker2.scale.x = 0.03;
    marker2.scale.y = 0.13;
    marker2.scale.z = 0.12;

    marker2.color.r = 0.0f;
    marker2.color.g = 1.0f;
    marker2.color.b = 0.0f;
    marker2.color.a = 1.0;
    
    marker1.lifetime = ros::Duration();
    marker_pub2.publish(marker2); 


    // draw the 3rd obstacle 
    marker3.header.frame_id = "/ground";       
    marker3.type = shape;
    marker3.header.stamp = ros::Time::now();
    marker3.ns = "basic_shape6";
    marker3.id = 6;
    marker3.action = visualization_msgs::Marker::ADD;
    marker3.pose.position.x = ox[2]-0.6;
    marker3.pose.position.y = oy[2]-0.5;
    marker3.pose.position.z = 0.06;
    marker3.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    
    marker3.scale.x = 0.03;
    marker3.scale.y = 0.13;
    marker3.scale.z = 0.12;

    marker3.color.r = 0.0f;
    marker3.color.g = 1.0f;
    marker3.color.b = 0.0f;
    marker3.color.a = 1.0;
    
    marker3.lifetime = ros::Duration();
    marker_pub3.publish(marker3);

    // draw the ground   
    marker4.header.frame_id = "/ground";       
    marker4.type = shape;
    marker4.header.stamp = ros::Time::now();
    marker4.ns = "ground";
    marker4.id = 7;
    marker4.action = visualization_msgs::Marker::ADD;
    marker4.pose.position.x = 0;
    marker4.pose.position.y = 0;
    marker4.pose.position.z = 0;
    marker4.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
    
    marker4.scale.x = 1.4;
    marker4.scale.y = 1.2;
    marker4.scale.z = 0.0;

    marker4.color.r = 55.0f;
    marker4.color.g = 55.0f;
    marker4.color.b = 55.0f;
    marker4.color.a = 1.0;
    
    marker4.lifetime = ros::Duration();
    marker_pub4.publish(marker4);
      
        
    
  }


};

// draw the obstales and ground, for the first time it recieve localiztion and obstacles infos from service 
void obstaclepublisher()
{
  ros::NodeHandle nn;
  //draw the 1st obstacle
  ros::Publisher marker_pub1;
  marker_pub1 = nn.advertise<visualization_msgs::Marker>("visualization_marker1", 10);
  visualization_msgs::Marker marker1;
  uint32_t shape = visualization_msgs::Marker::CUBE;
  marker1.header.frame_id = "/ground";       
  marker1.type = shape;
  marker1.header.stamp = ros::Time::now();
  marker1.ns = "basic_shape4";
  marker1.id = 4;
  marker1.action = visualization_msgs::Marker::ADD;
  marker1.pose.position.x = ox[0]-0.6;
  marker1.pose.position.y = oy[0]-0.5;
  marker1.pose.position.z = 0.075;
  marker1.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  
  marker1.scale.x = 0.07;
  marker1.scale.y = 0.10;
  marker1.scale.z = 0.15;

  marker1.color.r = 0.0f;
  marker1.color.g = 1.0f;
  marker1.color.b = 0.0f;
  marker1.color.a = 1.0;
  
  marker1.lifetime = ros::Duration();
  marker_pub1.publish(marker1);

  //draw the 2nd obstacle
  ros::Publisher marker_pub2;
  marker_pub2 = nn.advertise<visualization_msgs::Marker>("visualization_marker2", 10);
  visualization_msgs::Marker marker2;
  
  marker2.header.frame_id = "/ground";       
  marker2.type = shape;
  marker2.header.stamp = ros::Time::now();
  marker2.ns = "basic_shape5";
  marker2.id = 5;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = ox[1]-0.6;
  marker2.pose.position.y = oy[1]-0.5;
  marker2.pose.position.z = 0.06;
  marker2.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  
  marker2.scale.x = 0.03;
  marker2.scale.y = 0.13;
  marker2.scale.z = 0.12;

  marker2.color.r = 0.0f;
  marker2.color.g = 1.0f;
  marker2.color.b = 0.0f;
  marker2.color.a = 1.0;
  
  marker1.lifetime = ros::Duration();
  marker_pub2.publish(marker2);


  //draw the 3rd obstacle
  ros::Publisher marker_pub3;
  marker_pub3 = nn.advertise<visualization_msgs::Marker>("visualization_marker3", 10);
  visualization_msgs::Marker marker3;
  
  marker3.header.frame_id = "/ground";       
  marker3.type = shape;
  marker3.header.stamp = ros::Time::now();
  marker3.ns = "basic_shape6";
  marker3.id = 6;
  marker3.action = visualization_msgs::Marker::ADD;
  marker3.pose.position.x = ox[2]-0.6;
  marker3.pose.position.y = oy[2]-0.5;
  marker3.pose.position.z = 0.06;
  marker3.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
  
  marker3.scale.x = 0.03;
  marker3.scale.y = 0.13;
  marker3.scale.z = 0.12;

  marker3.color.r = 0.0f;
  marker3.color.g = 1.0f;
  marker3.color.b = 0.0f;
  marker3.color.a = 1.0;
  
  marker3.lifetime = ros::Duration();
  marker_pub3.publish(marker3);
}

// broadcast a tf named /ground, which is the basis of the whole map
void groundbroadcast()
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(naox-0.6,naoy-0.5,0));
    tf::Quaternion q;
    q.setRPY(0,0,-naotheta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"/base_footprint","/ground"));

    obstaclepublisher();
}


// receive the nao position and obstacles position
bool my_serviceServer1(using_markers::obstacleDetect::Request &req,using_markers::obstacleDetect::Response &res)
{    
   
    ROS_INFO_STREAM(req);
    // obstacle position
    ox[0]=req.obx1;
    ox[1]=req.obx2;
    ox[2]=req.obx3;
    oy[0]=req.oby1;
    oy[1]=req.oby2;
    oy[2]=req.oby3;
    // nao pose
    naox=req.naox;
    naoy=req.naoy;
    naotheta=req.naotheta;
    res.ret=1;
    groundbroadcast();
    obstaclepublisher();

    launchflag=1;
    obDrawer ob;
    ros::spin();
    return 1;
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"my_tf_broadcaster");
    ros::NodeHandle n;
    
    // receive the nao position and obstacles position
    ros::ServiceServer my_service1 = n.advertiseService("drawmap",my_serviceServer1);
    
    ros::spin();    
    
    return 0;
    
}