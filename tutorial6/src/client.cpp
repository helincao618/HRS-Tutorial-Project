#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <cstring>

#include <ros/ros.h>

#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include "sensor_msgs/JointState.h"
#include "nao_control_tutorial_2/MoveJoints.h"
#include <string.h>
#include "nao_control_tutorial_2/coordinate.h"
#include <tf/transform_broadcaster.h>

class Nao_control
{
public:
    // ros handler
    ros::NodeHandle nh_;

    // Image transport and subscriber:
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::Mat image;

    // Aruco camera parameters
    aruco::CameraParameters cameraParameters;

    // Aruco marker parameters
    float aruco_x, aruco_y, aruco_z,roll,pitch,yaw;
    float markerSize;
    

    Nao_control() : it_(nh_)
    {
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 500, &Nao_control::imageCallBack, this);
        
        //Nao_control::exercise2("RArm");
    }

    ~Nao_control()
    {
    }

    // Image callback function is executed when a new image is received:
    void imageCallBack(const sensor_msgs::ImageConstPtr& msg) 
    {
        ROS_INFO_STREAM("Image callback execution");
        cv_bridge::CvImageConstPtr cvImagePointer;
	    try 
	    {
            cvImagePointer = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
	        image = cvImagePointer->image.clone();
        }
	    catch (cv_bridge::Exception& except) 
	    {
	        ROS_ERROR("cv_bridge exception: %s", except.what());
            return;
        }

	    // Distortion matrix:
        cv::Mat dist = (cv::Mat_<double>(4, 1) <<
                -0.0870160932911717,
                0.128210165050533,
                0.003379500659424,
                -0.00106205540818586);

	    // Camera matrix:
        cv::Mat cameraP = (cv::Mat_<double>(3, 3) <<
                274.139508945831, 0.0, 141.184472810944,
                0.0, 275.741846757374, 106.693773654172,
                0.0, 0.0, 1.0);

	    cameraParameters.setParams(cameraP,dist,cv::Size(640,480));
        aruco::MarkerDetector arucoDetector;
        std::vector<aruco::Marker> arucoMarkers;
        arucoDetector.detect(image, arucoMarkers);

        if (arucoMarkers.size() > 0) 
	    {
	        ROS_WARN_STREAM("I could detect "<<arucoMarkers.size()<<" aruco marker(s).");
	        markerSize = 0.064;
	        arucoMarkers[0].calculateExtrinsics(markerSize, cameraParameters, true);
            arucoMarkers[0].draw(image, cv::Scalar(0,0,255), 2);
            aruco_x = arucoMarkers[0].Tvec.at<float>(0);
            aruco_y = arucoMarkers[0].Tvec.at<float>(1);
            aruco_z = arucoMarkers[0].Tvec.at<float>(2);
            roll = arucoMarkers[0].Rvec.at<float>(0);
            pitch = arucoMarkers[0].Rvec.at<float>(1);
            yaw = arucoMarkers[0].Rvec.at<float>(2);
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin( tf::Vector3(aruco_x, aruco_y, aruco_z) );
            tf::Quaternion q;
            q.setRPY(roll, pitch, yaw);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "CameraTop_optical_frame", "aruco_marker"));

        }
	    else
	    {
	    ROS_WARN_STREAM("No aruco marker detected");
            aruco_x = 0.0;
            aruco_y = 0.0;
            aruco_z = 0.0;
            roll = 0.0;
            pitch = 0.0;
            yaw = 0.0;
	    }
      ROS_INFO_STREAM("aruco_x ="<<aruco_x);
      ROS_INFO_STREAM("aruco_y ="<<aruco_y);
      ROS_INFO_STREAM("aruco_z ="<<aruco_z);
      ROS_INFO_STREAM("aruco_R ="<<roll);
      ROS_INFO_STREAM("aruco_P ="<<pitch);
      ROS_INFO_STREAM("aruco_Y ="<<yaw);


	    // Display marker
      cv::imshow("marker", image);
      cv::waitKey(3);



      ros::ServiceClient movejointaruco_client = nh_.serviceClient <nao_control_tutorial_2::MoveJoints>("MoveJoint_aruco");
      nao_control_tutorial_2::MoveJoints srv;
      srv.request.name="RArm";
      srv.request.desired.linear.x = aruco_x;
      srv.request.desired.linear.y = aruco_y;
      srv.request.desired.linear.z = aruco_z;
      srv.request.desired.angular.x = roll;
      srv.request.desired.angular.y = pitch;
      srv.request.desired.angular.z = yaw;
      if (movejointaruco_client.call(srv))
          {
            ROS_INFO("Service called");

          }
        else
          {
            ROS_ERROR("srv3 Failed");
          }



    }
    // TODO: create function for each task
    void exercise1_2(string name)
    {
      ros::ServiceClient coordinate_client = nh_.serviceClient <nao_control_tutorial_2::coordinate>("coordinate");
      nao_control_tutorial_2::coordinate srv;
      
      srv.request.name=name;

      if (coordinate_client.call(srv))
          {
            ROS_INFO("Service called");
            ROS_INFO_STREAM(srv.response);
          }
        else
          {
            ROS_ERROR("srv1 Failed");
          }

    }



    void exercise1_3(string name, vector<double> gesture)
    {
      ros::ServiceClient movejoint_client = nh_.serviceClient <nao_control_tutorial_2::MoveJoints>("MoveJoint");
      nao_control_tutorial_2::MoveJoints srv;
      
      srv.request.name=name;
      srv.request.velocity=0.35;
      srv.request.desired.linear.x = gesture[0];
      srv.request.desired.linear.y = gesture[1];
      srv.request.desired.linear.z = gesture[2];
      srv.request.desired.angular.x = gesture[3];
      srv.request.desired.angular.y = gesture[4];
      srv.request.desired.angular.z = gesture[5];

      if (movejoint_client.call(srv))
          {
            ROS_INFO("Service called");
            ROS_INFO_STREAM(srv.response);
          }
        else
          {
            ROS_ERROR("srv2 Failed");
          }
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nao_tutorial_control_2");

    Nao_control my_NAO_ControlInstance;
    my_NAO_ControlInstance.exercise1_2("RArm");

    vector<double> gesture;
    gesture.push_back(0.112559);
    gesture.push_back(-0.000288848);
    gesture.push_back(0.171656);
    gesture.push_back(-1.69423);
    gesture.push_back(-0.487957);
    gesture.push_back(-1.37036);
    my_NAO_ControlInstance.exercise1_3("LArm",gesture);


    ros::spin();
    return 0;

}
