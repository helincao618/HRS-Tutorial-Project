/**************************************Helin Cao & Chengjie Yuan **************************************************************************************/
// Detecting the Aruco maker and publish the Rvec and Tvec of the marker for futher computation
/******************************************************************************************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video.hpp>
#include <aruco/aruco.h>
#include <std_srvs/Empty.h>
#include <speech/arucomsgs.h>
#include <tf/transform_broadcaster.h>
using namespace std;
using namespace cv;
using namespace aruco;

static const std::string OPENCV_WINDOW_4 = "3D marker position";

Mat src;

/********************************Convert the image to the cv form and do the marker detection*****************************************************/
class ImageConverter
{  
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  
  //Subscriber for the images from camera
  image_transport::Subscriber image_sub_;
  
  //Publisher for the Rvec and Tvec
  ros::Publisher arucomarker_pub_;

public:
  
  //constuctor
  ImageConverter()
    : it_(nh_)
  {
      // Subscribe to input video frame
      image_sub_ = it_.subscribe("/nao_robot/camera/bottom/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
      //Publish to the topic "detectedmarker"
	    arucomarker_pub_ = nh_.advertise<speech::arucomsgs>("detectedmarker",1000);
    	namedWindow(OPENCV_WINDOW_4);
  }

  ~ImageConverter()
  {
   
    destroyWindow(OPENCV_WINDOW_4);
  }
  
  //callback function for the subscriber: transport the image to CV form
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

      // create a variable in the class
      CameraParameters TheCameraParameters;

      // load the parameter matrix in the constructor
      Mat dist(1,5,CV_32FC1);
      dist.at<float>(0,0)=0.01630453814015667;
      dist.at<float>(0,1)=-0.3211887554012183;
      dist.at<float>(0,2)=0.0;
      dist.at<float>(0,3)=0.0;
      dist.at<float>(0,4)=0.5176028258435494;

      Mat cameraP(3,3,CV_32FC1);

      cameraP.at<float>(0,0)=280.8380624446123;
      cameraP.at<float>(0,1)=0.000000;
      cameraP.at<float>(0,2)=160.0;
      cameraP.at<float>(1,0)=0.000000;
      cameraP.at<float>(1,1)=280.3120563628607;
      cameraP.at<float>(1,2)=120.0;
      cameraP.at<float>(2,0)=0.000000;
      cameraP.at<float>(2,1)=0.000000;
      cameraP.at<float>(2,2)=1.000000;

      TheCameraParameters.setParams(cameraP,dist,Size(240,320));
      TheCameraParameters.resize( Size(240,320));
      
      /*3D MARKER POSITION*/
      aruco::MarkerDetector Detector;
 	    speech::arucomsgs detected_marker;

 	    float aruco_x, aruco_y, aruco_z,roll,pitch,yaw;
      std::vector< Marker >  detectedMarkers;
      Detector.detect(cv_ptr->image,detectedMarkers,TheCameraParameters,0.1,true);
      
      //Get the  Rvec and Tvec from the marker and publish them
      for (unsigned i = 0; i< detectedMarkers.size(); i++ )
        {
          detected_marker.id.resize(detectedMarkers.size());
          detected_marker.trafo.resize(detectedMarkers.size());
          detectedMarkers[i].draw(cv_ptr->image,Scalar(0,0,255),2);  
          detected_marker.id[i] = detectedMarkers[i].id;
          detected_marker.trafo[i].linear.x = detectedMarkers[i].Tvec.at<float>(0);
          detected_marker.trafo[i].linear.y = detectedMarkers[i].Tvec.at<float>(1);
          detected_marker.trafo[i].linear.z = detectedMarkers[i].Tvec.at<float>(2);
          detected_marker.trafo[i].angular.x = detectedMarkers[i].Rvec.at<float>(0);
          detected_marker.trafo[i].angular.y = detectedMarkers[i].Rvec.at<float>(1);
          detected_marker.trafo[i].angular.z = detectedMarkers[i].Rvec.at<float>(2);
          std::cout << "Tvec" <<detectedMarkers[0].Tvec << std::endl;
	        std::cout << "Rvec" <<detectedMarkers[0].Rvec << std::endl;
          aruco_x = detectedMarkers[0].Tvec.at<float>(0);
          aruco_y = detectedMarkers[0].Tvec.at<float>(1);
          aruco_z = detectedMarkers[0].Tvec.at<float>(2);
          roll = detectedMarkers[0].Rvec.at<float>(0);
          pitch = detectedMarkers[0].Rvec.at<float>(1);
          yaw = detectedMarkers[0].Rvec.at<float>(2);
	  
    // print the markerframe in rviz
	  static tf::TransformBroadcaster br;
          tf::Transform transform;
          transform.setOrigin( tf::Vector3(aruco_x, aruco_y, aruco_z) );
          tf::Quaternion q;
          q.setRPY(roll, pitch, yaw);
          transform.setRotation(q);
          br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "CameraBottom_optical_frame", "aruco_marker"));
        }
        
        imshow(OPENCV_WINDOW_4, cv_ptr->image);
        waitKey(3);   
	      arucomarker_pub_.publish(detected_marker);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_dect");
  ros::NodeHandle n;
  ImageConverter ic;
  ros::spin();
  return 0;
}

