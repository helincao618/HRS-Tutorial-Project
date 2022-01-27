#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
static const std::string OPENCV_WINDOW = "Top camera";
static const std::string OPENCV_WINDOW_2 = "Colour extraction";
static const std::string OPENCV_WINDOW_3 = "Blob extraction";
static const std::string OPENCV_WINDOW_4 = "Bottom camera";
static const std::string OPENCV_WINDOW_5 = "Blurred grayscale";
static const std::string OPENCV_WINDOW_6 = "Circular shapes";



using namespace cv;
using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Subscriber image_sub2_;
//  cv::Mat HSVImage;



public:
  cv::Mat HSVImage;
  cv::Mat ThreshImage;
  cv::Mat Image_blob;
  cv::Mat element;
  cv::Mat canny_output;
  cv::Mat drawing;

  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  int iLowH=0;
  int iHighH=179;

  int iLowS=0;
  int iHighS=255;

  int iLowV=0;
  int iHighV=255;


  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_sub2_ = it_.subscribe("/nao_robot/camera/bottom/camera/image_raw", 1,
      &ImageConverter::imageCb2, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

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

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    // RGB TO HSV
    //    cv::Mat HSVImage;
    //    cv::Mat ThreshImage;
    //cvtColor(cv_ptr->image,pGray , CV_RGB2GRAY);
    cvtColor(cv_ptr->image,HSVImage,CV_BGR2HSV);


//    cv::createTrackbar("lowH",OPENCV_WINDOW_2,&iLowH,179);
//    cv::createTrackbar("HighH",OPENCV_WINDOW_2,&iHighH,179);
//    cv::createTrackbar("LowS",OPENCV_WINDOW_2,&iLowS,255);
//    cv::createTrackbar("HighS",OPENCV_WINDOW_2,&iHighS,255);
//    cv::createTrackbar("LowV",OPENCV_WINDOW_2,&iLowV,255);
//    cv::createTrackbar("HighV",OPENCV_WINDOW_2,&iHighV,255);
//    inRange(HSVImage,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),ThreshImage);
//     inRange(HSVImage,cv::Scalar(0,118,227),cv::Scalar(0,255,255),ThreshImage); red
    inRange(HSVImage,cv::Scalar(106,80,74),cv::Scalar(138,255,255),ThreshImage); //blue
    cv::imshow(OPENCV_WINDOW_2, ThreshImage);
//  cv::dilate(ThreshImage,Image_blob,50);

    element = getStructuringElement(cv::MORPH_RECT,cv::Size(5,5));
    cv::erode(ThreshImage,Image_blob,element);
    //element = getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::dilate(Image_blob,Image_blob,1);
    Canny( Image_blob, canny_output, 30, 30, 3 );
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    vector<Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    {
      mu[i] = moments( contours[i], false );
    }
    vector<Point2f> mc( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    {
      mc[i] = Point2d( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
    }
    cvtColor(Image_blob,drawing,CV_GRAY2BGR);
      for( int i = 0; i< contours.size(); i++ )
      {
        Scalar color = Scalar( 255, 0, 0);
        //drawContours( Image_blob, contours, i, color, 2, 8, hierarchy, 0, Point() );
        circle( drawing, mc[i], 3, Scalar( 0, 0, 255), 1, 8, 0 );

        char tam[100];
        sprintf(tam, "(%0.0f,%0.0f)",mc[i].x,mc[i].y);
        putText(drawing, tam, Point(mc[i].x, mc[i].y), FONT_HERSHEY_SIMPLEX, 0.3, cvScalar(255,0,0),1);
      }



    cv::imshow(OPENCV_WINDOW_3, drawing);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

  }
  void imageCb2(const sensor_msgs::ImageConstPtr& msg)
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

    cv::imshow(OPENCV_WINDOW_4, cv_ptr->image);
    cv::Mat pGray=cv_ptr->image;
    cvtColor(cv_ptr->image,pGray , CV_RGB2GRAY);
    blur( pGray, pGray, Size(3,3) );
    cv::imshow( OPENCV_WINDOW_5, pGray);
    vector<Vec3f>circles;
    cv::HoughCircles(pGray,circles,HOUGH_GRADIENT,1,pGray.rows/16,100,40,30,60);
    for( size_t i = 0; i < circles.size(); i++ )
      {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          // circle center
          circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
          // circle outline
          circle( cv_ptr->image, center, radius, Scalar(0,0,255), 3, 8, 0 );
       }

    cv::imshow(OPENCV_WINDOW_6, cv_ptr->image);
    cv::waitKey(3);



    image_pub_.publish(cv_ptr->toImageMsg());
  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "tutorial_3");
  ImageConverter ic;




    ros::spin();
  return 0;
}


