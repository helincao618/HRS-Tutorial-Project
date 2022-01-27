#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video.hpp>
#include <aruco/aruco.h>

using namespace std;
using namespace cv;
using namespace aruco;
static const std::string OPENCV_WINDOW_0 = "Track window camshift";
static const std::string OPENCV_WINDOW_1 = "ROI image";
static const std::string OPENCV_WINDOW_2 = "Probability image";
static const std::string OPENCV_WINDOW_3 = "Optical flow";
static const std::string OPENCV_WINDOW_4 = "3D marker position";

Mat src;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  vector<Point2f> p0;
  Mat ImageOld;
  bool flag=false;
  Mat mask;
  
  




  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
      &ImageConverter::imageCb, this);

    namedWindow(OPENCV_WINDOW_0);
    namedWindow(OPENCV_WINDOW_1);
    namedWindow(OPENCV_WINDOW_2);
    namedWindow(OPENCV_WINDOW_3);
    namedWindow(OPENCV_WINDOW_4);
  }

  ~ImageConverter()
  {
    destroyWindow(OPENCV_WINDOW_0);
    destroyWindow(OPENCV_WINDOW_1);
    destroyWindow(OPENCV_WINDOW_2);
    destroyWindow(OPENCV_WINDOW_3);
    destroyWindow(OPENCV_WINDOW_4);
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

    // create a ROI
    Rect myROI(174,35,36,90);
    // parameters
    Mat orginImage; //camera image
    Mat HSVImage;   //camera image in HSV form
    Mat loadedImage; // image of templateImg.jpg
    Mat LoadHSVImage; // hsv + ROI + loadedImage
    vector<Mat> splitedImage;
    vector<Mat> store_img;
    Mat maskImage;
    Mat LoadHSVImage_gray;
    // params for histgram
    int hbins = 180;
    int histSize[] = {hbins};
    MatND h_hist;
    float hranges[] = { 0, 180 };
    int channels[] = {0};
    int thresh=160;
    const float* ranges[] = { hranges };

    // params for back projection via Hue channel
    int hchannel[]={0};
    MatND backproj;

    // params to draw the adaptive window
    Point2f points[4];

    // params for corner detection
    Mat image_gray;
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point2f> corners_after;
    int max_corners = 15;
    double quality_level = 0.01;
    double min_distance = 5.0;
    int block_size = 3;
    bool use_harris = false;
    double k = 0.04;
    vector<uchar> status;
    vector<float> err;

    // params for optical flow
    IplImage *frame;

    // create a variable in the class
    CameraParameters TheCameraParameters;

   // load the parameter matrix in the constructor
  Mat dist(1,5,CV_32FC1);
  dist.at<float>(0,0)=-0.066494;
  dist.at<float>(0,1)=0.095481;
  dist.at<float>(0,2)=-0.000279;
  dist.at<float>(0,3)=0.002292;
  dist.at<float>(0,4)=0.000000;
  Mat cameraP(3,3,CV_32FC1);

  cameraP.at<float>(0,0)=551.543059;
  cameraP.at<float>(0,1)=0.000000;
  cameraP.at<float>(0,2)=327.382898;
  cameraP.at<float>(1,0)=0.000000;
  cameraP.at<float>(1,1)=553.736023;
  cameraP.at<float>(1,2)=225.026380;
  cameraP.at<float>(2,0)=0.000000;
  cameraP.at<float>(2,1)=0.000000;
  cameraP.at<float>(2,2)=1.000000;

  TheCameraParameters.setParams(cameraP,dist,Size(640,480));
  TheCameraParameters.resize( Size(640,480));



    // init
    orginImage=cv_ptr->image;
    loadedImage = imread("/home/hrs/Desktop/templateImg.jpg");

    cvtColor(loadedImage(myROI), LoadHSVImage ,CV_BGR2HSV);
    cvtColor(cv_ptr->image, HSVImage ,CV_BGR2HSV);

    // load the image and split the image

    split(LoadHSVImage,splitedImage);

    //generate the mask via S channel
    cv::threshold(splitedImage[1],maskImage,170,255.0,CV_THRESH_BINARY); // threshold=160

    //generate Histgram and normalize it to the range of [0,255]
    calcHist(&splitedImage[0], 1, channels, maskImage,
                h_hist, 1, histSize, ranges,
                true, // the histogram is uniform
                false);
    normalize(h_hist, h_hist, 0, 255, NORM_MINMAX, -1, Mat() );

    // back projection
    calcBackProject( &HSVImage, 1,hchannel,h_hist, backproj, ranges);

    // mean-shift and draw the rectangle
    // Setup the termination criteria, either 10 iteration or move by atleast 1 pt
    TermCriteria term_crit(TermCriteria::EPS | TermCriteria::COUNT, 10, 1);
    /*meanShift(backproj,myROI,term_crit);
    rectangle(orginImage, myROI, 255, 2);*/

    // camshift
    RotatedRect rot_rect = CamShift(backproj, myROI, term_crit);
    rot_rect.points(points);
      for (int i = 0; i < 4; i++)
        line(orginImage, points[i], points[(i+1)%4], 255, 4);
  
    

    //features to track
    cvtColor(cv_ptr->image,image_gray, CV_RGB2GRAY);
    // Optical flow
    if(flag==false)
      {

          ImageOld= image_gray;
          cv::goodFeaturesToTrack(ImageOld,
                       p0,
                       max_corners,
                       quality_level,
                       min_distance,
                       cv::Mat(),
                       block_size,
                       use_harris,
                       k);
          mask = Mat::zeros(cv_ptr->image.size(), cv_ptr->image.type());

      }

     //conner detection

    vector<Point2f> good_new;
    if(flag)
    {
      calcOpticalFlowPyrLK(ImageOld,image_gray,p0,corners_after,status,err);
      for(uint i = 0; i < p0.size(); i++)
      {
          // Select good points
          if(status[i] == 1) {
              good_new.push_back(corners_after[i]);
              // draw the tracks
              line(mask,corners_after[i], p0[i],Scalar(0,255,0),4);
              circle(cv_ptr->image, corners_after[i], 5,Scalar(0,0,255) ,-1);
          }
      }

      ImageOld = image_gray;
      p0 = good_new;

    }

    Mat img;
    add(cv_ptr->image, mask, img);

    /*3D MARKER POSITION*/
    aruco::MarkerDetector Detector;
    //Detector.setDictionary("ARUCO_MIP_36h12");
    std::vector< Marker >  detectedMarkers;
    Detector.detect(cv_ptr->image,detectedMarkers,TheCameraParameters,5,true);
    for (unsigned i = 0; i< detectedMarkers.size(); i++ )
    {
      ROS_INFO_STREAM("3D pose"<<detectedMarkers[i].Tvec);
      

      detectedMarkers[i].draw(cv_ptr->image,Scalar(0,0,255),2);
    }
    

    //Rodrigues(rotationVector, rotationMatrix);


    // Update GUI Window
    imshow(OPENCV_WINDOW_0, orginImage);
    imshow(OPENCV_WINDOW_1, loadedImage); //loadedImage(myROI)
    imshow(OPENCV_WINDOW_2, backproj);
    imshow(OPENCV_WINDOW_3, img);
    imshow(OPENCV_WINDOW_4, cv_ptr->image);


    flag = true;


    waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }




};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tutorial_4");
  ImageConverter ic;
  ros::spin();
  return 0;
}
