/********************************************************Cong Wang*******************************************************************
 * This package fulfills the following functions of our project:
 * 1. Getting the position of the object (cup) in the world-frame according to the base-footprint of NAO.
 * 2. Approaching the object until NAO reaches the pre-grasp position
 * 3. Grasping the object, after that, standing up
 * 
 ************************************************************************************************************************************
 * The constuction of my code: 
 * 1) Firstly, after the speech recognition part, the service Mode_change is called and NAO starts searching. I use Yolo to detect the object.
 * 
 * 2) Secondly, if NAO finds the object, the moving & approaching part starts. The approaching function is seperated into 2 parts: far-approach and near-approach.
 * The boundary of near and far is about 50cm. The detection and distance calculating algorithm is different in the 2 modes. 
 * When in far-approach mode, I get the ROI from yolo. After that, I use a filter to get the red pixels of the cup. The distance of the cup is calculated based on
 * the pixel height of the filtered image.
 * When in near-approach mode, the distance is calculated based only on the pixel coordinate of the cup. I have made a fitting curve of the measured data of the 
 * cup position and pixel coordinate from 3cm to 50cm. 
 * 
 * 3) Thirdly, when NAO reaches the pre-grasp position, the service grasping will be called to fulfill grasping motion.
 *
 ************************************************************************************************************************************
 * The IO port of this code:
 * Publisher: pub_pose; to publish the object's coordinate
 * Service client:  shezhang_cli; to call the localization server to start navigation-localization part
                    navi_cli; to call and start the navigation part 
                    yuan_cli; to call the face recognition server after grasping the object

 * Service server: mode1_srv; call this service to control the whole contol mode of my programm
                              set req to 0 is waiting for the programm to start
                              set req to 1 is starting searching and far approaching
                              set req to 2 is starting near approaching
                              set req to 3 is let NAO autonomously approach the object and grasp, then stand up
************************************************************************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_srvs/Empty.h>

//opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video.hpp>

//datknet
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>


//srv
#include <detection/mode1.h>
#include <speech/walk_polar.h>
#include <using_markers/destination.h>

//msg
#include <detection/detectPose.h>

using namespace std;
using namespace cv;


namespace detectionSpace
{
    class Detection
    {
        private:
            // The node handle
            ros::NodeHandle nh_;
            // Node handle in the private namespace
            ros::NodeHandle priv_nh_;


            //Publisher
            ros::Publisher pub_walk; // walking publisher
            ros::Publisher pub_pose; // publishing detected pose

            //Subscriber
            ros::Subscriber sub_detected_image; // subscriber for yolo
            ros::Subscriber sub_box;      // step 1, far approach
            ros::Subscriber sub_cup_near; // step 2, near approach

            //Service client
            ros::ServiceClient stop_walk_cli; // client to stop walking, not used in the code
            ros::ServiceClient head_down_cli; // down head
            ros::ServiceClient head_up_cli;   // rise head
            ros::ServiceClient pick_up_cli;   // pick up 
            ros::ServiceClient standup_cli;   // client to call the stand service
            ros::ServiceClient shezhang_cli;  // client to call the navigation-localization service
            ros::ServiceClient navi_cli; //client to call the navigation programm in using_markers package

            ros::ServiceClient walk_general_cli; // client to call the walk-polar service
            ros::ServiceClient yuan_cli;      // client to call the face-recognition service

            //Service server
            ros::ServiceServer mode1_srv; // using to change mode



            // Window
            std::string OPENCV_WINDOW_0 = "Detected Image";
            std::string OPENCV_WINDOW_1 = "Cup";
            //std::string OPENCV_WINDOW_2 = "Colour extraction";  // track bar
            std::string OPENCV_WINDOW_3 = "Threshold Cup Box";

            // Callback functions
            void getDetectedImage (const sensor_msgs::Image::ConstPtr &msg); // get yolo msg
            void cupCallback (const darknet_ros_msgs::BoundingBoxes::ConstPtr &box); // far approach callback
            void nearcupCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr &cup); // near approach callback 
            bool mode1Callback (detection::mode1::Request &req, detection::mode1::Response &res); // changing mode callback, important!!!

        public:

            pthread_mutex_t count_mutex; 

            // Important flag !!!
            bool flag_walk = 0; //!!! When 1, the NAO will not walk, when 0 the NAO can walk
                                // This flag is changed when service server 'mode1_srv' is called and req is set to 3
            bool find = 0; // !!! 1 is found the object, 0 is to start searching
            // mode flag
            int mode_approach = 0; //!!! mode falg 1 is for far-approcah, 2 is near-approach, 3 is picking up and ending
                                   // This flag is changed when service server 'mode1_srv' is called
                                   // Also autonomously changes in code when NAO is approaching the object


            //box from yolo
            int64 cup_xmin;
            int64 cup_xmax;
            int64 cup_ymin;
            int64 cup_ymax;

            //image
            cv_bridge::CvImagePtr detected_image_cv_ptr;
            sensor_msgs::Image detected_image_ros;
            Mat cup_box;
            Mat hsv_box;
            Mat thresh_box;


            //track bar
            int iLowH=0;
            int iHighH=179;
            int iLowS=0;
            int iHighS=255;
            int iLowV=0;
            int iHighV=255;

            //erode and delate
            Mat kernel;
            Mat Image_blob;
            Mat Image_blob_0;

            // pixel height
            int pixel_height_old = 0;
            int pixel_height_new = 0;
            int pixel_height_cal;
            int y_top, y_bottom;
            int row_box;
            int col_box;
            int pixel_norm = 40;
            int pixel_abs;

            //centre
            double cup_x_centre_old = 0;
            double cup_x_centre_new;

            double near_x_centre_cup;
            double near_y_top_cup_old=0;
            double near_y_top_cup_new;
            double near_y_diff;
            double near_x_mid_pub;

            // grasp
            double goal_x_grasp = 160;
            double goal_x_move_grasp = 178;
            double near_x_move;
            double goal_y_grasp = 184;
          
            // Distance
            double distance_cal;
            double distance_move;

            double distance_near_cal;
            double distance_near_move;

            //walk flag         
            bool flag_rise = false;
            bool flag_down = false;

            // turn abs
            double turn_abs_1 = 30;  // far
            double turn_abs_2 = 10;  // near
            double turn_abs_3 = 20; // near-far
            double turn_abs_near;
                 
            // move flag
            int count=0;  // to remove error of the pixel
            int count_cal=0; // walking flag far
            int near_count = 0; // to remove error and walking flag near
            
            // distance flag
            int flag_dist_cal = 6; // flag to deside the distance
            int flag_dist_move = 6; // flag of moving distance
            int flag_near_dist = 6; // flag of near approaching distance
            
            //find flag          
            double time_find = 15;
            double turn_find = 0.2; // turning angle
            bool find_walk = false; // true is not to walk

            int first_check = 0; // searching flag, avoid calculating when searching
            int find_count_flag = 0;
            bool find_1_start = 1; // first searching method, turning left step-by-step until 90 degree
            bool find_2_start = 0; // second searching method, turning right step-by-step until 90 degree
            bool find_3_start = 0; // third searching method, going to another position
            bool find_4_start = 0; // forth searching method, turning right continouslly. 

            int find_1_count = 0;
            int find_2_count = 0;

            // picking flag
            bool picking_1 = false;
            bool picking_2 = false;

            // stand flag
            bool stand_up = false;

            // ending flag
            bool end_flag = false;

            // pub position
            detection::detectPose pose_far;  // theta + is right, - is left
            detection::detectPose pose_near; // theta + is right, - is left
            double dx_pose;

            // distance curve fitting
            double a1 = 515.2;
            double b1 = 0.00235;
            double c1 = 2.877;
            double a2 = 126.3;
            double b2 = 0.008414;
            double c2 = 5.471;

            //distance pub near
            double dist_near_y_pub = -1;
            double dist_near_x_pub = -1;
            double near_theta_pub = 0;
            double near_dist_pub = 0;

            // call navigation flag
            bool shezhang_flag = 0;
            bool yansong_flag = 0;
            
            // functions
            void walk(double x, double y, double theta); // walking function
            void stop_walk();                            // stop walk
            void down_head();                            // down head
            void rise_head();                            // rise head
            void approach(int label, double distance);   // approaching function, label 1 for far approaching, 2 for near approaching
            void find_turn();                            // searching function
            void pickup();                               // picking up
            void get_dist_near(double x, double y);      // calculating distance near, for publishing
            void walk_polar(double dist, double theta);  // walking polar
            void stand();                                // stand up
            void yuan();                                 // call face-recognition
            void shezhang();                             // call navigation-localization
            void navi(double dist, double theta);        // start navigation

            Detection(ros::NodeHandle nh, ros::NodeHandle nh_priv);
            ~Detection();

    };
}
