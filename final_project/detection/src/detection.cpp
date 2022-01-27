// Please check the headfile detection.h
#include<detection/detection.h>

using namespace std;
using namespace cv;


namespace detectionSpace
{
    Detection::Detection(ros::NodeHandle nh, ros::NodeHandle nh_2)
    {

        count_mutex = PTHREAD_MUTEX_INITIALIZER;
        
        namedWindow(OPENCV_WINDOW_0);
        namedWindow(OPENCV_WINDOW_1);
        //namedWindow(OPENCV_WINDOW_2); // trackbar
        namedWindow(OPENCV_WINDOW_3);

        sub_detected_image = nh_2.subscribe("/darknet_ros/detection_image", 2, &detectionSpace::Detection::getDetectedImage, this);
        sub_box = nh.subscribe("/darknet_ros/bounding_boxes", 2, &detectionSpace::Detection::cupCallback, this);
        sub_cup_near = nh.subscribe("/darknet_ros/bounding_boxes", 2, &detectionSpace::Detection::nearcupCallback, this);

        // publisher position
        pub_pose = nh.advertise <detection::detectPose>("DetectPose", 2);

        //mode server
        mode1_srv = nh_2.advertiseService("Mode_change", &detectionSpace::Detection::mode1Callback, this);

        //walk
        pub_walk = nh.advertise<geometry_msgs::Pose2D>("/cmd_pose", 3);
        stop_walk_cli=nh_2.serviceClient<std_srvs::Empty>("/stop_walk_srv");

        //walk cli
        walk_general_cli = nh.serviceClient<speech::walk_polar>("/WalkPolar");

        //head down
        head_down_cli=nh_2.serviceClient<std_srvs::Empty>("/Downhead");

        //head rise
        head_up_cli = nh_2.serviceClient<std_srvs::Empty>("/Risehead");

        // pick up
        pick_up_cli = nh_2.serviceClient<std_srvs::Empty>("/Movejoints");

        // stand up
        standup_cli = nh_2.serviceClient<std_srvs::Empty>("/WalkBack");

        // end, call face-recognition
        yuan_cli = nh_2.serviceClient<std_srvs::Empty>("/facerecognition");

        // call navigation-localization
        shezhang_cli = nh_2.serviceClient<std_srvs::Empty>("/mapping");

        // call navigation and start navigation
        navi_cli = nh_2.serviceClient<using_markers::destination>("/naviDestination");


        // msg to pub
        pose_far.dist = 0;
        pose_far.theta = 0;

        pose_near.dist = 0;
        pose_near.theta = 0; 
    }

    Detection::~Detection()
    {
       destroyWindow(OPENCV_WINDOW_0);
    }


    // Function to get image from Yolo
    void Detection::getDetectedImage (const sensor_msgs::Image::ConstPtr &msg)
    {
        pthread_mutex_lock( &this->count_mutex );
        ROS_INFO("Getting detected image from Yolo!");
        detected_image_cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        imshow(OPENCV_WINDOW_0, detected_image_cv_ptr->image);
        waitKey(3);// important
        pthread_mutex_unlock( &this->count_mutex );
    }
    
    // Main function for far approach
    void Detection::cupCallback (const darknet_ros_msgs::BoundingBoxes::ConstPtr &box)
    {
        pthread_mutex_lock( &this->count_mutex );

        if(mode_approach == 0)
        {
            ROS_INFO("Waiting for changing mode...");
            ROS_INFO("Waiting for changing mode...");
            ROS_INFO("Waiting for changing mode...");
        }

        //*****************************************Find object*********************************************************
        if(!find && mode_approach == 1)
        {
            ROS_INFO("NAO can not see the object, now start searching...");
            ROS_INFO("NAO can not see the object, now start searching...");
            rise_head(); 
            if(find_count_flag > 4)
            {
                ROS_INFO("NAO is searching the object!!!");
                ROS_INFO("NAO is searching the object!!!");             
                for(int i=0;i < box->bounding_boxes.size();i++)
                {
                    if(box->bounding_boxes[i].Class == "cup")
                    {
                        ROS_INFO("I have found the object!!!!");
                        ROS_INFO("I have found the object!!!!");
                        ROS_INFO("I have found the object!!!!");
                        find = true;

                        cup_x_centre_new = box->bounding_boxes[i].xmin - 160;
                        cup_x_centre_old = cup_x_centre_new;
                        turn_find = 0;
                        break;
                    } 
                }
                if(!find_walk && find_1_start)
                {
                    ROS_INFO("Turning left to find the object");
                    ROS_INFO("Turning left to find the object");
                    walk(0,0,turn_find);
                    sleep(2);
                    find_2_count++;
                    if(find_2_count > 4)
                    {
                        find_1_start = 0;
                        //find_2_start = 1;
                        find_4_start = 1;
                    }
                }
                if(!find_walk && find_2_start)
                {
                    ROS_INFO("Going to another place to find");
                    ROS_INFO("Going to another place to find");
                    ROS_INFO("Going to another place to find");
                    walk(0, 0, -6*turn_find);
                    sleep(6);
                    find_3_start = 1;
                    find_2_start = 0;
                }

                if(!find_walk && find_3_start)
                {
                    ROS_INFO("Going to another place to find");
                    ROS_INFO("Going to another place to find");
                    ROS_INFO("Going to another place to find");

                    walk(0.3, 0, 0);
                    sleep(5);
                    find_3_start = 0;
                    find_4_start = 1;
                }

                if(!find_walk && find_4_start)
                {
                    ROS_INFO("Turning right to find the object");
                    ROS_INFO("Turning right to find the object");
                    walk(0,0,-turn_find);
                    sleep(2);
                }
            }
            find_count_flag++;
        }
        //***************************************************************************************************************

        if(mode_approach == 1 && flag_dist_move > 2 && find)
        {
            
            for(int i=0;i < box->bounding_boxes.size();i++)
            {
                if(box->bounding_boxes[i].Class == "cup")
                {
                    cup_xmin = box->bounding_boxes[i].xmin;
                    cup_xmax = box->bounding_boxes[i].xmax;
                    cup_ymin = box->bounding_boxes[i].ymin;
                    cup_ymax = box->bounding_boxes[i].ymax;


                    cup_x_centre_new = 0.5*(cup_xmin + cup_xmax) - 160;

                    // ROS_INFO("The cup is in: ");
                    // cout << "x" << ":" << cup_xmin << " " << cup_xmax << endl;
                    // cout << "y" << ":" << cup_ymin << " " << cup_ymax << endl;

                } 
                else
                {
                    //ROS_INFO("Can not detect the cup!");
                }    
            }

            
            ROS_INFO("Now is in approach mode 1!!!");
            ROS_INFO("----------------------------------");

            // Call navigation 
            if(!shezhang_flag)
            {
                shezhang();
                shezhang_flag = 1;
            }
            
            // flag to find pixel
            bool find_top = 0;
            bool find_bottom = 0;

            if(!flag_rise)
            {
                rise_head(); 
                flag_rise = true;
            }


            if(first_check == 0)
            {
                //first_check = true;
                sleep(5);
                ROS_INFO("Waiting...");
            }


            if(first_check > 3)
            {
                //***************************************dealing with image************************************

                // Set the detected cup box to ROI
                Rect cup_ROI (cup_xmin, cup_ymin, cup_xmax-cup_xmin, cup_ymax-cup_ymin);
                cup_box = detected_image_cv_ptr->image(cup_ROI);
                imshow(OPENCV_WINDOW_1, cup_box);
                waitKey(3);// important!!
                
                //Filter the red pixels in the ROI

                // change to HSV
                cvtColor(cup_box,hsv_box,CV_BGR2HSV);

                //Track bar test
                /*
                cv::createTrackbar("lowH",OPENCV_WINDOW_2,&iLowH,179);
                cv::createTrackbar("HighH",OPENCV_WINDOW_2,&iHighH,179);
                cv::createTrackbar("LowS",OPENCV_WINDOW_2,&iLowS,255);
                cv::createTrackbar("HighS",OPENCV_WINDOW_2,&iHighS,255);
                cv::createTrackbar("LowV",OPENCV_WINDOW_2,&iLowV,255);
                cv::createTrackbar("HighV",OPENCV_WINDOW_2,&iHighV,255);
                inRange(hsv_box,cv::Scalar(iLowH,iLowS,iLowV),cv::Scalar(iHighH,iHighS,iHighV),thresh_box);
                */
                
                // filter
                inRange(hsv_box,cv::Scalar(0,150,120),cv::Scalar(179,236,255),thresh_box);
                imshow(OPENCV_WINDOW_3, thresh_box);

                //ROS_INFO_STREAM(thresh_box.size);
                // cout << thresh_box.rows << endl;
                // cout << thresh_box.cols << endl;

                row_box = thresh_box.rows;
                col_box = thresh_box.cols; 

                //********************************************************************************************************************      

                //************************************************Find the top and bottom of the cup pixels***************************
                int find_y_top = 0;
                int find_y_bottom = row_box;         

                while(!find_top)
                {
                    for(int x = 0;x<col_box;x++)
                    {
                        if(int(thresh_box.at<uchar>(find_y_top,x)) == 255) //important, the type is uchar here, stands for 0-255
                        {
                            find_top = 1;
                            y_top = find_y_top;
                            break;
                        }                      
                    }

                    find_y_top++;
                }

                while(!find_bottom)
                {
                    for(int x = 0;x<col_box;x++)
                    {
                        if(int(thresh_box.at<uchar>(find_y_bottom,x)) == 255)
                        {
                            find_bottom = 1;
                            y_bottom = find_y_bottom;
                            break;
                        }                      
                    }

                    find_y_bottom--;
                }
                //***********************************************************************************************************************

                pixel_height_new = y_bottom - y_top;

                //************************************Get pixel height*************************************************************

                if(abs(pixel_height_new - pixel_height_old) < 4 && abs(cup_x_centre_new - cup_x_centre_old) < 40) // to eliminate noise
                {
                    ROS_INFO("I'm locating the object! ");
                    ROS_INFO("Please wait!");

                    count++;             
                }
                
                else
                {
                    ROS_INFO("Too much noise....................");
                    count = 0;
                }

                pixel_height_old = pixel_height_new;

                if(count > 4) // 3 times without noise
                {
                    ROS_INFO("--------------------------------------------"); 
                    ROS_INFO("Fineshed locating!");
                    ROS_INFO("The height of the object is:");
                    pixel_height_cal = pixel_height_new;
                    ROS_INFO_STREAM(pixel_height_cal); 
                    count_cal++; //1 time calculate, when count_cal>2, walk
                
                    pixel_abs = pixel_norm - pixel_height_cal;
                }
                //*******************************************************************************************************************
                
                // ********************************decide the distance to walk*******************************************************
                if(count_cal > 4)
                {

                    switch (pixel_abs)
                    {
                        case 1:
                            distance_cal = 0.1;
                            flag_dist_cal = 1;
                            pose_far.dist = 50;
                            break;

                        case 2:
                            distance_cal = 0.1;
                            flag_dist_cal = 1;
                            pose_far.dist = 51;
                            break;

                        case 3:
                            distance_cal = 0.1;
                            flag_dist_cal = 1;
                            pose_far.dist = 52;
                            break;

                        case 4:
                            distance_cal = 0.1;
                            flag_dist_cal = 1;
                            pose_far.dist = 54;
                            break;

                        case 5:
                            distance_cal = 0.1;
                            flag_dist_cal = 1;
                            pose_far.dist = 56;
                            break;

                        case 6:
                            distance_cal = 0.15;
                            flag_dist_cal = 2;
                            pose_far.dist = 58;
                            break;

                        case 7:
                            distance_cal = 0.15;
                            flag_dist_cal = 2;
                            pose_far.dist = 60;
                            break;
                            
                        case 8:
                            distance_cal = 0.25;
                            flag_dist_cal = 2;
                            pose_far.dist = 65;
                            break;

                        case 9:
                            distance_cal = 0.25;
                            flag_dist_cal = 2;
                            pose_far.dist = 70;
                            break;

                        case 10:
                            distance_cal = 0.3;
                            flag_dist_cal = 2;
                            pose_far.dist = 75;
                            break;

                        case 11:
                            distance_cal = 0.3;
                            flag_dist_cal = 2;
                            pose_far.dist = 80;
                            break;

                        case 12:
                            distance_cal = 0.3;
                            flag_dist_cal = 2;
                            pose_far.dist = 90;
                            break;

                        case 13:
                            distance_cal = 0.3;
                            flag_dist_cal = 3;
                            pose_far.dist = 93;
                            break;

                        case 14:
                            distance_cal = 0.3;
                            flag_dist_cal = 3;
                            pose_far.dist = 97;
                            break;

                        case 15:
                            distance_cal = 0.30;
                            flag_dist_cal = 3;
                            pose_far.dist = 100;
                            break;

                        case 16:
                            distance_cal = 0.30;
                            flag_dist_cal = 4;
                            pose_far.dist = 105;
                            break;

                        case 17:
                            distance_cal = 0.30;
                            flag_dist_cal = 4;
                            pose_far.dist = 110;
                            break;

                        case 18:
                            distance_cal = 0.35;
                            flag_dist_cal = 4;
                            pose_far.dist = 113;
                            break;

                        case 19:
                            distance_cal = 0.35;
                            flag_dist_cal = 4;
                            pose_far.dist = 117;
                            break;

                        case 20:
                            distance_cal = 0.40;
                            flag_dist_cal = 5;
                            pose_far.dist = 120;
                            break;
                    
                        default:
                            ROS_INFO("The pixel height is somehow very strange!!! ");
                            ROS_INFO("The pixel height is somehow very strange!!! ");
                            ROS_INFO("The pixel height is somehow very strange!!! ");
                            distance_cal = 0.1;
                            pose_far.dist = -1;
                            break;
                    }
                }

                ROS_INFO_STREAM("The distance flag is: " << flag_dist_cal << "!!!!");

                //******************************************************************************************************************


                ROS_INFO("--------------------------------------------");
                ROS_INFO("The centre is in: ");
                ROS_INFO_STREAM(cup_x_centre_new);
                cup_x_centre_old = cup_x_centre_new;



               //**************************************publish pose*************************************************************
               if( count_cal > 4)
               {
                   dx_pose = (11.5 * cup_x_centre_new) / pixel_height_cal;
                   pose_far.theta = -atan(dx_pose/pose_far.dist);
                   pub_pose.publish(pose_far); // publishing object position
                   if (!yansong_flag) // call service in package using_markers and send object position
                   {
                        navi(pose_far.dist, pose_far.theta); 
                        yansong_flag = 1;
                   }
               }
               //****************************************************************************************************************




                //*****************************************Walk******************************************************************
                if(!flag_walk && count_cal > 4)
                {
                    approach(1, distance_cal);
                }
                //***************************************************************************************************************

            }
            first_check++;
        }

        // *****************************************Change to near approach***************************************************************
        else if(mode_approach == 1 && flag_dist_move <= 2)
        {
            ROS_INFO("Near the object!!!");
            ROS_INFO("Near the object!!!");
            ROS_INFO("Now changing to control mode 2");

            mode_approach = 2;
        }
        //**********************************************************************************************************************************

        else if(mode_approach == 2)
        {
            ROS_INFO("Mode 1 closed...");
        }


        else if(mode_approach == 3)
        {
            ROS_INFO("Picked up object!");
        }

        pthread_mutex_unlock( &this->count_mutex );
    
    }


    // Main function for near approach
    void Detection::nearcupCallback (const darknet_ros_msgs::BoundingBoxes::ConstPtr &cup)
    {
        pthread_mutex_lock( &this->count_mutex );
        if(mode_approach == 2)
        {
            ROS_INFO("Now in approach mode 2!!!");
            ROS_INFO("----------------------------------");
            flag_dist_move = 0;

            if(!flag_down)
            {
                down_head(); // bowing the head inorder to see the cup
                flag_down = true;
            }

            int x_min;
            int x_max;
            int y_min;
            int y_max;

            for(int i=0;i < cup->bounding_boxes.size();i++)
            {
                if(cup->bounding_boxes[i].Class == "cup" || cup->bounding_boxes[i].Class == "bowl" || cup->bounding_boxes[i].Class == "frisbee")
                {
                    x_min = cup->bounding_boxes[i].xmin;
                    x_max = cup->bounding_boxes[i].xmax;
                    y_min = cup->bounding_boxes[i].ymin;
                    y_max = cup->bounding_boxes[i].ymax;
                }
            }
            
            //***********************************get the pixel coordinate of the cup ROI****************************************************
            near_x_centre_cup = 0.5*(x_min + x_max) - goal_x_grasp;
            near_y_top_cup_new = y_min;

            near_x_mid_pub = 0.5*(x_min + x_max);// pub

            ROS_INFO_STREAM ("x " << near_x_centre_cup);
            //ROS_INFO_STREAM ("x " << near_x_mid_pub);
            ROS_INFO_STREAM ( "y " << near_y_top_cup_new);
            //******************************************************************************************************************************

            if(abs(near_y_top_cup_new - near_y_top_cup_old) <= 1)
            {
                ROS_INFO("I am locating the cup, Please wait!");
                near_count++;                                       //count to walk
                get_dist_near(near_x_mid_pub, near_y_top_cup_new);
                pose_near.dist = near_dist_pub;
                pose_near.theta = near_theta_pub;
            }
            
            else if (abs(near_y_top_cup_new - near_y_top_cup_old) > 1)
            {
                ROS_INFO(" Too much noise!!!");
            }

            near_y_top_cup_old = near_y_top_cup_new;

            near_y_diff = goal_y_grasp - near_y_top_cup_new;


            if(near_count > 5) // eliminate the noise
            {
                //******************************************************calculate the moving distance***************************************************
                if(near_y_diff < 0)
                {
                    ROS_INFO("The distance is less than 0");
                    //ROS_INFO("Error occurs");
                    near_count = 0;
                }

                else if(near_y_diff > 156 && near_y_diff <= goal_y_grasp)
                {
                    distance_near_cal = 0.2;
                    flag_near_dist = 5;
                }

                else if(near_y_diff > 130 && near_y_diff <= 156)
                {
                    distance_near_cal = 0.1;
                    flag_near_dist = 4;
                }

                else if(near_y_diff > 74 && near_y_diff <= 130)
                {
                    distance_near_cal = 0.05;
                    flag_near_dist = 3;
                }

                else if(near_y_diff > 10 && near_y_diff <= 74)
                {
                    flag_near_dist = 2;
                    distance_near_cal = 0.0015*near_y_diff;
                    near_x_move = ((goal_x_move_grasp - near_x_mid_pub)*distance_near_cal) / near_y_diff;
                    ROS_INFO("I think NAO is reaching the pre-grasp position!!!");
                    picking_1 = true;
                }

                else if(near_y_diff <= 10)
                {
                    flag_near_dist = 1;
                    ROS_INFO("Yeah!!!!!!!!!!!!!!!!!!!!!!!!");
                    ROS_INFO("Hahahahahahahahahahahahahaha");
                    ROS_INFO("NAO has reached the pre-grasp position!");
                    ROS_INFO("NAO has reached the pre-grasp position!");
                    ROS_INFO("NAO has reached the pre-grasp position!");
                    near_count = 0;
                }

                ROS_INFO_STREAM("The flag of distance near is: " << flag_near_dist);

                if(flag_near_dist > 3)
                {
                    turn_abs_near  = turn_abs_3;
                }

                else if(flag_near_dist <= 3)
                {
                    turn_abs_near = turn_abs_2;
                }
                //********************************************************************************************************

            }
            ROS_INFO_STREAM("The flag of distance near is: " << flag_near_dist);

            //*********************************************publishing object position*********************************************
            if(near_count > 5)
            {
                // publish the pose near
                pub_pose.publish(pose_near);

                if (!yansong_flag) // call service in package using_markers and send object position
                {
                    navi(pose_near.dist, pose_near.theta); 
                    yansong_flag = 1;
                }               
            }

            //***********************************************approaching*******************************************************
            if(!flag_walk && near_count > 5)
            {
                approach(2, distance_near_cal);
                if(picking_2)
                {
                    flag_walk = true;
                }     
            }
            //*************************************************picking up********************************************************
            if(picking_2)
            {
                sleep(3);
                ROS_INFO("Begin to pick up!!!");
                mode_approach = 3;
                pickup(); // Begin picking up
            }
            //*******************************************************************************************************************
        }

        else if(mode_approach == 1) // when in mode 1, far approach
        {
            ROS_INFO("Mode 2 is closed");
        }

        // **************************************************stand up**************************************************************
        else if(mode_approach == 3 && !stand_up)
        {
            ROS_INFO("Picked up object!");
            if(!stand_up)
            {
                stand();
                stand_up = true;
            }
        }
        //***********************************Programm finished and start face recognition***************************************
        else if(mode_approach == 3 && stand_up && !end_flag)
        {
            ROS_INFO("Begin to call the recognition programm ");
            if(!end_flag)
            {
                yuan();
                end_flag = true;
            }
        }

        else if(mode_approach == 3 && stand_up && end_flag)
        {
            ROS_INFO("Programm finished!!!");
            ROS_INFO("Nao is seaching people!!!");
        }

        pthread_mutex_unlock( &this->count_mutex );
        //************************************************************************************************************************
    }


    // Main moving function
    void Detection::approach(int label, double dist) 
    {
        //*************************** moving in far approaching mode************************************
        if(label == 1) 
        {
            while(!flag_walk && count_cal > 4)
            {
                if(cup_x_centre_new>turn_abs_1)
                {
                    //***************turn right********************
                    if(flag_dist_cal >=5)
                    {
                        ROS_INFO("NAO turning SMALL right!!!");
                        walk(0,0,-0.1);
                        sleep(1);
                        count_cal = 0;
                        //wait(2);
                        break;
                    }

                    else if(flag_dist_cal == 4)
                    {
                        ROS_INFO("NAO turning MIDDLE right!!!");
                        walk(0,0,-0.0025*cup_x_centre_new);
                        sleep(1);
                        count_cal = 0;
                        //wait(2);
                        break;
                    }

                    else if(flag_dist_cal <= 3)
                    {
                        ROS_INFO("NAO turning Large right!!!");
                        walk(0,0,-0.003*cup_x_centre_new);
                        sleep(1);
                        count_cal = 0;
                        //wait(2);
                        break; 
                    }

                }
                else if (cup_x_centre_new<-turn_abs_1)
                {
                    //*****************turn left********************
                    if(flag_dist_cal >=5)
                    {
                        ROS_INFO("NAO turning SMALL left!!!");
                        walk(0,0,0.1);
                        sleep(1);
                        count_cal = 0;
                        //wait(2);
                        break;
                    }

                    else if(flag_dist_cal == 4)
                    {
                        ROS_INFO("NAO turning MIDDLE left!!!");
                        walk(0,0,-0.0025*cup_x_centre_new);
                        sleep(1);
                        count_cal = 0;
                        //wait(2);
                        break;
                    }

                    else if(flag_dist_cal <= 3)
                    {
                        ROS_INFO("NAO turning Large left!!!");
                        walk(0,0,-0.003*cup_x_centre_new);
                        sleep(1);
                        count_cal = 0;
                        //wait(2);
                        break; 
                    }
                }

                //*****************Walk forward************************
                else if (abs(cup_x_centre_new) <= turn_abs_1)    
                {
                    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    ROS_INFO("NAO begin walking!!!!");
                    ROS_INFO("NAO begin walking!!!!");
                    ROS_INFO("NAO begin walking!!!!");
                    distance_move = dist;
                    //flag_walk = 1;
                
                    walk(distance_move, 0, 0);
                    flag_dist_move = flag_dist_cal;
                    ROS_INFO("Nao walking: ");
                    ROS_INFO_STREAM("Nao is walking" << distance_move << "meter");
                    ROS_INFO_STREAM("Nao is walking" << distance_move << "meter");
                    ROS_INFO_STREAM("Nao is walking" << distance_move << "meter");
                    count_cal = 0;
                    distance_cal = 0;
                    sleep(1);
                    ROS_INFO_STREAM("The moving distance flag is: " << flag_dist_move << "!!!");
                    //wait(10);
                    break;
                }
                else
                {
                    ROS_INFO("approach ERROR!!!!!!!!!!!!!!!!!!!");
                    ROS_INFO("approach ERROR!!!!!!!!!!!!!!!!!!!");
                    break;
                }
                ROS_INFO("Walking...");
            }
        }

        //********************************moving in near approaching mode********************************************
        else if (label == 2)
        {
            ROS_INFO("Close approaching!!!!");
            while(!flag_walk && near_count > 5)
            {
                double turn_angle = 0.0055*abs(near_x_centre_cup);

                //*****************turn right**********************
                if(near_x_centre_cup > turn_abs_near)
                {
                    ROS_INFO("NAO turning right!!!");
                    ROS_INFO_STREAM(turn_angle);
                    walk(0,0,-turn_angle);
                    sleep(1);
                    near_count = 0;
                    break;
                }
                //*****************turn left*********************
                else if(near_x_centre_cup < -turn_abs_near)
                {
                    ROS_INFO("NAO turning left!!!");
                    walk(0,0,turn_angle);
                    sleep(1);
                    near_count = 0;
                    break;                   
                }
                //****************walk forward***********************
                else if (abs(near_x_centre_cup) < abs(turn_abs_near))
                {
                    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    ROS_INFO("NAO begin walking!!!!");
                    ROS_INFO("NAO begin walking!!!!");
                    distance_move = dist;
                    
                    if(flag_near_dist == 2) // the last approach before the pre-grasp
                    {
                        ROS_INFO("CLosest approach!!");
                        if(picking_1)
                        {
                            picking_2 = true; // set the picking flag to true
                        }
                        walk(distance_move, near_x_move, 0);
                    }

                    else if(flag_near_dist != 2)
                    {
                        walk(distance_move, 0, 0);
                    } 

                    ROS_INFO("Nao walking: ");
                    ROS_INFO_STREAM("Nao is walking" << distance_move << "meter");
                    ROS_INFO_STREAM("Nao is walking" << distance_move << "meter");
                    ROS_INFO_STREAM("Nao is walking" << distance_move << "meter");
                    //sleep(2);
                    near_count = 0;
                    break;
                }

                else
                {
                    ROS_INFO("approach ERROR!!!!!!!!!!!!!!!!!!!");
                    ROS_INFO("approach ERROR!!!!!!!!!!!!!!!!!!!");
                    near_count = 0;
                    break;
                }
                ROS_INFO("Walking...");
            }
            
        }
    }
    //*********************************************************************************************************

    // Walk function
    void Detection::walk(double x, double y, double theta)
    {
        geometry_msgs::Pose2D pose_walk;
        pose_walk.x = x;
        pose_walk.y = y;
        pose_walk.theta = theta;
        ROS_INFO_STREAM("Walking x = " << x << " y = " << y << " theta = " << theta);
        pub_walk.publish(pose_walk); 
    }

    // Stop walk function, not used in the code
    void Detection::stop_walk()
    {
        std_srvs::Empty emp;
        stop_walk_cli.call(emp);
        flag_walk=0;

    }

    // Bow head function, bow head to -10 degree
    void Detection::down_head()
    {
        std_srvs::Empty emp;
        ROS_INFO("bowing my head............");
        ROS_INFO("bowing my head............");
        ROS_INFO("bowing my head............");
        head_down_cli.call(emp);

    }

    // Rise head function, rise to 20 degree
    void Detection::rise_head()
    {
        std_srvs::Empty emp;
        ROS_INFO("rising my head............");
        ROS_INFO("rising my head............");
        ROS_INFO("rising my head............");
        head_up_cli.call(emp);
    }

    // Pick up object function
    void Detection::pickup()
    {
        std_srvs::Empty emp;
        ROS_INFO("Picking up !!!!");
        ROS_INFO("Picking up !!!!");
        ROS_INFO("Picking up !!!!");
        ROS_INFO("Picking up !!!!");
        pick_up_cli.call(emp);
    }

    // Stand up function, after grasping, NAO stands up with holding the object in hand
    void Detection::stand()
    {
        std_srvs::Empty emp;
        ROS_INFO("Standing up !!!");
        ROS_INFO("Standing up !!!");
        ROS_INFO("Standing up !!!");
        standup_cli.call(emp);
    }

    // Function to call face recognition after standing up
    void Detection::yuan()
    {
        std_srvs::Empty emp;
        ROS_INFO("Calling face recognition!!!");
        ROS_INFO("Calling face recognition!!!");
        ROS_INFO("Calling face recognition!!!");
        yuan_cli.call(emp);
    }

    // Function to call navigation before detection starts
    void Detection::shezhang()
    {
        std_srvs::Empty emp;
        ROS_INFO("Calling localization!!!");
        ROS_INFO("Calling localization!!!");
        ROS_INFO("Calling localization!!!");
        shezhang_cli.call(emp);
    }

    // Function to start navigation until there's no obstacle between the object and NAO
    void Detection::navi(double dist, double theta)
    {
        using_markers::destination obj_pose;
        obj_pose.request.dist = dist;
        obj_pose.request.theta = theta;
        ROS_INFO("Starting navigation!!!");
        ROS_INFO("Starting navigation!!!");
        ROS_INFO("Starting navigation!!!");
        navi_cli.call(obj_pose);
    }

    // Function to calculate the position of the object less than 50cm
    void Detection::get_dist_near(double x, double y)
    {
        ROS_INFO("Calculating the distance near !");
        dist_near_y_pub = a1*sin(b1*y + c1) + a2*sin(b2*y + c2);
        dist_near_x_pub = (dist_near_y_pub * (160 - x)) / (241 - near_y_top_cup_new);
        near_theta_pub = atan(dist_near_x_pub/dist_near_y_pub);
        near_dist_pub = dist_near_y_pub / (cos(near_theta_pub));

        // ROS_INFO_STREAM("The y is: " << dist_near_y_pub);
        // ROS_INFO_STREAM("The x is: " << dist_near_x_pub);
        ROS_INFO_STREAM("The theta is: " << near_theta_pub);
        ROS_INFO_STREAM("The distance is: " << near_dist_pub);        
    }
    
    // !!! Function to change the basic mode of the programm, the most important IO port of my programm
    bool Detection::mode1Callback (detection::mode1::Request &req, detection::mode1::Response &res)
    {
        if(req.req == 1)
        {
            mode_approach = 1; // change to mode 1, begin detecting and far approaching
        }

        else if(req.req == 2)
        {
            mode_approach = 2; // change to mode 2, start near approaching when the object is near 50cm
        }

        else if(req.req == 3)
        {
            flag_walk = 0;  // when no obstacle between object and NAO, NAO begin moving, approaching and grasping
        }
        else
        {
            ROS_ERROR("Service Mode_change not correctly called");
        }
        return true;
    }

    // Another walk function, in polar coordinate of the robot
    void Detection::walk_polar(double dist, double theta) 
    {
        speech::walk_polar walk_srv;
        walk_srv.request.dist = dist;
        walk_srv.request.theta = theta;
        walk_general_cli.call(walk_srv);
    }

}