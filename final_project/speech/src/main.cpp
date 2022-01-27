/************************************Chengjie Yuan************************************************************************
 Using the code template of nao_control_tutorial_3
 *This code contains speech recognition, tactile part along with emergency stop

  At the very first, different subscribers, publishers, serviceclients are declared.
  The whole process of the code:
     Activate the tactile button
     When presssing the front button, the robot starts(speech recognition) and wait for command.
     If it can understand the command it would call the next subtask to detect the goal object
     If not, It would repeat aksing the commander until it can understand the command.
     When the rear button is pressed or it has no contact with the ground, it would stop walking  
***************************************************************************************************************************/

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <boost/algorithm/string.hpp>
#include <boost/date_time.hpp>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <std_msgs/ColorRGBA.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include "actionlib/client/simple_action_client.h"
#include <speech/BlinkAction.h>
#include  <speech/prespeech.h>
#include  <speech/facerecoged.h>
#include  <speech/face_area.h>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <detection/mode1.h> // header for service in detection



using namespace std;

bool stop_thread=false;
void spinThread()
{
  while(!stop_thread)
  {
    ros::spinOnce();
    //ROS_INFO_STREAM("Spinning the thing!!");
  }
}

class Nao_control
{

protected:

  // ROS node handler
  ros::NodeHandle nh_;

  // Subscriber to head tactile states
  ros::Subscriber tactile_sub;

  // Publisher for nao speech
  ros::Publisher speech_pub;

  // Publisher for nao vocabulary parameters
  ros::Publisher voc_params_pub;

  // Client for starting speech recognition
  ros::ServiceClient recog_start_srv;

  // Client for stoping speech recognition
  ros::ServiceClient recog_stop_srv;
 
  // Client for stoping walking
  ros::ServiceClient walk_stop_srv;

  // Subscriber to speech recognition
  ros::Subscriber recog_sub;

  // Publisher to nao walking
  ros::Publisher walk_pub;
  
  //Publisher to face
  ros::Publisher face_pub;

  // Subscriber for foot contact
  ros::Subscriber footContact_sub;

  // Client for prespeech
  ros :: ServiceClient prespeech_srv;

  // Client for prepose
  ros :: ServiceClient prepose_srv;

  // Client for face_recognition
  ros :: ServiceClient facerecog_srv;

  // Client for markerdection
  ros :: ServiceClient head_srv;

  // Client for object_detection
  ros :: ServiceClient objdect_srv;


public:

  boost::thread *spin_thread;

  // Create the action client
  actionlib::SimpleActionClient<speech::BlinkAction> my_actionClient;
  naoqi_bridge_msgs::SpeechWithFeedbackActionGoal speech;
  bool foot_contact;
  bool walkflag = true;
  Nao_control(): my_actionClient(nh_, "blink", true) // Init the action client: target the /blink action server. Here "true" causes the action client to spin its own thread
  {
    // Subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
    tactile_sub=nh_.subscribe("/tactile_touch",1, &Nao_control::tactileCallback, this);

    speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);

    voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);

    prespeech_srv = nh_.serviceClient <speech::prespeech>( "/prespeech" ) ;

    prepose_srv = nh_.serviceClient <speech::prespeech>( "/prepose" ) ;

    facerecog_srv=nh_.serviceClient<std_srvs::Empty>("/facerecognition");

    head_srv=nh_.serviceClient<std_srvs::Empty>("/Risehead");

    objdect_srv=nh_.serviceClient<detection::mode1>("/Mode_change");

    recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");

    recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");

    walk_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_walk_srv");

    recog_sub=nh_.subscribe("/word_recognized",1, &Nao_control::speechRecognitionCallback, this);

    footContact_sub = nh_.subscribe<std_msgs::Bool>("/foot_contact", 1, &Nao_control::footContactCallback, this);

    walk_pub=nh_.advertise<geometry_msgs::Pose2D>("/cmd_pose", 3);

    face_pub=nh_.advertise<std_msgs::String>("/facename", 3);

    stop_thread=false;
    
    spin_thread=new boost::thread(&spinThread);
  }
  ~Nao_control()
  {
    stop_thread=true;
    sleep(1);
    spin_thread->join();
  }
  
  //Get the current foot-contact information
  void footContactCallback(const std_msgs::BoolConstPtr& contact)
  {
    /*get the contact information*/
    foot_contact=contact->data;
  }
  
  //judge whether the words could be accruately recognition
  void speechRecognitionCallback(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
  {
     /*memorize the words and store them in a big string as a whole sentence*/
     ROS_INFO("recognizing now");
     speech.goal_id.id="second";
     
     for (int i=0;i<msg->words.size();i++)
     { 
       ROS_INFO_STREAM("The confidence_values is:"<<msg->confidence_values[i]);
       
       //check if the word is in the vocalbulary using the confidence values
       if (msg->confidence_values[i]>=0.45)
        {
          speech.goal.say+= msg->words[i];
        }
       else
        {
          speech.goal.say+= "unknown";
        }       
       speech.goal.say+=" "; 
     }
     ROS_INFO_STREAM("The recognized result is:"<<speech.goal.say);
      
  }
  /***********************************************Start button*******************************************************************************/
  void tactileCallback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& tactileState)
  { 
    if (tactileState->button==tactileState->buttonFront)
    {
      if (tactileState->state == tactileState->statePressed)
      {
        ROS_WARN_STREAM("front tactile has been Pressed");       
        /*set vocabulary parameters*/
        naoqi_bridge_msgs::SetSpeechVocabularyActionGoal voc_param;
        voc_param.goal_id.id="first";
        voc_param.goal.words.push_back("bring");
        voc_param.goal.words.push_back("jack");
        voc_param.goal.words.push_back("Tom");
        voc_param.goal.words.push_back("cup");
        voc_params_pub.publish(voc_param);              
        pre_task();
      }
    }
  
    // if (tactileState->button==tactileState->buttonMiddle)
    // {
    //   if (tactileState->state == tactileState->statePressed)
    //   {
    //     ROS_WARN_STREAM("middletactile has been Pressed");
    //   }
    // }
  /***********************************************Stop button******************************************************************************/
    if (tactileState->button==tactileState->buttonRear)
    {
       if (tactileState->state == tactileState->statePressed)
           { 
             ROS_WARN_STREAM("Rear tactile has been Pressed");
             stopWalk();              
            }
    }                 
  
  }
  
  /*********************************************************
  // Call the service to say "Can i help you"
     and wait for command
  **********************************************************/

  void pre_task()
  {
    //speech.goal.say="";
    speech::prespeech emp0;
    std_srvs::Empty emp1;
    emp0.request.req=1;
    if(prespeech_srv.call(emp0))
      {
         ROS_INFO("prespeech");
          /*start recognition*/
          ros::Time ti_0, tf_0;
          ti_0=ros::Time::now();
          tf_0=ti_0;
          ros::Rate r(60);
          if(recog_start_srv.call(emp1))
            {
              ROS_INFO("start recognition");
              ROS_INFO("Waiting for command");                     
            }
          //Waiting time for command
          while((tf_0.toSec()-ti_0.toSec())<=10)
            {  
              tf_0=ros::Time::now();
              ros::spinOnce();
              r.sleep();
            }
      }
    first_task();
  }

  /*********************************************************
  // Judge if the robot can unterstand the command
  **********************************************************/
  void first_task()
  { 
      
       /*stop recognition*/
      std_srvs::Empty emp2;
      if(recog_stop_srv.call(emp2))
        {
          ROS_INFO("stop recognition");
        }
      //Set certain command to compare with
      string s1,s2,s3;
      s1 = speech.goal.say;
      s2 = "bring jack cup "; 
      s3 = "bring Tom cup ";
      
      
      /***********************************************************Situation: Do not understand***********************************************************/
      /***********************************************************Go back to pretask()*********************************************************************/
      if ((s1.compare(s2))!= 0)
        {
            std::istringstream buf(s1);
            std::istream_iterator<std::string> beg(buf), end;
            std::vector<std::string> tokens(beg, end);
            speech::prespeech emp_an2;
            emp_an2.request.req=3;
            if(prespeech_srv.call(emp_an2))
                {
                    ROS_INFO("Do not Unterstand");
                }

            if ((tokens[1]!= "jack") || (tokens[1]!= "Tom")) 
              {
                speech::prespeech emp_an3;
                emp_an3.request.req=4;
                prespeech_srv.call(emp_an3);                             
              } 
            speech.goal.say="";
            pre_task();         
        }
     /**************************************************************Situation: Understand****************************************************************/
      if (((s1.compare(s2)) == 0)|| ((s1.compare(s3)==0)))
        {
          std::istringstream buf1(s1);
          std::istream_iterator<std::string> beg1(buf1), end1;
          std::vector<std::string> tokens1(beg1, end1);
          
          // Publish the command name to the face recognition part for further searching 
          if(tokens1[1]== "jack")
          {
              ///new modify
              std_msgs::String face_name;
              face_name.data = "jack";
              face_pub.publish(face_name);
              face_pub.publish(face_name);
              face_pub.publish(face_name);

          }
          if(tokens1[1]== "Tom")
          {
              ///new modify
              std_msgs::String face_name1;
              face_name1.data = "Tom";
              face_pub.publish(face_name1);
              face_pub.publish(face_name1);
              face_pub.publish(face_name1);
          }          
        speech::prespeech emp_an1;
        emp_an1.request.req=2;
        
        //Situation: Understand
        if(prespeech_srv.call(emp_an1))
          {
            ROS_INFO("Unterstand");
            ROS_INFO("Ready for task");
            speech::prespeech ini_pose;
            ini_pose.request.req=1;
            //stand for the task
            if(prepose_srv.call(ini_pose))
              {
                ROS_INFO("Start ot find the object");
                sleep(5);
                std_srvs::Empty objemp;
                std_srvs::Empty heademp;
                head_srv.call(heademp);
                std_srvs::Empty emp_stop;
                /*stop recognition*/
                if(recog_stop_srv.call(emp_stop))
                    {
                    ROS_INFO("stop recognition");
                    }
                
                //start object detection to find the goal object
                
                detection::mode1 srv_detection;
                srv_detection.request.req = 1;
                objdect_srv.call(srv_detection);
              }
          }
      }        

  }

  /*********************************************************
  // Emergnecy stop
  **********************************************************/

  void main_loop(ros::Rate rate_sleep)
  {

    while(nh_.ok())
    {
      /*stop if the foot contact the ground*/
      if(!foot_contact)
      { 
        //ROS_INFO("Do make NAO leave the ground !!!!!!!");
        stopWalk();
      }
      rate_sleep.sleep();
    }
  }

  void walker(double x, double y, double theta)
  {
    geometry_msgs::Pose2D Dpostion;
    Dpostion.x = x;
    Dpostion.y = y;
    Dpostion.theta = theta;
    walk_pub.publish(Dpostion);
  }

  void stopWalk()
  {      
    std_srvs::Empty emp3;
    if( walk_stop_srv.call(emp3))
      {
        //ROS_INFO("stop walking");
      }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_tutorial_3");
  Nao_control TermiNAOtor;
  ros::Rate rate_sleep(10);

  ROS_INFO_STREAM("Started the Client");
  // Waiting for NAO blink action server to start (roslaunch nao_apps leds.launch):
  TermiNAOtor.my_actionClient.waitForServer();

  // Get into the main loop:
  TermiNAOtor.main_loop(rate_sleep);

  return 0;
}
