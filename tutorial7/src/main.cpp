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
#include <nao_control_tutorial_3/BlinkAction.h>



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

  // Subscriber to bumpers states
  ros::Subscriber bumper_sub;

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

  // Subscriber for foot contact
  ros::Subscriber footContact_sub;

public:

  boost::thread *spin_thread;

  // Create the action client
  actionlib::SimpleActionClient<nao_control_tutorial_3::BlinkAction> my_actionClient;
  naoqi_bridge_msgs::SpeechWithFeedbackActionGoal speech;
  bool foot_contact;
  bool walkflag = true;
  Nao_control(): my_actionClient(nh_, "blink", true) // Init the action client: target the /blink action server. Here "true" causes the action client to spin its own thread
  {

    // Subscribe to topic bumper and specify that all data will be processed by function bumperCallback
    bumper_sub=nh_.subscribe("/bumper",1, &Nao_control::bumperCallback, this);

    // Subscribe to topic tactile_touch and specify that all data will be processed by function tactileCallback
    tactile_sub=nh_.subscribe("/tactile_touch",1, &Nao_control::tactileCallback, this);

    speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);

    voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);

    recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");

    recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");

    walk_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_walk_srv");

    recog_sub=nh_.subscribe("/word_recognized",1, &Nao_control::speechRecognitionCallback, this);

    footContact_sub = nh_.subscribe<std_msgs::Bool>("/foot_contact", 1, &Nao_control::footContactCallback, this);

    walk_pub=nh_.advertise<geometry_msgs::Pose2D>("/cmd_pose", 3);

    stop_thread=false;
    spin_thread=new boost::thread(&spinThread);
  }
  ~Nao_control()
  {
    stop_thread=true;
    sleep(1);
    spin_thread->join();
  }

  void footContactCallback(const std_msgs::BoolConstPtr& contact)
  {
    /*get the contact information*/
    foot_contact=contact->data;

  }

  void speechRecognitionCallback(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg)
  {
     /*memorize the words and store them in a big string as a whole sentence*/
     ROS_INFO("recognizing now");
     speech.goal_id.id="second";
     
     for (int i=0;i<msg->words.size();i++)
     { 
       speech.goal.say+= msg->words[i];
       speech.goal.say+=" "; 
     }
     ROS_INFO_STREAM("The recognized result is:"<<speech.goal.say);
      
  }

  void bumperCallback(const naoqi_bridge_msgs::Bumper::ConstPtr& bumperState)
  {
    nao_control_tutorial_3::BlinkGoal blinkGoal;
    std_msgs::ColorRGBA color;
    std::vector<std_msgs::ColorRGBA> colors;
    bool blinkFlag = false;
    double time(0.0);

    if (bumperState->bumper==bumperState->right)
    {
      if (bumperState->state == bumperState->statePressed){
        blinkFlag = true;
        ROS_WARN_STREAM("Right Bumper has been Pressed");
        color.r = 0;
        color.g = 1;
        color.b = 0;
        color.a = 1;
        colors.push_back(color);
        time = 4.0;
      }
    }
    else if(bumperState->bumper==bumperState->left)
    {
      if (bumperState->state == bumperState->statePressed){
        blinkFlag = true;
        ROS_WARN_STREAM("Left Bumper has been Pressed");
        color.r = 1;
        color.g = 0;
        color.b = 0;
        color.a = 1;
        colors.push_back(color);
        time = 2.0;
      }
    }
    else
    {
      blinkFlag = false;
    }

    if(blinkFlag == true)
    {
      blinkGoal.colors = colors;
      blinkGoal.blink_duration = ros::Duration(time);
      blinkGoal.blink_rate_mean = 1.0;
      blinkGoal.blink_rate_sd = 0.1;
      my_actionClient.sendGoal(blinkGoal, this->doneBlinkCallback, this->activeBlinkCallback, this->feedbackBlinkCallback);
    }
    else
    {
      // Must be called to preempt the current action otherwise NAO stays in an infinite blinking state...
      my_actionClient.cancelAllGoals();
    }
  }

  // Called once when the blink goal completes
  static void doneBlinkCallback(const actionlib::SimpleClientGoalState& state, const nao_control_tutorial_3::BlinkResultConstPtr& result)
  {
    ROS_INFO_STREAM("Finished in state: "<< state.toString().c_str());
  }

  // Called once when the blink goal becomes active
  static void activeBlinkCallback()
  {
    ROS_INFO("Blink goal just went active");
  }

  // Called every time feedback is received for the blink goal
  static void feedbackBlinkCallback(const nao_control_tutorial_3::BlinkFeedbackConstPtr& feedback)
  {
    ROS_INFO_STREAM("Got the following Feedback: "<< feedback->last_color);
  }

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
        voc_param.goal.words.push_back("thank");
        voc_param.goal.words.push_back("you");
        voc_param.goal.words.push_back("guys");
        voc_params_pub.publish(voc_param);
        std_srvs::Empty emp1;
        /*start recognition*/
        if(recog_start_srv.call(emp1))
        {
         ROS_INFO("start recognition");
        }
      }
    }
    if (tactileState->button==tactileState->buttonMiddle)
    {
      if (tactileState->state == tactileState->statePressed){
        ROS_WARN_STREAM("middletactile has been Pressed");
        std_srvs::Empty emp2;
        /*stop recognition*/
        if(recog_stop_srv.call(emp2))
            {
             ROS_INFO("stop recognition");
            }
        /*say the sentence*/
        speech_pub.publish(speech);
        /*clear the stack*/
        speech.goal.say="";
      }
    }

    if (tactileState->button==tactileState->buttonRear)
    {
       if (tactileState->state == tactileState->statePressed)
           { 
             ROS_WARN_STREAM("Rear tactile has been Pressed");
             if (walkflag == true)
             {/*walk along the straight line*/
              walker(0.5,0.0,0.0);
              /*wait until action finished*/
              sleep(10);
              /*turn left*/
              walker(0.0,0.0,1.57);
              /*wait until action finished*/
              sleep(6);
              /*turn right*/
              walker(0.0,0.0,-1.57);
              /*nao will stop if rear tactile was pressed again*/
              walkflag = false;
             }
             else
             {
               stopWalk();
               /*nao will start if rear tactile was pressed again*/
               walkflag = true;
             }
             
                  
            }
    }
                  
           
        
  }


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
