Readme od  tutorial6 - Introduction to cartesian control on NAO 
Team member(Group A): Helin Cao, Yansong Wu, Chengjie Yuan, Cong Wang 
Robot: NAO 
Compile: catkin_make
execute: 
$ roslaunch nao_control_tutorial_2 nao.launch 
$ rosrun nao_control_tutorial_2 naoControl2 

After that,you will see the pose of NAO's left hand in the terminal. It will first raise its left hand and take it down back. Then, the published tf of marker appreas automaticly in the rviz. If you put a marker in the front of the robot, the robot will catch the marker with the proper hand(the default mode is to catch only based on the spatial position). To switch the mode, just change the value of variable "bonus" in the beginning of the py file from 0 to 1. Then, the robot will work in the bonus mode. That means, not only the position but also the orientation will be took into consideration. Because of the limitation of its arm's structrue, in this mode NAO cannot always touch the marker. But in the rviz, man can easily detect that, its hand framework moves closely to marker's framework.
