
*****************************************************************************************
Package name: using_markers
Operating environment: ROS kinetic
Author: Yansong Wu
*****************************************************************************************

Usage:
1. Generate navigation trajectory
2. Control the robot NAO go through the predined workspace
3. Display a map with NAO，obstacles, target and planned trajectory on the rviz    

************************************************************************

Input:
1. The destination position wrt NAO is received through the server /naviDestination
2. The initial localization and obstacles position for navigation are recieved through the server /naviObstacle
3. The initial localization and obstacles position for the map are recieved through the server /drawmap
4. The real-time localization information for the map is subsrcibed on the topic /naoposition

Output:
1. The control command to control the movement of NAO is sended through the client on the server /WalkPolar
2. After the navigation finishes, it will call the server /Mode_change to trigger the grasping part

************************************************************************

Run the code:
$ roslaunch using_markers map.launch

Then a rviz window appears. After recieving the localization, target and ovbstacles positions. A map is generated 
on the rviz. Green cubes refer to obstacles. Red points mean the waypoints. Blue ball refers to the target. Then a
trajectory is planned. NAO will wolks along the trajectory, until there is no more obstacle between NAO and the 
target. Then it will trun to the target and  call the server /Mode_change. Then the grasping part beginns.

