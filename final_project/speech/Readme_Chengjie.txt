*****************************Chengjie Yuan****************************************************
Package name: Speech
Operating environment: ROS kinetic
************************************************************************************************
This package contains different functions, not only the speech part, because some functions are
implemented in the scripts using python API (ALMotion API)
Functions
************************************************************************************************
Control part: 
1. Set different initial positions(convenient for testing)
2. Set pregrasp position for the robot to further grasp
3. Tuning e.g. head operation for better view, hand operation for grasp
4. Specific walking pose to make sure the robot hold the object tightly while walking

Visual part:
1. Recognize different faces
2. Detect Aruco Makers and get Information
3. Compute the current position and the obstacle position based on the Aruco Marker

Speech part:
1.Talk with the people and get the command with speech recognition
2.Define different certain sentences for different situations e.g. "Can I help you", "Understand" etc.
************************************************************************************************
Interfaces
************************************************************************************************
Input :
1. When speech part works, considering different situations, Nao can talk through the service server /prespeech
2. Different poses can be initialized through the the service server /prepose
3. When reaching the goal position, NAO will bend its body to grasp object through the service server /Movejoints and /hand_operation
4. When needs tune for better view,these service server will be used  /Shakehead, /Downhead, /Risehead
5. Specific walking pose is set through the service server /WalkBack
6. Face recognition is implemented through the service server /facerecognition and the known face name will be subsrcibed from topic /facename
7. The initial localization and obstacles position are computed through the suscriber topic /detectedmarker

Output
1.When understanding the command, next task object detection will be called through  /Mode_change, at the same time the known face name will be published to /facename
2.The important information of the Aruco Markers (Rvec,Tvec) is published to /detectedmarker
3. The initial localization and obstacles position for navigation will be given by calling /naviObstacle
4. The initial localization and obstacles position for the map will be given by calling  /drawmap
5. The real-time localization information for the map will be published to the topic /naoposition

*************************************************************************************************************************
How to implement this package
*************************************************************************************************************************
1. $ roslaunch speech yuan.launch
2. $ roslaunch speech cong.launch
3. $ roslaunch speech cao.launch
4. $ rosrun speech test_node