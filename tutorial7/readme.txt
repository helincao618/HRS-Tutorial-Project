Readme tutorial7 
Team member(Group A): Helin Cao, Yansong Wu, Chengjie Yuan, Cong Wang 
Robot: NAO 
Compile: 
$ catkin_make
execute: 
$ roslaunch nao_bringup nao_full_py.launch
$ roslaunch nao_control_tutorial_3 nao3.launch

For exercise2, the vocabulary contains 'thank' 'you' and 'guys'.The recognition result can be seen on terminal.
For exercise3, firstly, you should make sure that, NAO is in a stand position and set stiffness enable in the terminal.($ rosservice call /body_stiffness/enable).
You can press the rear tactile and the nao will walk straight forward, turn left and turn right. 

