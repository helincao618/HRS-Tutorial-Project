/*******************************************************************************************/
Important libraries (dependencies) which needs to be installed before
/*******************************************************************************************/
1.dlib
2.face_recognition

How to install dlib:
$ git clone https://github.com/davisking/dlib.git
$ cd dlib
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build
$ cd ..
$ python2(3) setup.py install

How to install face_recognition:
$ git clone https://github.com/ageitgey/face_recognition.git
$ pip2(3) install face_recognition

/*******************************************************************************************/
Packages needed for the whole task
/*******************************************************************************************/
Packages:

darknet_ros (not included in zip file)
speech
using_markers
detection

Details can be seen in different Readme in each package 
/*******************************************************************************************/
How to implement the whole task
/*******************************************************************************************/
1.$ roslaunch nao_bringup nao_full_py.launch

2.$ roslaunch nao_apps speech.launch 

3.$ roslaunch nao_apps tactile.launch 

4.$ roslaunch speech yuan.launch 

5.$ roslaunch detection darknet_ros_bottom.launch

6.$ roslaunch speech cong.launch

7.$ rosrun detection detection

8.$ roslaunch speech cao.launch

9.$ roslaunch using_markers map.launch

10.$ rosrun speech test_node
