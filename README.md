# Image_guided_Navigation_for_Robotics_Project
This git repository is my submission for Image-guided Navigation for Robotics Module at King's College London. A video for demonstration of how to use it is available [here](https://emckclac-my.sharepoint.com/:v:/g/personal/k23133931_kcl_ac_uk/EZk3IHEwcylDiV8dwL06d54B1hmGNu8GzWmnpYw1AYWLkQ?e=YqZXIY&nav=eyJyZWZlcnJhbEluZm8iOnsicmVmZXJyYWxBcHAiOiJTdHJlYW1XZWJBcHAiLCJyZWZlcnJhbFZpZXciOiJTaGFyZURpYWxvZy1MaW5rIiwicmVmZXJyYWxBcHBQbGF0Zm9ybSI6IldlYiIsInJlZmVycmFsTW9kZSI6InZpZXcifSwicGxheWJhY2tPcHRpb25zIjp7fX0%3D)

# Introduction
The project aim to create an extension in 3D slicer for planning a straight path for a needle to go into the brain and then send the information for simulation in ROS.

# Prerequisites
Before using the program, you will need to install **_3D slicer_** on your **_Window Computer_**, and **_ROS 1_** with **_Noetic Distribution_** on a **_Virtual Machine_** on the **_same computer_**.
- [3D slicer installation](https://download.slicer.org/)
- [Virtual machine installation](https://www.virtualbox.org/wiki/Downloads)
- [ROS Noetic Ninjemys installation](https://wiki.ros.org/ROS/Installation)

Note: The system might still work if you decided to install 3D slicer and ROS in a different way. However, the network setup between them might be different and you might need to edit the **user_parameters.yaml** file.

Additionally, you will need to install **OpenIGTLinkIF** extension on **3D slicer** and **ROS-IGTL-Bridge** package on your **ROS workspace**.
- [Extension installation on 3D slicer](https://slicer.readthedocs.io/en/latest/user_guide/extensions_manager.html)
- [ROS-IGTL-Bridge package installation on ROS](https://github.com/openigtlink/ROS-IGTL-Bridge)

# How to install the repository 
## 3D slicer
Download this repository and unzip it on your Window. Or

	git clone https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project.git

Open 3D slicer, choose 
Choose Extension wizard module, select `Developer Tools` -> `Extension Wizard`.
![image](https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project/assets/155381330/3bdb9e5d-bc9c-4927-b3fd-ae484df0ef61)

In `Extension Wizard`, select `Select Extension`, go to \<your_download_folder\>/Needle_Path_planning. And click `Select Folder`. A window will pop up to ask you to load the extension. Click `Yes`
![image](https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project/assets/155381330/0e099e5c-dbdf-4611-9459-1f758e4713c1)

You now can use the extension as other extension in 3D slicer!!!
![image](https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project/assets/155381330/52600035-8068-47ed-9cb9-7868822996f4)

The extension take `vtkMRMLMarkupsFiducialNode` and `vtkMRMLLabelVolumeNode` as input and plan a path to the brain. Make sure to use correct data type!

Run and wait !!!. I did test on a very big dataset and it will take about 8-20s depending on your requirements
Then `select Prepare data to send to ROS` to prepare data for sending to ROS!!!

## ROS
Download this repository and unzip it on your Virtual Machine. Or

	git clone https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project.git

Copy two package `moveit_needle_sim` and `needle_path_simulation` from \<your_download_folder\>/ROS_needle_insert_simulation_ws/src into \<your_workspace\>/src folder in your workspace. It is expected that you have already install [ROS-IGTL-Bridge](#prerequisites) in this workspace.

Go to your workspace `cd <your_workspace>`

Compile the code `catkin_make`

Source your workspace in every new terminal you open

	source <Your workspace>/devel/setup.bash
OR
echo "source ~/<Your workspace>/devel/setup.bash" >> ~/.bashrc


RUNNNNN
!!! REMEMBER TO TURN 3D SLICER SERVER ON BEFORE RUNNING !!!
roslaunch moveit_needle_sim demo.launch

Open a new terminal (Remember to source)
roslaunch needle_path_simulation needle_insertion.launch

FOLLOWING THE INSTRUCTION
REMEMBER TO TURN RvizVisualToolsGui ON. If you don't see it, select Panels -> RvizVisualToolsGui
To Fully visualize the simulation, make sure in the Displays tab, you have the following subsriber
+ MotionPlanning
+ MarkerArray: Topic should be /rviz_visual_tools
+ Marker: Topic should be /brain_mesh

When you prepare to move the robot, there will be a purple phantom that move before the robot move. If you don't want to see it, turn it off in MotionPlanning -> Planned Path -> Show Robot Visual/Collision




Go to IGT entension
Create a new server.
Remember to turn it to Active !!!






PERSONALIZE
- You can change a few paramerter in <Your workspace>/src/needle_path_simulation/config/user_parameters.yaml
- If you want to use your own robot
+ You can configurate it using MoveIt, especially MoveIt Setup Assisstance which offer a user friendly set up.
+ Change the parameters in user_parameters.yaml to fit your new robot
+ To control your robot:
roslaunch <Your robot move it package> demo.launch
roslaunch needle_path_simulation needle_insertion.launch


DEBUGGING
// If catkin_make fail, run
rospack list 
// To see if you have ros_igtl_bridge, moveit_visual_tools, moveit_ros_core, moveit_msgs, and other moveit package install!!! To install them:
sudo apt-get update
sudo apt install ros-noetic-moveit
sudo apt-get install ros-noetic-moveit-visual-tools
// If continnue to fail, make sure you have source the workspace
// If you continue to fail Reinstall ROS and MoveIt
