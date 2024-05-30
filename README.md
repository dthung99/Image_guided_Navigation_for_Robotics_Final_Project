# Image_guided_Navigation_for_Robotics_Final_Project
This git repository is my submission for Image-guided Navigation for Robotics Module at King's College London.


The project aim to create an extension in 3D slicer for planning a straight path for a needle to go into the brain and then send the information for simulation in ROS.



How to install:
Before using the program, you will need to install 3D slicer on your Window computer, and ROS 1 with noetic distribution on a virtual machine on the same computer
The system might still work if you decided to install 3D slicer and ros-noetic distro in a different way. However, the network setup between them will be different and not covered here.

After you install 3D slicer, you need to install IGT
After that, download this repository, and unzip it,

Choose Extension wizard module, select extension.
Choose <Your download folder>/Needle_Path_planning

And load the modules.

Open the modules, and upload your data.

Run and wait !!!. I did test on a very big dataset and it will take about 8-20s depending on your requirement
Then create model and prepare to send them to ROS!!!

Go to IGT entension
Create a new server.
Remember to turn it to Active !!!




In ROS

You will need to create you workspace following this tutorial
You will need to install MoveIt package following this tutorial
You will need to install ROS-IGTL-Bridge package in your workspace

Download and unzip repository, copy two package moveit_needle_sim and needle_path_simulation into the src folder of your workspace
mv <Your download folder>/ROS_needle_insert_simulation_ws/src <Your workspace>/src

Build the package
catkin_make

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
