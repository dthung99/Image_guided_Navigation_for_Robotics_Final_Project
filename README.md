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

Additionally, you will need to install `OpenIGTLinkIF` extension on **3D slicer** and `ROS-IGTL-Bridge` package on your **ROS workspace** for communication; and `MoveIt` package in ROS for controlling the robot.
- [Extension installation on 3D slicer](https://slicer.readthedocs.io/en/latest/user_guide/extensions_manager.html)
- [ROS-IGTL-Bridge package installation on ROS](https://github.com/openigtlink/ROS-IGTL-Bridge)
- [MoveIt installation](https://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/getting_started/getting_started.html#)

# How to install the repository 
## 3D slicer
Download this repository and unzip it on your Window. Or

	git clone https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project.git

Open 3D slicer, choose 
Choose Extension wizard module, select `Developer Tools` -> `Extension Wizard`.

![image](https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project/assets/155381330/3bdb9e5d-bc9c-4927-b3fd-ae484df0ef61)

In `Extension Wizard`, select `Select Extension`, go to `<your_download_folder>/Needle_Path_planning`. And click `Select Folder`. A window will pop up to ask you to load the extension. Click `Yes`

![image](https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project/assets/155381330/0e099e5c-dbdf-4611-9459-1f758e4713c1)

You now can use the extension as other extension in 3D slicer !!!

![image](https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project/assets/155381330/52600035-8068-47ed-9cb9-7868822996f4)

The extension take `vtkMRMLMarkupsFiducialNode` and `vtkMRMLLabelVolumeNode` as input and plan a path to the brain. Make sure to use correct data type!

Run and wait !!! I did test on a very big dataset and it will take about 8-20s depending on your requirements
Then select `Prepare data to send to ROS` to prepare data for sending to ROS !!!

## ROS
Download this repository and unzip it on your Virtual Machine. Or

	git clone https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project.git

Copy two package `moveit_needle_sim` and `needle_path_simulation` from `<your_download_folder>/ROS_needle_insert_simulation_ws/src` into `<your_workspace>/src` folder in your workspace. It is expected that you have already install [ROS-IGTL-Bridge](#prerequisites) in this workspace.

Go to your workspace

	cd <your_workspace>

Compile the code

	catkin_make

Source your workspace in every new terminal you open

	source <Your workspace>/devel/setup.bash
OR

 	echo "source ~/<Your workspace>/devel/setup.bash" >> ~/.bashrc

## Streamming from 3D slicer to ROS and run simulation !!!
In 3D slicer, after you click `Prepare data to send to ROS`:
- Open `OpenIGTLinkIF` extension.
- In `Connectors`, click `+`. Then select `Server` and `Active` in `Properties` section.
- Default `Hostname` and `Port` should be used, which are expected to be `localhost` and `18944`.
- Expand the `I/O Configuration`, add the `vtkMRMLMarkupsFiducialNode` and `vtkMRMLModelNode` that you want to send
- IMPORTANT: you can only send `vtkMRMLMarkupsFiducialNode` whose control points' names are "entry_point" and "target_point". These are created for you in `ROS_entry_and_target_points`

![image](https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project/assets/155381330/790a0858-afcd-4c97-85dd-e7a409373b6f)

In ROS:
- Open _**Two Terminals**_. Remember to source your workspace.
- In one terminal, visualize the robot model:

	  roslaunch moveit_needle_sim demo.launch
- In the other terminal, run the communication and control nodes (Only run this node _**AFTER**_ turning the 3D `OpenIGTLinkIF` server _**ON**_:

	  roslaunch needle_path_simulation needle_insertion.launch
- Follow the instruction of ROS. `RvizVisualToolsGui` is an essential component to interact with the code. If you don't see it, select `Panels` -> `RvizVisualToolsGui`

![image](https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project/assets/155381330/2629aeb2-fa2b-4a1a-bc02-0e09d2cd350f)

# Debugging
## catkin_make fails :(
- Make sure you have install all the necessary package: ros_igtl_bridge, moveit
- Run `rospack list` to see what package are already installed. Normally, I would see ros_igtl_bridge, moveit_visual_tools, moveit_ros_core, moveit_msgs, and other moveit_... packages. If you can't find them, check your installation again. An example for install ROS package is:

	  sudo apt-get update
	  sudo apt-get install ros-noetic-moveit
	  sudo apt-get install ros-noetic-moveit-visual-tools
- Remember to source your workspace.
- If you continue to fail, reinstall ROS and MoveIt.

## Fails to compute the plan everytime
- The brain and the needle path might be at an angle that causes the robot to colide itself. You can:
  + Try to rotate ± translate the brain model (`vtkMRMLModelNode`) and needle path (`vtkMRMLMarkupsFiducialNode`) using [Transforms module](https://slicer.readthedocs.io/en/latest/user_guide/modules/transforms.html) to a different pose that enable the robot to move.
  + ![image](https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project/assets/155381330/e3966376-5349-4c84-a6e3-8a6f135abdb0)
  + Change the origins of the robot: you can move the robot around the world coordinate by changing the URDF file in `<your_workspace>/src/needle_path_simulation/urdf/standard_test_Robot.urdf`. The metrics that is advised to move is the origin xyz of `<joint name="base_to_part_1" type="revolute"> ... </joint>`.
  + Try to set `jump_threshold: 5.0` into `jump_threshold: 0.0`, or increase the time allowed for planning `planning_time_limit` in `<your_workspace>/src/needle_path_simulation/config/user_parameters.yaml`.

## The robot do not move/The text do not change after clicking _**Next**_
- Restart 3D slicer. Shut down and relaunch the ROS Nodes
- Remember to turn 3D Slicer `OpenIGTLinkIF` server on before running ROS. There will be an error messages in the console if they can not connect.
- Only open _**ONE**_ 3D slicer app. Opening two 3D servers might cause conflict.
- Only launch _**ONE**_ time for each `roslaunch`. You might accidentally launch one launch files multiple times in one run.
- Waiting might be an option! Sometime you might send a very heavy mesh to ROS (if your input label map is very big). Or you might set the velocity of the robot too low (by changing the user_parameters.yaml)

# Personalisation 
- You can change a few paramerter in

	  <Your workspace>/src/needle_path_simulation/config/user_parameters.yaml
- If you want to use your own robot
  + First configurate it using `MoveIt`. `MoveIt Setup Assistant` offers a user friendly way for setting up your own robot ([MoveIt Setup Assistant](https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html)).
    
  + Change the parameters in `user_parameters.yaml` to fit your new robot, especially the `planning_group`, `base_link`, and `needle_orientation_in_end_effector_frame`
    
  + Controlling your robot with:

	    roslaunch <your_robot_moveit-configuration> demo.launch
	    roslaunch needle_path_simulation needle_insertion.launch

- You can always change the different options in th visibility of `MotionPlanning` so that you have a comfortable experience.

# Note
To Fully visualize the simulation, make sure in the `Displays` tab, the following subsribers are available:
- `MotionPlanning`
- `MarkerArray`: Topic should be /rviz_visual_tools
- `Marker`: Topic should be /brain_mesh

![image](https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project/assets/155381330/9c1eb122-8a35-45b2-af95-9184465bebcc)

When you prepare to move the robot in each stage, there will be a purple phantom that move before the robot move. If you don't want to see it, turn it off in `MotionPlanning` -> `Planned Path` -> `Show Robot Visual`. Or you can also speed it up/change its color there.



Best wishes
Hung
Dang The Hung





