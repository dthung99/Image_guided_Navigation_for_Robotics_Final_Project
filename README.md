# Image_guided_Navigation_for_Robotics_Final_Project
This git repository is my submission for Image-guided Navigation for Robotics Module at King's College London.


mkdir -p ~/needle_insert_simulation_ws/src

download
setup ROS-IGTL-Bridge

cd ~/needle_insert_simulation_ws
catkin_make

source ~/needle_insert_simulation_ws/devel/setup.bash

echo "source ~/needle_insert_simulation_ws/devel/setup.bash" >> 
~/.bashrc




https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project.git 

git clone --depth 1 --single-branch --branch main https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project.git ROS_needle_insert_simulation_ws

git clone https://github.com/dthung99/Image_guided_Navigation_for_Robotics_MSc_Project.git
mv .\Image_guided_Navigation_for_Robotics_MSc_Project\ROS_needle_insert_simulation_ws\src\ .
rm -rf .\Image_guided_Navigation_for_Robotics_MSc_Project\
