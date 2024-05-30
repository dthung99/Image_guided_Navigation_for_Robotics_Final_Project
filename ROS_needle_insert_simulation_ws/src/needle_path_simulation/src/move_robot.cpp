// The code was adapted from https://github.com/moveit/moveit_tutorials/blob/melodic-devel/doc/move_group_interface/src/move_group_interface_tutorial.cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// IGTlink msg
#include <ros_igtl_bridge/igtlpoint.h>
#include <ros_igtl_bridge/igtlpolydata.h>
#include <visualization_msgs/Marker.h>

namespace rvt = rviz_visual_tools;

// Declare some parameters
// Some moveit parameters
geometry_msgs::Pose initial_pose; // Initial and target pose
geometry_msgs::Pose target_pose; // Initial and target pose
double planning_time_limit = 10.0; // Time limit for path planning
moveit_msgs::RobotTrajectory trajectory; // Trajectory
double jump_threshold = 0.0; // No large robot jump in joint space > 0.01 radian
double task_space_discretize_distance = 0.001; // The distance between consecutive waypoints along the Cartesian path when inserting the needle = 0.001 m
double starting_duration = 5.0; // Time to move from starting position to initial pose
double insert_needle_duration = 3.0; // Time to move from initial pose to target pose (insert needle)
double remove_needle_duration = 3.0; // Time to move from target pose to initial pose (remove needle)
double fraction; // How well the end effector follow the waypoints
bool success = true; // Variable to check if planning is success
moveit::planning_interface::MoveGroupInterface::Plan my_plan; // Call the planner to compute the plan and visualize it.
Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity(); // position in 3D for title
double GUI_text_height = 1.0;
// Pointer to control moveit
moveit_visual_tools::MoveItVisualTools* visual_tools_Ptr = nullptr;
moveit::planning_interface::MoveGroupInterface* move_group_Ptr = nullptr;
// Variable to communicate with 3D slicer
std::string entry_point_name = "entry_point"; // "entry_point_name";
std::string target_point_name = "target_point"; // "target_point_name";
double slicer_scale = 0.001; // The data in 3D slicer will be multiplied with the scale, 1mm -> 0.001
visualization_msgs::Marker brain_mesh; // Brain mesh to visualize the brain
visualization_msgs::Marker empty_brain_mesh; // Used to remove Brain mesh by publishing an empty mesh
ros::Publisher brain_mesh_pub; // Place holder for Brain mesh publisher
bool received_entry_point = false; // Turn to true after received entry pose
bool received_target_point = false; // Turn to true after received target pose
bool received_brain_mesh = false; // Turn to true after received brain model
int early_ending_counter = 3; // Countdown to 0 if user try to do path planning with no target

// ************************************************************************************************************
// ************************************************************************************************************
void set_needle_orientation_for_start_and_end_pose(tf2::Vector3& needle_orientation_in_end_effector_frame,
                                                   geometry_msgs::Pose& initial_pose,
                                                   geometry_msgs::Pose& target_pose);
void set_constraint_from_initial_orientation(geometry_msgs::Pose& initial_pose,
                                             moveit::planning_interface::MoveGroupInterface& move_group);
void show_friendly_GUI_text(moveit_visual_tools::MoveItVisualTools& visual_tools, std::string text);
void set_execute_duration_for_trajectory(moveit_msgs::RobotTrajectory& input_trajectory, double execute_duration);
bool move_through_list_of_points(std::vector<geometry_msgs::Pose>& waypoints, 
                                 moveit::planning_interface::MoveGroupInterface& move_group,
                                 const robot_state::JointModelGroup* joint_model_group,
                                 moveit_visual_tools::MoveItVisualTools& visual_tools,
                                 geometry_msgs::Pose& brain_model_pose,
                                 double velociy_scaling,
                                 double eef_step, // step to discretize in task space
                                 double execute_duration,
                                 std::string action);
void clean_exit(moveit::planning_interface::MoveGroupInterface& move_group,
                moveit_visual_tools::MoveItVisualTools& visual_tools);
void pointCallback(const ros_igtl_bridge::igtlpoint::ConstPtr& msg);
geometry_msgs::Point convertSlicerPoint32ToROSPoint64(const geometry_msgs::Point32& point32);
void convert_igtlpolydata_to_brain_mesh_Marker(const ros_igtl_bridge::igtlpolydata& data);
void polydataCallback(const ros_igtl_bridge::igtlpolydata::ConstPtr& msg);
// ************************************************************************************************************
// ************************************************************************************************************

int main(int argc, char** argv)
{
  // ************************************
  // Set up stuff
  // ************************************
  // ************************************
  // Declare some parameters
  // Some moveit parameters
  std::vector<geometry_msgs::Pose> waypoints; // List of via point in task space
  // Msg to get information from IGT link (Get data from 3D slicer)
  ros_igtl_bridge::igtlpoint slicer_input_points;
  ros_igtl_bridge::igtlpolydata slicer_input_polydata;
  // Define a pose for the brain_model (specified relative to frame_id)
  geometry_msgs::Pose brain_model_pose;
  brain_model_pose.orientation.w = 1.0;
  brain_model_pose.position.x = 0.0;
  brain_model_pose.position.y = 1.3;
  brain_model_pose.position.z = 1.0;
  // ************************************
  // Initiate ROS
  ros::init(argc, argv, "move_group_interface_control_node");
  ros::NodeHandle node_handle;
  // Get entry point, target point name and 3D slicer scale
  ros::param::get("/entry_point_name", entry_point_name);
  ros::param::get("/target_point_name", target_point_name);
  ros::param::get("/slicer_scale", slicer_scale);
  // Get other parameters
  ros::param::get("/planning_time_limit", planning_time_limit); // Get the planning time limit
  ros::param::get("/jump_threshold", jump_threshold); // Get jump_threshold to avoid snapping of robot
  ros::param::get("/starting_duration", starting_duration); // Time to move from starting position to initial pose
  ros::param::get("/insert_needle_duration", insert_needle_duration); // Time to move from initial pose to target pose (insert needle)
  ros::param::get("/remove_needle_duration", remove_needle_duration); // Time to move from target pose to initial pose (remove needle)
  ros::param::get("/GUI_text_height", GUI_text_height); // Get the height of GUI text in RVIZ
  ros::param::get("/task_space_discretize_distance", task_space_discretize_distance); // The distance between consecutive waypoints along the Cartesian path when inserting the needle = 0.001 m
  // Get needle_orientation_in_end_effector_frame
  double needle_x;
  double needle_y;
  double needle_z;
  ros::param::get("/needle_orientation_in_end_effector_frame/x", needle_x);
  ros::param::get("/needle_orientation_in_end_effector_frame/y", needle_y);
  ros::param::get("/needle_orientation_in_end_effector_frame/z", needle_z);
  tf2::Vector3 needle_orientation_in_end_effector_frame(needle_x, needle_y, needle_z);

  // Set single threading
  ros::AsyncSpinner spinner(1);
  spinner.start();
  // Setup the move group
  std::string planning_group = "robot_arm";
  ros::param::get("/planning_group", planning_group);
  moveit::planning_interface::MoveGroupInterface move_group(planning_group);
  move_group_Ptr = &move_group;
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(planning_group);
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // Visualization with the package MoveItVisualTools
  std::string base_link = "base_link";
  ros::param::get("/base_link", base_link);
  moveit_visual_tools::MoveItVisualTools visual_tools(base_link);
  visual_tools_Ptr = &visual_tools;
  visual_tools.deleteAllMarkers();
  // Remote control allows users to step through a high level script
  visual_tools.loadRemoteControl();
  // Load the marker pub to help visualisation
  visual_tools.loadMarkerPub();
  visual_tools.loadRvizMarkers();
  // Set position of text markers
  text_pose.translation().z() = GUI_text_height;
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();
  // ************************************
  // Getting Basic Information
  // Print the name of the reference frame, end-effector link, planning group
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str());
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));
  // Set the planning time limit
  move_group.setPlanningTime(planning_time_limit);
  // Setup brain mesh and empty_brain_mesh
  empty_brain_mesh.color.a = 1.0;
  empty_brain_mesh.header.frame_id = brain_mesh.header.frame_id = move_group.getPlanningFrame();
  empty_brain_mesh.ns = brain_mesh.ns = "triangle_list";
  empty_brain_mesh.id = brain_mesh.id = 5;
  empty_brain_mesh.type = brain_mesh.type = visualization_msgs::Marker::TRIANGLE_LIST;
  brain_mesh.action = visualization_msgs::Marker::ADD;
  empty_brain_mesh.pose.orientation.w = brain_mesh.pose.orientation.w = 1.0;
  empty_brain_mesh.scale.x = brain_mesh.scale.x = slicer_scale;
  empty_brain_mesh.scale.y = brain_mesh.scale.y = slicer_scale;
  empty_brain_mesh.scale.z = brain_mesh.scale.z = slicer_scale;
  geometry_msgs::Point point;
  ROS_INFO("Point coordinates: x=%f, y=%f, z=%f", point.x, point.y, point.z);
  geometry_msgs::Point origin_point;
  empty_brain_mesh.points.push_back(origin_point);
  empty_brain_mesh.points.push_back(origin_point);
  empty_brain_mesh.points.push_back(origin_point);
  ros::param::get("/brain_mesh/color/a", brain_mesh.color.a);
  ros::param::get("/brain_mesh/color/r", brain_mesh.color.r);
  ros::param::get("/brain_mesh/color/g", brain_mesh.color.g);
  ros::param::get("/brain_mesh/color/b", brain_mesh.color.b);

  // ************************************
  // Control the robot
  // ************************************

  // ************************************
  // User friendly welcome Text
  show_friendly_GUI_text(visual_tools, "Welcome to Needle Insert Simulation\nPress Next to continue");
  // Create subscriber to get data from 3D slicer
  ros::Subscriber igtlpoint_sub = node_handle.subscribe("/IGTL_POINT_IN", 5, pointCallback);
  ros::Subscriber igtlpolydata_sub = node_handle.subscribe("/IGTL_POLYDATA_IN", 1, polydataCallback);
  // Create publisher visulize brain mesh
  brain_mesh_pub = node_handle.advertise<visualization_msgs::Marker>("/brain_mesh", 1);
  show_friendly_GUI_text(visual_tools, "Please Send \"entry_point\", \"target_point\", \nand Brain model from 3D slicer");

  // Making sure that ROS received entry and target point
  for (; early_ending_counter>0; early_ending_counter--) {
    if (!received_entry_point && !received_target_point) {
      show_friendly_GUI_text(visual_tools, "Oops, I didn't reveived  \"entry_point\" \nand \"target_point\" from 3D slicer\nNumber of attempts: "
                             + std::to_string(early_ending_counter) + "\nPress Next to continue");
    } else if (!received_entry_point) {
      show_friendly_GUI_text(visual_tools, "Oops, I didn't reveived  \"entry_point\" from 3D slicer\nNumber of attempts: "
                             + std::to_string(early_ending_counter) + "\nPress Next to continue");
    } else if (!received_target_point) {
      show_friendly_GUI_text(visual_tools, "Oops, I didn't reveived  \"target_point\" from 3D slicer\nNumber of attempts: "
                             + std::to_string(early_ending_counter) + "\nPress Next to continue");
    }
  }
  if (!received_entry_point || !received_target_point) {
    show_friendly_GUI_text(visual_tools, "Shutting down\nPress next to shut down");
    clean_exit(move_group, visual_tools);
    return 0;
  }
  // Set the orientation base on start and end position of needle
  // after welcome text to wait for slicer to publish
  set_needle_orientation_for_start_and_end_pose(needle_orientation_in_end_effector_frame, initial_pose, target_pose);

  // ************************************
  // Move to initial pose
  brain_mesh_pub.publish(empty_brain_mesh); // remove brain model first
  // Destroy subscriber
  igtlpoint_sub.shutdown();
  igtlpolydata_sub.shutdown();
  // Set up to move to start point
  waypoints.resize(0);
  waypoints.push_back(move_group.getCurrentPose().pose);
  waypoints.push_back(initial_pose);
  success = move_through_list_of_points(waypoints, move_group, joint_model_group, visual_tools, brain_model_pose, 1.0, 0.01, starting_duration, "Move to Start Pose");
  if (!success) {
    show_friendly_GUI_text(visual_tools, "Shutting down\nPress next to shut down");
    clean_exit(move_group, visual_tools);
    return 0;}

  // ************************************
  // Add Brain Model to the environment
  if (received_brain_mesh) {
    // User friendly Text
    show_friendly_GUI_text(visual_tools, "Preparing to Add brain model\nPress Next to Add brain model");
    // Add brain model
    visual_tools.publishText(text_pose, "Brain model Added\nPress Next to continue", rvt::PURPLE, rvt::XXXLARGE, 3);
    // Visualize brain mesh
    brain_mesh_pub.publish(brain_mesh);
    ROS_INFO_NAMED("tutorial", "Added brain model into the workspace"); // Show text in RViz of status 
    visual_tools.trigger(); // Sending visualization
    visual_tools.prompt("Press 'next' to continue");
  }

  // ************************************
  // Set path constraints to make the needle point mantain desired orientation
  set_constraint_from_initial_orientation(initial_pose, move_group);

  // ************************************
  // Insert the needle
  waypoints.resize(0);
  waypoints.push_back(move_group.getCurrentPose().pose);
  waypoints.push_back(target_pose);
  success = move_through_list_of_points(waypoints, move_group, joint_model_group, visual_tools, brain_model_pose, 1.0, task_space_discretize_distance, insert_needle_duration, "Insert the Needle");
  if (!success) {
    show_friendly_GUI_text(visual_tools, "Shutting down\nPress next to shut down");
    clean_exit(move_group, visual_tools);
    return 0;}
  // ************************************
  // Remove the needle
  waypoints.resize(0);
  waypoints.push_back(move_group.getCurrentPose().pose);
  waypoints.push_back(initial_pose);
  success = move_through_list_of_points(waypoints, move_group, joint_model_group, visual_tools, brain_model_pose, 1.0, task_space_discretize_distance, remove_needle_duration, "Remove the Needle");
  if (!success) {
    show_friendly_GUI_text(visual_tools, "Shutting down\nPress next to shut down");
    clean_exit(move_group, visual_tools);
    return 0;}

  // ************************************
  // Ending the path planning
  show_friendly_GUI_text(visual_tools, "Finished!!!\nCongratulation!!!\nPress Next to End");
  // Clean exit
  clean_exit(move_group, visual_tools);
  return 0;
}

// ************************************************************************************************************
// ************************************************************************************************************
void set_needle_orientation_for_start_and_end_pose(tf2::Vector3& needle_orientation_in_end_effector_frame,
                                                   geometry_msgs::Pose& initial_pose,
                                                   geometry_msgs::Pose& target_pose)
  {
  // Get the direction of needle insertion
  tf2::Vector3 insertion_direction(target_pose.position.x - initial_pose.position.x,
                                   target_pose.position.y - initial_pose.position.y,
                                   target_pose.position.z - initial_pose.position.z);
  // Get the angle to rotate the end_effector
  double rotation_angle = needle_orientation_in_end_effector_frame.angle(insertion_direction);
  // Get the axix to rotate the end_effector
  tf2::Vector3 rotation_axis = needle_orientation_in_end_effector_frame.cross(insertion_direction);
  // Declare and compute the Quaternion for the orientation of the needle
  tf2::Quaternion needle_orientation;
  needle_orientation.setRotation(rotation_axis, rotation_angle);
  // Set orientation for start and end pose
  initial_pose.orientation.x = needle_orientation.x();
  initial_pose.orientation.y = needle_orientation.y();
  initial_pose.orientation.z = needle_orientation.z();
  initial_pose.orientation.w = needle_orientation.w();
  target_pose.orientation.x = needle_orientation.x();
  target_pose.orientation.y = needle_orientation.y();
  target_pose.orientation.z = needle_orientation.z();
  target_pose.orientation.w = needle_orientation.w();
}

void set_constraint_from_initial_orientation(geometry_msgs::Pose& initial_pose,
                                             moveit::planning_interface::MoveGroupInterface& move_group) {
  // Declare the constraint
  moveit_msgs::OrientationConstraint Needle_ocm;
  Needle_ocm.link_name = move_group.getEndEffectorLink();
  Needle_ocm.header.frame_id = move_group.getPlanningFrame();
  Needle_ocm.orientation = initial_pose.orientation;
  Needle_ocm.absolute_x_axis_tolerance = 0.01;
  Needle_ocm.absolute_y_axis_tolerance = 0.01;
  Needle_ocm.absolute_z_axis_tolerance = 0.01;
  Needle_ocm.weight = 1.0;
  // Set it as the path constraint for the group.
  moveit_msgs::Constraints Needle_constraints;
  Needle_constraints.orientation_constraints.push_back(Needle_ocm);
  move_group.setPathConstraints(Needle_constraints);
}

void show_friendly_GUI_text(moveit_visual_tools::MoveItVisualTools& visual_tools, std::string text) {
  // Show user friendly Text to control the UI
  visual_tools.publishText(text_pose, text, rvt::PURPLE, rvt::XXXLARGE, 3);
  visual_tools.trigger(); // Sending visualization
  visual_tools.prompt(text);
}

void set_execute_duration_for_trajectory(moveit_msgs::RobotTrajectory& input_trajectory, double execute_duration) {
  std::vector<trajectory_msgs::JointTrajectoryPoint>& joint_trajectories = input_trajectory.joint_trajectory.points;
  size_t size = joint_trajectories.size();
  execute_duration = execute_duration/size;
  for (int i=0; i<size; i++) {
    joint_trajectories[i].time_from_start = ros::Duration(execute_duration*i);
  }
}

bool move_through_list_of_points(std::vector<geometry_msgs::Pose>& waypoints, 
                                 moveit::planning_interface::MoveGroupInterface& move_group,
                                 const robot_state::JointModelGroup* joint_model_group,
                                 moveit_visual_tools::MoveItVisualTools& visual_tools,
                                 geometry_msgs::Pose& brain_model_pose,
                                 double velociy_scaling = 1.0,
                                 double eef_step = 0.001, // step to discretize in task space
                                 double execute_duration = 3.0,
                                 std::string action = "default action") // show_brain DELETE LATER
  {    
  move_group.setMaxVelocityScalingFactor(velociy_scaling); // change the movement speed
  show_friendly_GUI_text(visual_tools, "Preparing to " + action + "\nPress next to show the path planned");
  // Plan the trajectory;
  fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  set_execute_duration_for_trajectory(trajectory, execute_duration); // Slow down or speed up the robot
  my_plan.trajectory_ = trajectory;
  // Stop if planning fail
  if (fraction != 1.0) {
    visual_tools.publishText(text_pose, "Path planning failed!!! \nPress Next to End Planning", rvt::RED, rvt::XXXLARGE, 3);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' to end the planning"); // Interactive UI
    return false;
  }
  // Visualization
  visual_tools.publishText(text_pose, "Path to " + action + "\nPress Next to Move", rvt::PURPLE, rvt::XXXLARGE, 3);
  visual_tools.publishArrow(waypoints[0].position, waypoints.back().position, rvt::TRANSLUCENT_DARK, rvt::XXSMALL, 1); // Trajectory line
  // // ### These two line below show visualization of the trajectory calculated (It is suppose to be a straight line)
  // // and the position of the target as a sphere. Comment/Uncomment if you want to visualize them ###
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group, rvt::TRANSLUCENT_LIGHT); // Trajectory line
  // visual_tools.publishSphere(waypoints.back().position, rvt::TRANSLUCENT_DARK, rvt::XXSMALL, "Sphere", 2); // A sphere at target position
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' to accept the planning and " + action); // Interactive UI
  ROS_INFO_NAMED("tutorial", "%s", action.c_str()); // Interactive UI
  // Move the robot
  visual_tools.publishText(text_pose, "Executing: " + action, rvt::PURPLE, rvt::XXXLARGE, 3); // Friendly text
  visual_tools.trigger();
  move_group.execute(my_plan); // execute the plan
  show_friendly_GUI_text(visual_tools, action + ": Success!!!\nPress next to continue");
  ROS_INFO_NAMED("tutorial", "%s: Success!!!", action.c_str()); // Interactive UI
  return true;
}

void clean_exit(moveit::planning_interface::MoveGroupInterface& move_group,
                moveit_visual_tools::MoveItVisualTools& visual_tools) {
  // Clean exit
  visual_tools.deleteAllMarkers(); // Clear Marker
  move_group.clearPathConstraints(); // Clear Path constraint
  visual_tools.trigger();
  brain_mesh_pub.publish(empty_brain_mesh); // remove brain model
  ros::shutdown(); // Shut down ros
}

void pointCallback(const ros_igtl_bridge::igtlpoint::ConstPtr& msg) {
  // Check the name of the point first (entry_point_name and target_point_name could be set in user parameter) and store their value
  if (msg->name == entry_point_name) {
    initial_pose.position.x = msg->pointdata.x*slicer_scale;
    initial_pose.position.y = msg->pointdata.y*slicer_scale;
    initial_pose.position.z = msg->pointdata.z*slicer_scale;
    ROS_INFO_NAMED("tutorial", "I got point \"%s\" from 3D slicer", msg->name.c_str());
    received_entry_point = true; // Turn to true after received entry pose
  } else if (msg->name == target_point_name) {
    target_pose.position.x = msg->pointdata.x*slicer_scale;
    target_pose.position.y = msg->pointdata.y*slicer_scale;
    target_pose.position.z = msg->pointdata.z*slicer_scale;
    ROS_INFO_NAMED("tutorial", "I got point \"%s\" from 3D slicer", msg->name.c_str());
    received_target_point = true; // Turn to true after received target pose
  }
  // Signal the main code that ROS received both points
  if (received_entry_point && received_target_point) {
    early_ending_counter = -1;
    visual_tools_Ptr->publishText(text_pose, "I have received both entry_point and target_point\nPress Next to continue", rvt::PURPLE, rvt::XXXLARGE, 3);
    visual_tools_Ptr->publishArrow(initial_pose.position, target_pose.position, rvt::PURPLE, rvt::XXSMALL, 1); // Visualize the path
    visual_tools_Ptr->trigger();
  }
  return;
}

geometry_msgs::Point convertSlicerPoint32ToROSPoint64(const geometry_msgs::Point32& point32) {
    // Convert geometry_msgs::Point32 To geometry_msgs::Point
    geometry_msgs::Point point;
    point.x = -point32.x;
    point.y = -point32.y;
    point.z = point32.z;
    return point;
}

void convert_igtlpolydata_to_brain_mesh_Marker(const ros_igtl_bridge::igtlpolydata& data) {
  int numTriangles = data.polygons.size();
  geometry_msgs::Point vertice;
  geometry_msgs::Point32 triangle;
  brain_mesh.points.resize(0); // Delete mesh data of previous sending. Uncomment this line if you want to visualize multiple mesh at once
  for (int i = 0; i < numTriangles; i++) {
    // Triangle store the index of the point (in int)
    triangle = data.polygons[i];
    // Get the point from the index
    vertice = convertSlicerPoint32ToROSPoint64(data.points[static_cast<uint32_t>(triangle.x)]);
    brain_mesh.points.push_back(vertice);
    vertice = convertSlicerPoint32ToROSPoint64(data.points[static_cast<uint32_t>(triangle.y)]);
    brain_mesh.points.push_back(vertice);
    vertice = convertSlicerPoint32ToROSPoint64(data.points[static_cast<uint32_t>(triangle.z)]);
    brain_mesh.points.push_back(vertice);
  }    
}

void polydataCallback(const ros_igtl_bridge::igtlpolydata::ConstPtr& msg) {
  // Ros only handles polydata in msg->points and msg->polygons
  // Raise warning if the polydata received have other geometries
  if (msg->strips.size() != 0) {
  ROS_WARN_NAMED("Polydata mismatch", "The polydata is expected to have only points and polygons data. \"strips\" or triangles data are sent from 3D slicer");
  }
  if (msg->lines.size() != 0) {
  ROS_WARN_NAMED("Polydata mismatch", "The polydata is expected to have only points and polygons data. \"lines\" or lines data are sent from 3D slicer");
  }
  if (msg->verts.size() != 0) {
  ROS_WARN_NAMED("Polydata mismatch", "The polydata is expected to have only points and polygons data. \"verts\" or vertices data are sent from 3D slicer");
  }
  // Information about the mesh received
  ROS_INFO_NAMED("tutorial", "I got polydata message with %lu points, %lu polygons, %lu triangles, %lu lines and %lu vertices",
                  msg->points.size(),
                  msg->polygons.size(),
                  msg->strips.size(),
                  msg->lines.size(),
                  msg->verts.size());
  
  if (msg->points.size() > 0) {
    received_brain_mesh = true; // Turn to true after received brain model
    // Set up brain mesh model
    brain_mesh.header.stamp = ros::Time::now();
    convert_igtlpolydata_to_brain_mesh_Marker(*msg);
    // Visualize brain mesh
    brain_mesh_pub.publish(brain_mesh);
  }
  return;
}
// ************************************************************************************************************
// ************************************************************************************************************
