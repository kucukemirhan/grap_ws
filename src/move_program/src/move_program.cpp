#include <memory>
#include <rclcpp/rclcpp.hpp> //ros client library
#include <moveit/move_group_interface/move_group_interface.h> // plan and execute motion
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> //last 2 for converting euler angles to quaternion
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); //initialize the ROS client library

  auto const node = std::make_shared<rclcpp::Node>(
    "move_program",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
      .use_intra_process_comms(true)
  );

  auto const logger = rclcpp::get_logger("move_program");

  // Create a MoveGroupInterface object for the "arm" group
  moveit::planning_interface::MoveGroupInterface MoveGroup(node, "grap_arm"); 
  
  // define goal pose
  
  tf2::Quaternion tf2_quaternion; // define orientation
  tf2_quaternion.setRPY(0.1, 0, -3.14/2); // set roll, pitch, yaw

  geometry_msgs::msg::Quaternion msg_quaternion = tf2::toMsg(tf2_quaternion); // convert to message type

  // set a goal pose
  geometry_msgs::msg::Pose goal_pose;

  goal_pose.orientation = msg_quaternion; // set orientation
  goal_pose.position.x = 0.4; // set x position
  goal_pose.position.y = 0.2; // set y position
  goal_pose.position.z = 0.4; // set z position
  
  MoveGroup.setPoseTarget(goal_pose); // set the target pose

  // plan the motion
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto const success = static_cast<bool>(MoveGroup.plan(plan)); // plan the motion
  
  // execute the plan
  if (success)
  {
    RCLCPP_INFO(logger, "Planning successful");
    MoveGroup.execute(plan); // execute the plan
  }
  else
  {
    RCLCPP_ERROR(logger, "Not able to plan and execute the motion");\
    return 1;
  }

  // shutdown ros
  rclcpp::shutdown();
  return 0;
}