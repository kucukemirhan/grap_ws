#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class ArucoFollower : public rclcpp::Node
{
public:
  ArucoFollower()
  : Node("aruco_follower"),
    move_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "grap_arm"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    control_period_ = 20ms;  // 50 Hz
    timer_ = this->create_wall_timer(control_period_, std::bind(&ArucoFollower::controlLoop, this));
    
    // Configure MoveIt for Cartesian planning
    move_group_.setMaxVelocityScalingFactor(0.3);  // Reduce speed for smoother motion
    move_group_.setMaxAccelerationScalingFactor(0.3);
    move_group_.setPlanningTime(0.1);  // Quick planning for real-time control
    
    RCLCPP_INFO(this->get_logger(), "Aruco follower (Cartesian orientation) started at 50 Hz");
  }

private:
  moveit::planning_interface::MoveGroupInterface move_group_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds control_period_;

  void controlLoop()
  {
    try {
      // Lookup transforms in base_link frame
      auto tf_marker = tf_buffer_.lookupTransform("base_link", "aruco_marker_frame", tf2::TimePointZero);
      auto tf_ee     = tf_buffer_.lookupTransform("base_link", move_group_.getEndEffectorLink(), tf2::TimePointZero);

      // Compute direction vector
      tf2::Vector3 ee_pos(tf_ee.transform.translation.x,
                          tf_ee.transform.translation.y,
                          tf_ee.transform.translation.z);
      tf2::Vector3 mk_pos(tf_marker.transform.translation.x,
                          tf_marker.transform.translation.y,
                          tf_marker.transform.translation.z);
      tf2::Vector3 dir = mk_pos - ee_pos;
      double len = dir.length();
      if (len < 1e-6) {
        move_group_.stop();
        return;
      }
      dir.normalize();

      // Compute quaternion to rotate camera_link's x-axis (1,0,0) into dir
      tf2::Vector3 src(0,1,0);
      double dot = src.dot(dir);
      tf2::Vector3 axis = src.cross(dir);
      double axis_len = axis.length();
      tf2::Quaternion q;
      if (axis_len < 1e-6) {
        if (dot > 0) {
          q.setValue(0,0,0,1);
        } else {
          q.setRotation(tf2::Vector3(0,0,1), M_PI);
        }
      } else {
        axis.normalize();
        double angle = std::acos(std::clamp(dot, -1.0, 1.0));
        q.setRotation(axis, angle);
      }
      q.normalize();

      // Get current pose and create target with new orientation
      geometry_msgs::msg::PoseStamped current = move_group_.getCurrentPose();
      geometry_msgs::msg::Pose target = current.pose;
      
      // Keep current position, only change orientation
      double alpha = 0.3; // 0 = gripper, 1 = marker, 0.5 = midpoint
      target.position.x = ee_pos.x() + alpha * (mk_pos.x() - ee_pos.x());
      target.position.y = ee_pos.y() + alpha * (mk_pos.y() - ee_pos.y());
      target.position.z = ee_pos.z() + alpha * (mk_pos.z() - ee_pos.z());
      target.orientation.x = q.x();
      target.orientation.y = q.y();
      target.orientation.z = q.z();
      target.orientation.w = q.w();

      // Create waypoints for Cartesian path (just current and target)
      std::vector<geometry_msgs::msg::Pose> waypoints;
      waypoints.push_back(target);

      // Plan Cartesian path
      moveit_msgs::msg::RobotTrajectory trajectory;
      const double jump_threshold = 0.0;  // Disable jump threshold
      const double eef_step = 0.01;       // 1cm resolution for orientation interpolation
      
      double fraction = move_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      
      if (fraction > 0.8) {  // If at least 80% of path is valid
        // Execute the Cartesian trajectory
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group_.execute(plan);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "Cartesian path planning failed (%.2f%% valid)", fraction * 100.0);
        
        // Fallback to regular planning
        move_group_.setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan fallback_plan;
        if (move_group_.plan(fallback_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
          move_group_.execute(fallback_plan);
        }
      }
    }
    catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF lookup failed: %s", ex.what());
      move_group_.stop();
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}