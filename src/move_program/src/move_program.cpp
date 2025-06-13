#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
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
    tf_listener_(tf_buffer_),
    following_enabled_(true)
  {
    // Declare and get initial target frame parameter
    this->declare_parameter<std::string>("target_frame", "aruco_marker_frame");
    this->get_parameter("target_frame", target_frame_);

    // Subscription to update target frame dynamically or handle commands
    target_frame_sub_ = this->create_subscription<std_msgs::msg::String>(
      "target_frame", 10,
      [this](const std_msgs::msg::String::SharedPtr msg){
        const std::string &cmd = msg->data;
        if (cmd == "stop") {
          following_enabled_ = false;
          RCLCPP_INFO(this->get_logger(), "Following stopped.");
        } else if (cmd == "home") {
          following_enabled_ = false;
          RCLCPP_INFO(this->get_logger(), "Going to home position...");
          goHome();
        } else {
          target_frame_ = cmd;
          following_enabled_ = true;
          RCLCPP_INFO(this->get_logger(), "Updated target_frame to: %s", target_frame_.c_str());
        }
      });

    control_period_ = 20ms;  // 50 Hz
    timer_ = this->create_wall_timer(control_period_, std::bind(&ArucoFollower::controlLoop, this));
    
    // Configure MoveIt for Cartesian planning
    move_group_.setMaxVelocityScalingFactor(0.3);
    move_group_.setMaxAccelerationScalingFactor(0.3);
    move_group_.setPlanningTime(0.1);
    
    RCLCPP_INFO(this->get_logger(), "Aruco follower (Cartesian orientation) started at 50 Hz, listening to frame '%s'", target_frame_.c_str());
  }

private:
  moveit::planning_interface::MoveGroupInterface move_group_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::milliseconds control_period_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_frame_sub_;
  std::string target_frame_;
  bool following_enabled_;

  void controlLoop()
  {
    if (!following_enabled_) return;
    try {
      // Lookup transforms in base_link frame
      auto tf_marker = tf_buffer_.lookupTransform("base_link", target_frame_, tf2::TimePointZero);
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

      // Compute quaternion to rotate camera_link's y-axis into dir
      tf2::Vector3 src(0,1,0);
      double dot = src.dot(dir);
      tf2::Vector3 axis = src.cross(dir);
      double axis_len = axis.length();
      tf2::Quaternion q;
      if (axis_len < 1e-6) {
        q.setValue(0,0,0,dot>0?1:0);
      } else {
        axis.normalize();
        double angle = std::acos(std::clamp(dot, -1.0, 1.0));
        q.setRotation(axis, angle);
      }
      q.normalize();

      // Current pose and target pose
      geometry_msgs::msg::PoseStamped current = move_group_.getCurrentPose();
      geometry_msgs::msg::Pose target = current.pose;
      // double alpha = 0.3;
      // target.position.x = ee_pos.x() + alpha * (mk_pos.x() - ee_pos.x());
      // target.position.y = ee_pos.y() + alpha * (mk_pos.y() - ee_pos.y());
      // target.position.z = ee_pos.z() + alpha * (mk_pos.z() - ee_pos.z());
      target.orientation.x = q.x();
      target.orientation.y = q.y();
      target.orientation.z = q.z();
      target.orientation.w = q.w();

      // Plan Cartesian path
      std::vector<geometry_msgs::msg::Pose> waypoints = { target };
      moveit_msgs::msg::RobotTrajectory traj;
      double fraction = move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj);
      if (fraction > 0.8) {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = traj;
        move_group_.execute(plan);
      } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Cartesian path planning failed (%.1f%% valid)", fraction*100.0);
        move_group_.setPoseTarget(target);
        moveit::planning_interface::MoveGroupInterface::Plan fallback;
        if (move_group_.plan(fallback) == moveit::core::MoveItErrorCode::SUCCESS) {
          move_group_.execute(fallback);
        }
      }
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "TF lookup failed: %s", ex.what());
      move_group_.stop();
    }
  }

  void goHome()
  {
    static const std::vector<double> home_joints(6, 0.0);
    move_group_.setJointValueTarget(home_joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_.execute(plan);
      RCLCPP_INFO(this->get_logger(), "Reached home position.");
    } else {
      RCLCPP_WARN(this->get_logger(), "Home plan failed.");
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
