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
    // Wrap raw this pointer in a no-op deleter shared_ptr to satisfy MoveGroupInterface
    move_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "grap_arm"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    control_period_ = 20ms;  // 50 Hz
    timer_ = this->create_wall_timer(control_period_, std::bind(&ArucoFollower::controlLoop, this));
    RCLCPP_INFO(this->get_logger(), "Aruco follower (orientation only) started at 50 Hz");
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
      tf2::Vector3 src(1,0,0);
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

      // Update orientation only
      geometry_msgs::msg::PoseStamped current = move_group_.getCurrentPose();
      geometry_msgs::msg::Pose target = current.pose;
      target.orientation.x = q.x();
      target.orientation.y = q.y();
      target.orientation.z = q.z();
      target.orientation.w = q.w();

      // Plan and execute
      move_group_.setPoseTarget(target);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_.execute(plan);
      } else {
        RCLCPP_WARN(this->get_logger(), "Orientation plan failed");
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
