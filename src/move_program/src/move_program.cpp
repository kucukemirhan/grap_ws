#include <memory>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

class ArucoFollower : public rclcpp::Node
{
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;

  ArucoFollower()
  : Node("aruco_follower"),
    move_group_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}), "arm_3dof"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    following_enabled_(true)
  {
    // Parameter: target TF frame
    this->declare_parameter<std::string>("target_frame", "aruco_marker_frame");
    this->get_parameter("target_frame", target_frame_);

    // Subscriber to change commands
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
          RCLCPP_INFO(this->get_logger(), "Tracking frame set to '%s'", target_frame_.c_str());
        }
      });

    // MoveGroup settings
    move_group_.setMaxVelocityScalingFactor(0.3);
    move_group_.setMaxAccelerationScalingFactor(0.3);
    move_group_.setPlanningTime(0.1);

    // Action client
    traj_client_ = rclcpp_action::create_client<FollowJT>(
      this, "/arm_3dof_controller/follow_joint_trajectory");

    // 50 Hz timer
    timer_ = this->create_wall_timer(20ms,
      std::bind(&ArucoFollower::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "ArucoFollower started, listening to frame '%s'", target_frame_.c_str());
  }

private:
  moveit::planning_interface::MoveGroupInterface move_group_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr target_frame_sub_;
  std::string target_frame_;
  bool following_enabled_;

  // Action client
  rclcpp_action::Client<FollowJT>::SharedPtr traj_client_;
  rclcpp_action::Client<FollowJT>::GoalHandle::SharedPtr active_goal_handle_;

  void controlLoop()
  {
    if (!following_enabled_) return;
    try {
      // Lookup TF for marker and end-effector
      auto tf_marker = tf_buffer_.lookupTransform(
        "base_link", target_frame_, tf2::TimePointZero);
      auto tf_ee = tf_buffer_.lookupTransform(
        "base_link", move_group_.getEndEffectorLink(), tf2::TimePointZero);

      // Positions
      tf2::Vector3 ee(tf_ee.transform.translation.x,
                      tf_ee.transform.translation.y,
                      tf_ee.transform.translation.z);
      tf2::Vector3 mk(tf_marker.transform.translation.x,
                      tf_marker.transform.translation.y,
                      tf_marker.transform.translation.z);

      // Direction
      tf2::Vector3 dir = mk - ee;
      const double min_dist = 1e-6;
      if (dir.length() < min_dist) {
        move_group_.stop();
        return;
      }
      dir.normalize();

      // Compute quaternion from (0,1,0) to dir
      tf2::Vector3 src(0,1,0);
      tf2::Vector3 axis = src.cross(dir);
      double dot = src.dot(dir);
      tf2::Quaternion q;
      if (axis.length() < 1e-6) {
        q.setValue(0,0,0, dot > 0 ? 1 : 0);
      } else {
        axis.normalize();
        double ang = std::acos(std::clamp(dot, -1.0, 1.0));
        q.setRotation(axis, ang);
      }
      q.normalize();

      // To RPY
      tf2::Matrix3x3 m(q);
      double r,p,y;
      m.getRPY(r,p,y);

      // Joint goal: yaw, pitch, roll
      std::vector<double> goal = {-y, p, r};
      
      // Only re-plan if goal changed significantly
      static std::array<double,3> last_goal = {{0.0,0.0,0.0}};
      const double EPS = 0.01;
      bool significant = std::fabs(goal[0]-last_goal[0])>EPS ||
                         std::fabs(goal[1]-last_goal[1])>EPS ||
                         std::fabs(goal[2]-last_goal[2])>EPS;
      if (!significant) {
        return;
      }
      last_goal = {{goal[0], goal[1], goal[2]}};

      move_group_.setJointValueTarget(goal);

      // Plan
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      if (move_group_.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Plan failed");
        return;
      }

      // Execute
      sendTrajectory(plan.trajectory_.joint_trajectory);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "TF error: %s", ex.what());
      move_group_.stop();
    }
  }

  void sendTrajectory(
    const trajectory_msgs::msg::JointTrajectory &input)
  {
    // If a goal is active (accepted or executing), skip new trajectory
    if (active_goal_handle_) {
      auto status = active_goal_handle_->get_status();
      if (status == rclcpp_action::GoalStatus::STATUS_ACCEPTED ||
          status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Active goal status=%d, skipping new trajectory", status);
        return;
      }
    }

    // Prepare trajectory for immediate execution
    auto traj = input;
    traj.header.stamp.sec = 0;
    traj.header.stamp.nanosec = 0;

    // Wait for the action server
    if (!traj_client_->wait_for_action_server(1s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server unavailable");
      return;
    }

    // Build and send new goal
    FollowJT::Goal goal_msg;
    goal_msg.trajectory = traj;

    rclcpp_action::Client<FollowJT>::SendGoalOptions opts;
    opts.goal_response_callback = [this](
      rclcpp_action::ClientGoalHandle<FollowJT>::SharedPtr gh)
    {
      if (!gh) {
        RCLCPP_ERROR(this->get_logger(), "Goal rejected");
      } else {
        active_goal_handle_ = gh;
        RCLCPP_INFO(this->get_logger(), "Goal accepted");
      }
    };
    opts.feedback_callback = [](auto, auto) {};
    opts.result_callback = [this](auto result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(this->get_logger(), "Execution succeeded");
      } else {
        RCLCPP_ERROR(
          this->get_logger(), "Execution failed: code %d",
          static_cast<int>(result.code));
      }
      // Clear active handle so next goal can be sent
      active_goal_handle_.reset();
    };

    traj_client_->async_send_goal(goal_msg, opts);
  }

  void goHome()
  {
    std::vector<double> home = {0.0, 0.0, 0.0};
    move_group_.setJointValueTarget(home);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      sendTrajectory(plan.trajectory_.joint_trajectory);
    } else {
      RCLCPP_WARN(this->get_logger(), "Home plan failed");
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoFollower>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
