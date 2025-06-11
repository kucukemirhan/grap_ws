#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>

using namespace std::chrono_literals;

class TeleopCartesian
{
public:
  TeleopCartesian(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    move_group_(node_, "grap_arm"),
    tf_buffer_(node_->get_clock()),
    tf_listener_(tf_buffer_),
    home_button_idx_(node_->declare_parameter<int>("home_button", 0)),
    control_period_(100ms),
    loop_dt_(0.1)
  {
    bool simulated = node_->get_parameter("use_sim_time", simulated);
    if (simulated) {
      RCLCPP_INFO(node_->get_logger(), "*** SIMULATION MODE ***");
      while (rclcpp::ok() && node_->now().nanoseconds() == 0) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(10ms);
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "*** REAL MODE ***");
    }

    last_twist_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(), "End effector link: %s",
                move_group_.getEndEffectorLink().c_str());

    waitForInitialPose();

    joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&TeleopCartesian::handleJoy, this, std::placeholders::_1));

    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&TeleopCartesian::handleTwist, this, std::placeholders::_1));

    timer_ = node_->create_wall_timer(control_period_,
      std::bind(&TeleopCartesian::controlLoop, this));

    RCLCPP_INFO(node_->get_logger(), "Teleop Cartesian running at %.0f Hz.",
                1.0 / loop_dt_);
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int home_button_idx_;
  geometry_msgs::msg::Pose current_pose_, target_pose_;
  geometry_msgs::msg::Twist last_twist_;
  rclcpp::Time last_twist_time_;
  std::mutex mutex_;

  const std::chrono::milliseconds control_period_;
  const double loop_dt_;

  void waitForInitialPose()
  {
    RCLCPP_INFO(node_->get_logger(), "Waiting for initial TF pose...");
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      if (lookupPose(current_pose_)) {
        target_pose_ = current_pose_;
        RCLCPP_INFO(node_->get_logger(), "Initial TF pose acquired.");
        break;
      }
      rclcpp::spin_some(node_);
      rate.sleep();
    }
  }

  bool lookupPose(geometry_msgs::msg::Pose &out)
  {
    try {
      auto tf_stamped = tf_buffer_.lookupTransform(
        "base_link", move_group_.getEndEffectorLink(), tf2::TimePointZero);
      // Assign Vector3 to Point explicitly
      out.position.x = tf_stamped.transform.translation.x;
      out.position.y = tf_stamped.transform.translation.y;
      out.position.z = tf_stamped.transform.translation.z;
      out.orientation = tf_stamped.transform.rotation;
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(),
                           5000, "TF lookup failed: %s", ex.what());
      return false;
    }
  }

  void handleJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    static int prev = 0;
    int curr = (home_button_idx_ < (int)msg->buttons.size())
               ? msg->buttons[home_button_idx_] : 0;
    if (curr && !prev) {
      RCLCPP_INFO(node_->get_logger(), "Home button pressed.");
      goHome();
    }
    prev = curr;
  }

  void handleTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_twist_      = *msg;
    last_twist_time_ = node_->now();
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist twist;
    rclcpp::Time tstamp;
    { std::lock_guard<std::mutex> lock(mutex_);
      twist = last_twist_; tstamp = last_twist_time_; }

    if ((node_->now() - tstamp).seconds() > loop_dt_ ||
        (twist.linear.x==0 && twist.linear.y==0 && twist.linear.z==0))
    {
      move_group_.stop();
      std::lock_guard<std::mutex> lock(mutex_);
      lookupPose(current_pose_);
      target_pose_ = current_pose_;
      return;
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      target_pose_.position.x += twist.linear.x * loop_dt_ * 0.05;
      target_pose_.position.y += twist.linear.y * loop_dt_ * 0.05;
      target_pose_.position.z += twist.linear.z * loop_dt_ * 0.05;
    }
    executeStep();
  }

  void executeStep()
  {
    std::vector<geometry_msgs::msg::Pose> wps;
    { std::lock_guard<std::mutex> lock(mutex_);
      wps = {current_pose_, target_pose_}; }

    moveit_msgs::msg::RobotTrajectory traj;
    double frac = move_group_.computeCartesianPath(wps, 0.005, 0.0, traj);

    if (frac > 0.9) {
      move_group_.execute(traj);
      lookupPose(current_pose_);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Cartesian plan failed: %.0f%%", frac*100);
    }
  }

  void goHome()
  {
    static const std::vector<double> home_joints(6, 0.0);
    move_group_.setJointValueTarget(home_joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_.plan(plan)==moveit::core::MoveItErrorCode::SUCCESS) {
      move_group_.execute(plan);
      lookupPose(current_pose_);
      target_pose_ = current_pose_;
      RCLCPP_INFO(node_->get_logger(), "Reached home.");
    } else {
      RCLCPP_WARN(node_->get_logger(), "Home plan failed.");
    }
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("teleop_cartesian");
  auto teleop = std::make_shared<TeleopCartesian>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
