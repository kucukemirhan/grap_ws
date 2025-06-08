#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

using namespace std::chrono_literals;

class TeleopCartesian
{
public:
  TeleopCartesian(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    move_group_(node_, "grap_arm"),
    home_button_idx_(node_->declare_parameter<int>("home_button", 0))
  {
    bool simulated = false;
    node_->get_parameter("use_sim_time", simulated);

    if (simulated) {
      RCLCPP_INFO(node_->get_logger(), "*** SIMULATION MODE ***");
      // Wait for /clock to start
      while (rclcpp::ok() && node_->now().nanoseconds() == 0) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(10ms);
      }
    } else {
      RCLCPP_INFO(node_->get_logger(), "*** REAL MODE ***");
    }

    // Initialize last_twist_time_ from same clock source
    last_twist_time_ = node_->now();

    RCLCPP_INFO(node_->get_logger(),
                "End effector link: %s",
                move_group_.getEndEffectorLink().c_str());

    waitForRobotState();

    // Subscribe to /joy
    joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&TeleopCartesian::joyCallback, this, std::placeholders::_1));

    // Subscribe to /cmd_vel
    cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&TeleopCartesian::cmdVelCallback, this, std::placeholders::_1));

    // 50 Hz control loop
    timer_ = node_->create_wall_timer(
      20ms,
      std::bind(&TeleopCartesian::controlLoop, this));

    RCLCPP_INFO(node_->get_logger(),
                "Teleop Cartesian ready — running at 50 Hz.");
  }

private:
  // Node + MoveIt interface
  rclcpp::Node::SharedPtr                        node_;
  moveit::planning_interface::MoveGroupInterface move_group_;

  // Subscriptions & timer
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr   joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::TimerBase::SharedPtr                             timer_;

  // Home-button index & last Joy message
  int                                               home_button_idx_;
  sensor_msgs::msg::Joy::SharedPtr                  last_joy_msg_;

  // Shared state
  geometry_msgs::msg::Pose  current_pose_;
  geometry_msgs::msg::Pose  target_pose_;
  geometry_msgs::msg::Twist last_twist_;
  rclcpp::Time              last_twist_time_;
  std::mutex                mutex_;

  void waitForRobotState()
  {
    rclcpp::Rate rate(10);
    while (rclcpp::ok() && !move_group_.getCurrentState()) {
      rclcpp::spin_some(node_);
      rate.sleep();
    }
    current_pose_ = move_group_.getCurrentPose().pose;
    target_pose_  = current_pose_;
    RCLCPP_INFO(node_->get_logger(), "Initial robot state received.");
  }

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    int curr = 0, prev = 0;
    if (home_button_idx_ < static_cast<int>(msg->buttons.size())) {
      curr = msg->buttons[home_button_idx_];
    }
    if (last_joy_msg_ && home_button_idx_ < static_cast<int>(last_joy_msg_->buttons.size())) {
      prev = last_joy_msg_->buttons[home_button_idx_];
    }

    // Rising edge → go home
    if (curr == 1 && prev == 0) {
      RCLCPP_INFO(node_->get_logger(), "Home button pressed — moving to home pose.");
      goToHome();
    }

    last_joy_msg_ = msg;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_twist_      = *msg;
    last_twist_time_ = node_->now();
  }

  void controlLoop()
  {
    geometry_msgs::msg::Twist twist;
    rclcpp::Time            stamp;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      twist = last_twist_;
      stamp = last_twist_time_;
    }

    // If idle >0.1 s or zero twist → stop
    if ((node_->now() - stamp).seconds() > 0.1 ||
        (twist.linear.x == 0.0 && twist.linear.y == 0.0 && twist.linear.z == 0.0))
    {
      move_group_.stop();
      std::lock_guard<std::mutex> lock(mutex_);
      target_pose_ = current_pose_;
      return;
    }

    // Integrate small Cartesian step
    constexpr double loop_dt = 0.02;  // 50 Hz
    constexpr double vel_sf  = 0.05;  // m/unit/sec
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target_pose_.position.x += twist.linear.x * vel_sf * loop_dt;
      target_pose_.position.y += twist.linear.y * vel_sf * loop_dt;
      target_pose_.position.z += twist.linear.z * vel_sf * loop_dt;
    }

    executeCartesianStep();
  }

  void executeCartesianStep()
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      waypoints = { current_pose_, target_pose_ };
    }

    moveit_msgs::msg::RobotTrajectory traj;
    double fraction = move_group_.computeCartesianPath(
      waypoints, /*eef_step*/ 0.005, /*jump_threshold*/ 0.0, traj);

    if (fraction > 0.9) {
      move_group_.execute(traj);
      std::lock_guard<std::mutex> lock(mutex_);
      current_pose_ = target_pose_;
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Cartesian plan failed: %.0f%%", fraction * 100.0);
    }
  }

  void goToHome()
  {
    // Define your robot’s home joint values here:
    std::vector<double> home_positions = {
      0.0,  // Shoulder Pan
      0.0,  // Shoulder Lift
      0.0,  // Elbow
      0.0,  // Wrist 1
      0.0,  // Wrist 2
      0.0   // Wrist 3
    };

    move_group_.setJointValueTarget(home_positions);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group_.execute(plan);
      RCLCPP_INFO(node_->get_logger(), "Reached home pose.");
      std::lock_guard<std::mutex> lock(mutex_);
      current_pose_ = move_group_.getCurrentPose().pose;
      target_pose_  = current_pose_;
    } else {
      RCLCPP_WARN(node_->get_logger(), "Home plan failed.");
    }
  }
};

int main(int argc, char *argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // NodeOptions can override use_sim_time via launch/CLI
  rclcpp::NodeOptions opts;
  auto node = rclcpp::Node::make_shared("teleop_cartesian", opts);

  // Instantiate and spin
  auto teleop = std::make_shared<TeleopCartesian>(node);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
