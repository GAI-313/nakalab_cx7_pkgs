// ROS
#include <rclcpp/rclcpp.hpp>

// Servo
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("servo_cmd_vel_node.cpp");

rclcpp::Node::SharedPtr node_;
rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
std::string frame_id_;

void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // TwistStampedメッセージを作成してパブリッシュ
  auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  twist_msg->header.stamp = node_->now();
  twist_msg->header.frame_id = frame_id_; // このフレームIDもパラメータから受け取ることができるようにした方がよいかもしれません
  twist_msg->twist = *msg;
  twist_cmd_pub_->publish(std::move(twist_msg));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;

  // パラメータの自動宣言を有効化
  node_options.use_intra_process_comms(false).automatically_declare_parameters_from_overrides(true);
  node_ = std::make_shared<rclcpp::Node>("servo_demo_node", node_options);

  // ノード初期化後の一時停止
  rclcpp::sleep_for(std::chrono::seconds(4));

  // パラメータ 'robot_description' の取得
  std::string robot_description;
  if (!node_->get_parameter("robot_description", robot_description))
  {
    RCLCPP_ERROR(LOGGER, "'robot_description' parameter not set");
    return EXIT_FAILURE;
  }

  // パラメータ 'frame_id' の取得
  if (!node_->get_parameter("frame_id", frame_id_))
  {
    RCLCPP_WARN(LOGGER, "'frame_id' parameter not set, using default 'panda_link0'");
    frame_id_ = "crane_x7_gripper_base_link";
  }

  // Planning Scene Monitorの設定
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, robot_description, tf_buffer, "planning_scene_monitor");

  if (planning_scene_monitor->getPlanningScene())
  {
    planning_scene_monitor->startStateMonitor("/joint_states");
    planning_scene_monitor->setPlanningScenePublishingFrequency(25);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                         "/moveit_servo/publish_planning_scene");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->providePlanningSceneService();
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning scene not configured");
    return EXIT_FAILURE;
  }

  // パブリッシャの作成
  twist_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>("arm/servo/cmd_vel", 10);

  // サブスクライバの作成
  auto cmd_vel_sub = node_->create_subscription<geometry_msgs::msg::Twist>(
      "arm/controll/cmd_vel", 10, cmdVelCallback);

  // MoveIt Servoの初期化
  auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node_);
  if (!servo_parameters)
  {
    RCLCPP_FATAL(LOGGER, "Failed to load the servo parameters");
    return EXIT_FAILURE;
  }
  auto servo = std::make_unique<moveit_servo::Servo>(node_, servo_parameters, planning_scene_monitor);
  servo->start();

  // マルチスレッドエグゼキュータの使用
  auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node_);
  executor->spin();

  // シャットダウン
  rclcpp::shutdown();
  return 0;
}
