#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "cx7_interface/srv/move_to_pose.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

class MoveToPoseService : public rclcpp::Node
{
public:
    MoveToPoseService(rclcpp::NodeOptions options) : Node("move_to_pose_service", options.automatically_declare_parameters_from_overrides(true))
    {
        // 状態監視のためのexecutorの設定
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(this->get_node_base_interface());
        std::thread([this]() { this->executor_->spin(); }).detach();

        // サービスサーバーの作成
        service_ = this->create_service<cx7_interface::srv::MoveToPose>(
            "move_to_pose",
            std::bind(&MoveToPoseService::handle_move_to_pose, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_move_to_pose(const std::shared_ptr<cx7_interface::srv::MoveToPose::Request> request,
                             std::shared_ptr<cx7_interface::srv::MoveToPose::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Service request received: planning_group: %s, goal_state: %s, max_vel: %f, max_accel: %f",
                    request->planning_group.c_str(), request->goal_state.c_str(), request->max_vel, request->max_accel);

        // リクエストからMoveGroupInterfaceを初期化
        MoveGroupInterface arm(this->shared_from_this(), request->planning_group);

        // リクエストからパラメータを設定
        arm.setMaxVelocityScalingFactor(request->max_vel);
        arm.setMaxAccelerationScalingFactor(request->max_accel);
        arm.setNamedTarget(request->goal_state.c_str());

        // アームを動かす
        bool success = (arm.move() == moveit::core::MoveItErrorCode::SUCCESS);

        response->success = success;
        RCLCPP_INFO(this->get_logger(), "Sending response: %s", success ? "Success" : "Failure");
    }

    rclcpp::Service<cx7_interface::srv::MoveToPose>::SharedPtr service_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<MoveToPoseService>(node_options);

    //rclcpp::spin(node);
    // メインスレッドを適切にブロックするための無限ループ
    rclcpp::Rate rate(1);
    while (rclcpp::ok()) {
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
