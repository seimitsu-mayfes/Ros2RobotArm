#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "my_robot_arm_moveit_config/srv/move_to_pose.hpp"

class MotionPlannerNode : public rclcpp::Node
{
public:
    MotionPlannerNode() : Node("motion_planner_node")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Motion Planner Node");

        // MoveItの初期化を別スレッドで実行
        move_group_thread_ = std::thread([this]() {
            // ノードのexecutorを作成
            auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            executor->add_node(this->get_node_base_interface());

            // MoveGroupInterfaceの初期化
            move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                this->shared_from_this(), "arm_group");

            // 基本設定
            // プランニング設定
            move_group_interface_->setPlanningTime(30.0);
            move_group_interface_->setNumPlanningAttempts(50);
            move_group_interface_->setMaxVelocityScalingFactor(0.1);
            move_group_interface_->setMaxAccelerationScalingFactor(0.1);
            move_group_interface_->setGoalPositionTolerance(0.05);
            move_group_interface_->setGoalOrientationTolerance(0.05);

            // プランナーの設定
            move_group_interface_->setPlannerId("RRTConnect");
            move_group_interface_->setWorkspace(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);

            // 現在の状態を取得して有効性を確認
            moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState();
            if (!current_state) {
                RCLCPP_ERROR(this->get_logger(), "Failed to get current robot state");
                return;
            }

            // Visual Toolsの設定
            visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
                this->shared_from_this(),
                "base_link",
                "rviz_visual_tools",
                move_group_interface_->getRobotModel());

            visual_tools_->loadRemoteControl();
            visual_tools_->deleteAllMarkers();
            visual_tools_->trigger();

            // RVizでの操作を監視
            move_group_interface_->setEndEffectorLink("end_effector");
            move_group_interface_->allowReplanning(true);

            // ホームポジションに移動するサービスを作成
            home_service_ = this->create_service<std_srvs::srv::Trigger>(
                "go_home",
                std::bind(&MotionPlannerNode::handle_go_home, this,
                         std::placeholders::_1, std::placeholders::_2));

            // 目標姿勢に移動するサービスを作成
            target_pose_service_ = this->create_service<my_robot_arm_moveit_config::srv::MoveToPose>(
                "move_to_pose",
                std::bind(&MotionPlannerNode::handle_move_to_pose, this,
                         std::placeholders::_1, std::placeholders::_2));

            // Visual Toolsの設定
            visual_tools_->loadRemoteControl();
            visual_tools_->deleteAllMarkers();
            visual_tools_->trigger();

            // 実行
            executor->spin();
        });

        RCLCPP_INFO(this->get_logger(), "Motion Planner Node initialized");
    }

    ~MotionPlannerNode()
    {
        if (move_group_thread_.joinable()) {
            move_group_thread_.join();
        }
    }

private:
    void handle_go_home(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to home position");
        
        // ホームポジションの関節角度を設定
        std::vector<double> home_position = {0.0, 0.0, 0.0, 0.0, 0.0};
        move_group_interface_->setJointValueTarget(home_position);

        // 現在の状態を確認
        moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState();
        if (!current_state) {
            response->success = false;
            response->message = "Failed to get current robot state";
            return;
        }

        // 動作計画と実行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        // Start stateを明示的に設定
        move_group_interface_->setStartStateToCurrentState();
        
        bool success = (move_group_interface_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            success = (move_group_interface_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }

        response->success = success;
        response->message = success ? "Successfully moved to home position" : "Failed to move to home position";
    }

    void handle_move_to_pose(
        const std::shared_ptr<my_robot_arm_moveit_config::srv::MoveToPose::Request> request,
        std::shared_ptr<my_robot_arm_moveit_config::srv::MoveToPose::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to target pose");

        // 目標姿勢を設定
        move_group_interface_->setPoseTarget(request->pose);

        // 現在の状態を確認
        moveit::core::RobotStatePtr current_state = move_group_interface_->getCurrentState();
        if (!current_state) {
            response->success = false;
            response->message = "Failed to get current robot state";
            return;
        }

        // 動作計画と実行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        // Start stateを明示的に設定
        move_group_interface_->setStartStateToCurrentState();
        
        bool success = (move_group_interface_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            success = (move_group_interface_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }

        response->success = success;
        response->message = success ? "Successfully moved to target pose" : "Failed to move to target pose";
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    std::thread move_group_thread_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
    rclcpp::Service<my_robot_arm_moveit_config::srv::MoveToPose>::SharedPtr target_pose_service_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto motion_planner_node = std::make_shared<MotionPlannerNode>();
    rclcpp::spin(motion_planner_node);
    rclcpp::shutdown();
    return 0;
}
