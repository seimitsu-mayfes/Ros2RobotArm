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

        // サービスの設定
        home_service_ = this->create_service<std_srvs::srv::Trigger>(
            "go_home",
            std::bind(&MotionPlannerNode::handle_go_home, this,
                     std::placeholders::_1, std::placeholders::_2));

        target_pose_service_ = this->create_service<my_robot_arm_moveit_config::srv::MoveToPose>(
            "move_to_pose",
            std::bind(&MotionPlannerNode::handle_move_to_pose, this,
                     std::placeholders::_1, std::placeholders::_2));

        // MoveItの初期化を遅延実行
        init_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MotionPlannerNode::init_moveit, this));

        RCLCPP_INFO(this->get_logger(), "Motion Planner Node initialized");
    }

private:
    void init_moveit()
    {
        // タイマーを停止
        init_timer_->cancel();

        // MoveGroupInterfaceの初期化
        move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm_group");

        if (!move_group_interface_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize MoveGroupInterface");
            return;
        }

        // 基本設定
        move_group_interface_->setPlanningTime(20.0);             // プランニング時間を増やす
        move_group_interface_->setNumPlanningAttempts(20);        // 試行回数を増やす
        move_group_interface_->setMaxVelocityScalingFactor(0.2);  // 速度を遅くする
        move_group_interface_->setMaxAccelerationScalingFactor(0.2);
        move_group_interface_->setGoalPositionTolerance(0.05);    // 位置の許容誤差をさらに緩和
        move_group_interface_->setGoalOrientationTolerance(0.2);  // 姿勢の許容誤差をさらに緩和
        move_group_interface_->setPlannerId("RRTConnect");

        // エンドエフェクタの設定
        move_group_interface_->setEndEffectorLink("end_effector");
        move_group_interface_->allowReplanning(true);

        // Visual Toolsの設定
        visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
            shared_from_this(),
            "base_link",
            "rviz_visual_tools",
            move_group_interface_->getRobotModel());

        visual_tools_->loadRemoteControl();
        visual_tools_->deleteAllMarkers();
        visual_tools_->trigger();

        RCLCPP_INFO(this->get_logger(), "MoveIt initialization completed");
    }

    void handle_go_home(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        if (!move_group_interface_) {
            response->success = false;
            response->message = "MoveGroupInterface not initialized";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Moving to home position");
        
        // SRDFで定義されたホームポジションを使用
        move_group_interface_->setNamedTarget("home");

        // 動作計画と実行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        // Start stateを明示的に設定
        move_group_interface_->setStartStateToCurrentState();
        
        bool success = (move_group_interface_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            // 計画された軌道の各ポイントでの制御値を表示
            const auto& trajectory = plan.trajectory_.joint_trajectory;
            RCLCPP_INFO(this->get_logger(), "Planned trajectory has %ld points", trajectory.points.size());
            
            for (size_t i = 0; i < trajectory.points.size(); ++i) {
                const auto& point = trajectory.points[i];
                std::stringstream ss;
                ss << "Point " << i << " (time: " << point.time_from_start.sec << "." 
                   << std::setfill('0') << std::setw(9) << point.time_from_start.nanosec << "s)";
                
                // 関節角度を表示
                ss << "\nJoint positions: [";
                for (size_t j = 0; j < point.positions.size(); ++j) {
                    if (j > 0) ss << ", ";
                    ss << point.positions[j];
                }
                ss << "]";
                
                // 関節速度を表示
                if (!point.velocities.empty()) {
                    ss << "\nJoint velocities: [";
                    for (size_t j = 0; j < point.velocities.size(); ++j) {
                        if (j > 0) ss << ", ";
                        ss << point.velocities[j];
                    }
                    ss << "]";
                }
                
                // 制御値をJSON形式で表示
                ss << "\nControl values: {\"angles\":[";
                for (size_t j = 0; j < point.positions.size(); ++j) {
                    if (j > 0) ss << ", ";
                    ss << point.positions[j];
                }
                ss << "]}";
                
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
            }

            success = (move_group_interface_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }

        response->success = success;
        response->message = success ? "Successfully moved to home position" : "Failed to move to home position";
    }

    void handle_move_to_pose(
        const std::shared_ptr<my_robot_arm_moveit_config::srv::MoveToPose::Request> request,
        std::shared_ptr<my_robot_arm_moveit_config::srv::MoveToPose::Response> response)
    {
        if (!move_group_interface_) {
            response->success = false;
            response->message = "MoveGroupInterface not initialized";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Moving to target pose: position[%.3f, %.3f, %.3f], orientation[%.3f, %.3f, %.3f, %.3f]",
            request->pose.position.x, request->pose.position.y, request->pose.position.z,
            request->pose.orientation.x, request->pose.orientation.y, request->pose.orientation.z, request->pose.orientation.w);

        // 現在の関節角度を表示
        auto current_joints = move_group_interface_->getCurrentJointValues();
        std::stringstream ss;
        ss << "Current joint values: [";
        for (size_t i = 0; i < current_joints.size(); ++i) {
            if (i > 0) ss << ", ";
            ss << current_joints[i];
        }
        ss << "]";
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

        // 現在の関節角度を取得して少し修正
        std::vector<double> joint_values = {
            0.2,    // joint1: Z軸回転（Y方向への移動のため）
            0.7,    // joint2: X軸回転（ホームポジションと同じ）
            -0.7,   // joint3: X軸回転（ホームポジションと同じ）
            0.0,    // joint4: X軸回転
            0.0     // joint5: Z軸回転
        };
        
        move_group_interface_->setJointValueTarget(joint_values);

        // 動作計画と実行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        
        // Start stateを明示的に設定
        move_group_interface_->setStartStateToCurrentState();
        
        bool success = (move_group_interface_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            // 計画された軌道の各ポイントでのモータ制御値を表示
            RCLCPP_INFO(this->get_logger(), "Planned trajectory points:");
            for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); ++i) {
                const auto& point = plan.trajectory_.joint_trajectory.points[i];
                std::stringstream ss;
                ss << "Point " << i << " (time: " << point.time_from_start.sec << "." 
                   << point.time_from_start.nanosec / 1000000 << "s): {\"angles\":[";
                for (size_t j = 0; j < point.positions.size(); ++j) {
                    if (j > 0) ss << ", ";
                    ss << point.positions[j];
                }
                ss << "]}";
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
            }

            success = (move_group_interface_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        }

        response->success = success;
        response->message = success ? "Successfully moved to target pose" : "Failed to move to target pose";
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
    rclcpp::Service<my_robot_arm_moveit_config::srv::MoveToPose>::SharedPtr target_pose_service_;
    rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto motion_planner_node = std::make_shared<MotionPlannerNode>();
    rclcpp::spin(motion_planner_node);
    rclcpp::shutdown();
    return 0;
}
