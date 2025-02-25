#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <string>
#include <thread>
#include <array>
#include <sstream>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class RobotArmController : public rclcpp::Node {
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    RobotArmController() : Node("robot_arm_controller") {
        // 現在の関節状態をパブリッシュするためのタイマーとパブリッシャー
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        joint_state_timer_ = this->create_wall_timer(
            20ms,  // 50Hz
            std::bind(&RobotArmController::publish_joint_states, this));

        // Follow Joint Trajectoryアクションサーバーの作成
        trajectory_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "arm_controller/follow_joint_trajectory",
            std::bind(&RobotArmController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotArmController::handle_cancel, this, std::placeholders::_1),
            std::bind(&RobotArmController::handle_accepted, this, std::placeholders::_1));

        // 現在の関節角度を初期化 (5軸アーム用)
        current_joint_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0};
        target_joint_positions_ = current_joint_positions_;
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5"};

        // TCP/IPサーバーの初期化とスレッド開始
        server_thread_ = std::thread(&RobotArmController::run_server, this);
        server_thread_.detach();  // スレッドをデタッチ

        RCLCPP_INFO(this->get_logger(), "Robot Arm Controller started");
    }

    ~RobotArmController() {
        if (server_sock_ >= 0) {
            close(server_sock_);
        }
        if (client_sock_ >= 0) {
            close(client_sock_);
        }
    }

private:
    // 現在の関節状態をパブリッシュ
    void publish_joint_states() {
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = this->get_clock()->now();
        joint_state.name = joint_names_;
        joint_state.position = current_joint_positions_;
        
        // 補間された位置に更新
        for (size_t i = 0; i < current_joint_positions_.size(); ++i) {
            double diff = target_joint_positions_[i] - current_joint_positions_[i];
            if (std::abs(diff) > 0.001) {  // 閾値以上の差がある場合
                current_joint_positions_[i] += diff * 0.1;  // 10%ずつ補間
            }
        }
        
        joint_state_pub_->publish(joint_state);
        
        // 現在位置を送信
        send_joint_positions();
    }

    void send_joint_positions() {
        if (client_sock_ >= 0) {
            std::stringstream ss;
            ss << "{\"angles\":[";
            for (size_t j = 0; j < current_joint_positions_.size(); ++j) {
                if (j > 0) ss << ",";
                ss << current_joint_positions_[j];
            }
            ss << "]}\n";
            std::string message = ss.str();
            
            int flag = 1;
            setsockopt(client_sock_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
            
            ssize_t sent = send(client_sock_, message.c_str(), message.length(), MSG_NOSIGNAL);
            if (sent < 0) {
                if (errno == EPIPE || errno == ECONNRESET) {
                    RCLCPP_WARN(this->get_logger(), "Connection lost during send");
                    close(client_sock_);
                    client_sock_ = -1;
                    reconnect_client();
                }
            }
        }
    }

    void reconnect_client() {
        if (client_sock_ >= 0) {
            close(client_sock_);
            client_sock_ = -1;
        }
        
        // 再接続を試みる
        RCLCPP_INFO(this->get_logger(), "Attempting to reconnect...");
        std::this_thread::sleep_for(1s);
    }

    // Follow Joint Trajectoryアクションのゴールハンドラ
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        // ジョイント名の検証
        for (const auto& name : goal->trajectory.joint_names) {
            if (std::find(joint_names_.begin(), joint_names_.end(), name) == joint_names_.end()) {
                RCLCPP_ERROR(this->get_logger(), "Received unknown joint name: %s", name.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        
        // 軌道点の検証
        if (goal->trajectory.points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory is empty");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // キャンセルハンドラ
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFollowJointTrajectory>)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 軌道実行ハンドラ
    void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        std::thread{std::bind(&RobotArmController::execute_trajectory, this, goal_handle)}.detach();
    }

    // 軌道の実行
    void execute_trajectory(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        auto result = std::make_shared<FollowJointTrajectory::Result>();

        rclcpp::Time trajectory_start = this->get_clock()->now();

        // 軌道の各ポイントを実行
        for (size_t i = 0; i < goal->trajectory.points.size(); ++i) {
            // キャンセルチェック
            if (goal_handle->is_canceling()) {
                result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
                goal_handle->canceled(result);
                return;
            }

            const auto& point = goal->trajectory.points[i];
            
            // 目標位置を更新
            for (size_t j = 0; j < goal->trajectory.joint_names.size(); ++j) {
                auto it = std::find(joint_names_.begin(), joint_names_.end(), 
                                  goal->trajectory.joint_names[j]);
                if (it != joint_names_.end()) {
                    size_t index = std::distance(joint_names_.begin(), it);
                    target_joint_positions_[index] = point.positions[j];
                }
            }

            // フィードバックを送信
            feedback->desired = point;
            feedback->actual.positions = current_joint_positions_;
            feedback->actual.time_from_start = this->get_clock()->now() - trajectory_start;
            goal_handle->publish_feedback(feedback);

            // 次のポイントまでの時間を計算して待機
            if (i < goal->trajectory.points.size() - 1) {
                const auto& next_point = goal->trajectory.points[i + 1];
                double next_time = rclcpp::Duration(next_point.time_from_start).seconds();
                double current_time = rclcpp::Duration(point.time_from_start).seconds();
                double time_diff = next_time - current_time;
                if (time_diff > 0) {
                    std::this_thread::sleep_for(std::chrono::duration<double>(time_diff));
                }
            }
        }

        // 目標位置に到達するまで待機
        bool reached = false;
        while (!reached && rclcpp::ok()) {
            reached = true;
            for (size_t i = 0; i < current_joint_positions_.size(); ++i) {
                if (std::abs(current_joint_positions_[i] - target_joint_positions_[i]) > 0.01) {
                    reached = false;
                    break;
                }
            }
            if (!reached) {
                std::this_thread::sleep_for(20ms);
            }
        }

        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        goal_handle->succeed(result);
    }

    void run_server() {
        while (rclcpp::ok()) {
            server_sock_ = socket(AF_INET, SOCK_STREAM, 0);
            if (server_sock_ < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
                std::this_thread::sleep_for(1s);
                continue;
            }

            struct sockaddr_in server_addr;
            server_addr.sin_family = AF_INET;
            server_addr.sin_addr.s_addr = INADDR_ANY;
            server_addr.sin_port = htons(5000);

            int opt = 1;
            setsockopt(server_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(server_sock_, IPPROTO_TCP, TCP_NODELAY, &opt, sizeof(opt));
            
            if (bind(server_sock_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
                close(server_sock_);
                std::this_thread::sleep_for(1s);
                continue;
            }

            if (listen(server_sock_, 1) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to listen");
                close(server_sock_);
                std::this_thread::sleep_for(1s);
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "TCP server listening on port 5000");

            while (rclcpp::ok()) {
                struct sockaddr_in client_addr;
                socklen_t client_len = sizeof(client_addr);
                client_sock_ = accept(server_sock_, (struct sockaddr *)&client_addr, &client_len);
                
                if (client_sock_ < 0) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to accept connection");
                    std::this_thread::sleep_for(1s);
                    continue;
                }

                RCLCPP_INFO(this->get_logger(), "Client connected");

                struct timeval tv;
                tv.tv_sec = 1;
                tv.tv_usec = 0;
                setsockopt(client_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
                
                char buffer[1024];
                while (rclcpp::ok()) {
                    ssize_t bytes_received = recv(client_sock_, buffer, sizeof(buffer), 0);
                    if (bytes_received <= 0) {
                        if (errno != EAGAIN && errno != EWOULDBLOCK) {
                            break;
                        }
                    }
                }

                RCLCPP_INFO(this->get_logger(), "Client disconnected");
                close(client_sock_);
                client_sock_ = -1;
            }

            close(server_sock_);
            server_sock_ = -1;
        }
    }

    // メンバー変数
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr trajectory_action_server_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    std::thread server_thread_;
    int server_sock_ = -1;
    int client_sock_ = -1;
    std::vector<std::string> joint_names_;
    std::vector<double> current_joint_positions_;
    std::vector<double> target_joint_positions_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
