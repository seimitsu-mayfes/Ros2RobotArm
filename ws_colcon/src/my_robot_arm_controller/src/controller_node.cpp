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
            50ms,  // 20Hz
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
        joint_state_pub_->publish(joint_state);
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

        // 軌道の各ポイントを実行
        for (size_t i = 0; i < goal->trajectory.points.size(); ++i) {
            // キャンセルチェック
            if (goal_handle->is_canceling()) {
                result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
                goal_handle->canceled(result);
                return;
            }

            // 目標位置を取得
            const auto& point = goal->trajectory.points[i];
            
            // 現在の関節角度を更新
            for (size_t j = 0; j < goal->trajectory.joint_names.size(); ++j) {
                auto it = std::find(joint_names_.begin(), joint_names_.end(), 
                                  goal->trajectory.joint_names[j]);
                if (it != joint_names_.end()) {
                    size_t index = std::distance(joint_names_.begin(), it);
                    current_joint_positions_[index] = point.positions[j];
                }
            }

            // TCP/IP経由でBLECentral.pyに送信
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
                        result->error_code = FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
                        goal_handle->abort(result);
                        return;
                    }
                }
            }

            // フィードバックを送信
            feedback->actual = point;
            goal_handle->publish_feedback(feedback);

            // 次のポイントまで待機
            if (i < goal->trajectory.points.size() - 1) {
                std::this_thread::sleep_for(50ms);  // 20Hz
            }
        }

        // 成功
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        goal_handle->succeed(result);
    }

    void run_server() {
        server_sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(5000);  // ポート5000を使用

        // ソケットオプションの設定
        int opt = 1;
        setsockopt(server_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        // 送信バッファサイズを最適化
        int sendbuff = 32768;
        setsockopt(server_sock_, SOL_SOCKET, SO_SNDBUF, &sendbuff, sizeof(sendbuff));
        
        // Nagleアルゴリズムを無効化して即時送信
        int flag = 1;
        setsockopt(server_sock_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        
        // キープアライブを有効化
        int keepalive = 1;
        setsockopt(server_sock_, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive));

        if (bind(server_sock_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            close(server_sock_);
            return;
        }

        if (listen(server_sock_, 1) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to listen");
            close(server_sock_);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "TCP server listening on port 5000");

        while (rclcpp::ok()) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            client_sock_ = accept(server_sock_, (struct sockaddr *)&client_addr, &client_len);
            
            if (client_sock_ < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to accept connection");
                continue;
            }

            RCLCPP_INFO(this->get_logger(), "Client connected");

            // クライアントからのメッセージを受信
            char buffer[1024];
            ssize_t bytes_received;
            
            // タイムアウト設定
            struct timeval tv;
            tv.tv_sec = 1;  // 1秒
            tv.tv_usec = 0;
            setsockopt(client_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            
            while (true) {
                bytes_received = recv(client_sock_, buffer, sizeof(buffer), 0);
                if (bytes_received > 0) {
                    // メッセージを受信した場合は継続
                    continue;
                } else if (bytes_received == 0) {
                    // クライアントが正常に切断した場合
                    break;
                } else {
                    // エラーの場合
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        // タイムアウトの場合は継続
                        continue;
                    } else {
                        // その他のエラーの場合は切断
                        break;
                    }
                }
            }

            RCLCPP_INFO(this->get_logger(), "Client disconnected");
            close(client_sock_);
            client_sock_ = -1;
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
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
