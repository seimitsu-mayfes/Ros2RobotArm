#include <rclcpp/rclcpp.hpp>
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

class RobotArmController : public rclcpp::Node {
public:
    RobotArmController() : Node("robot_arm_controller"), time_counter_(0.0) {
        // 定期的にジョイント角度を生成・パブリッシュするタイマーを作成
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 100ms間隔で更新(より高速な更新)
            std::bind(&RobotArmController::timer_callback, this));

        // ジョイント角度をパブリッシュするパブリッシャーを作成
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "joint_states", 10);

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
    void timer_callback() {
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = this->get_clock()->now();
        
        // 5つのジョイントの名前を設定
        joint_state.name = {
            "joint1", "joint2", "joint3", "joint4", "joint5"
        };

        // 時間に基づいて変化する角度値を設定
        double amplitude = 0.5;  // 振幅
        double frequency = 0.5;  // 周波数
        joint_state.position = {
            amplitude * sin(frequency * time_counter_),          // joint1: サイン波
            amplitude * cos(frequency * time_counter_),          // joint2: コサイン波
            amplitude * sin(frequency * time_counter_ * 2.0),    // joint3: 2倍の周波数
            amplitude * cos(frequency * time_counter_ * 0.5),    // joint4: 半分の周波数
            amplitude * sin(frequency * time_counter_ * 1.5)     // joint5: 1.5倍の周波数
        };
        
        // カウンターを更新
        time_counter_ += 0.1;  // 100msごとに0.1増加

        // パブリッシュ
        joint_pub_->publish(joint_state);

        // TCP/IP経由でBLECentral.pyに送信(最適化版)
        if (client_sock_ >= 0) {
            std::stringstream ss;
            ss << "{\"angles\":[";
            for (size_t i = 0; i < joint_state.position.size(); ++i) {
                if (i > 0) ss << ",";
                ss << joint_state.position[i];
            }
            ss << "]}\n";
            std::string message = ss.str();
            
            // TCP_NODELAYオプションを設定して即時送信
            int flag = 1;
            setsockopt(client_sock_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
            
            // エラーチェック付きの送信
            ssize_t sent = send(client_sock_, message.c_str(), message.length(), MSG_NOSIGNAL);
            if (sent < 0) {
                if (errno == EPIPE || errno == ECONNRESET) {
                    RCLCPP_WARN(this->get_logger(), "Connection lost during send");
                    close(client_sock_);
                    client_sock_ = -1;
                }
            }
        }
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

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::thread server_thread_;
    int server_sock_ = -1;
    int client_sock_ = -1;
    double time_counter_;  // 時間カウンター
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotArmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
