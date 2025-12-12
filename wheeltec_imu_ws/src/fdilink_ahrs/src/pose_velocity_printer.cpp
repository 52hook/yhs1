#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"       // 用于位置和速度
#include "sensor_msgs/msg/imu.hpp"         // 用于IMU四元数
#include "tf2/LinearMath/Quaternion.h"     // 四元数处理
#include "tf2/LinearMath/Matrix3x3.h"      // 四元数转欧拉角
#include <cmath>                           // 弧度转角度

// 手动定义PI，避免环境差异
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class PoseVelocityPrinter : public rclcpp::Node
{
public:
    PoseVelocityPrinter() : Node("pose_velocity_printer")
    {
        // 1. 订阅/odom：获取位置（x/y）和速度（linear.x）
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PoseVelocityPrinter::odom_callback, this, std::placeholders::_1));

        // 2. 订阅/imu：获取四元数（用于计算航向角）
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&PoseVelocityPrinter::imu_callback, this, std::placeholders::_1));
    }

private:
    // 处理odom数据：更新位置和速度
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x = msg->pose.pose.position.x;          // 位置x
        current_y = msg->pose.pose.position.y;          // 位置y
        current_linear_x = msg->twist.twist.linear.x;   // 纵向速度
        print_data();  // 数据更新后打印
    }

    // 处理IMU数据：从四元数转换航向角（角度）
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 提取IMU的四元数（x,y,z,w）
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );

        // 四元数转欧拉角（roll, pitch, yaw），单位：弧度
        tf2::Matrix3x3 m(q);
        double roll_rad, pitch_rad, yaw_rad;
        m.getRPY(roll_rad, pitch_rad, yaw_rad);

        // 弧度转角度（范围：±180°）
        current_yaw = yaw_rad * (180.0 / M_PI);

        print_data();  // 数据更新后打印
    }

    // 打印所有数据
    void print_data()
    {
        RCLCPP_INFO(this->get_logger(), 
            "X: %.2f, Y: %.2f, 航向角: %.2f°, 纵向速度: %.2f",
            current_x, current_y, current_yaw, current_linear_x);
    }

    // 订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // 数据变量
    double current_x = 0.0, current_y = 0.0;         // 位置（来自/odom）
    double current_yaw = 0.0;                        // 航向角（角度，来自/imu四元数）
    double current_linear_x = 0.0;                   // 纵向速度（来自/odom）
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseVelocityPrinter>());
    rclcpp::shutdown();
    return 0;
}

