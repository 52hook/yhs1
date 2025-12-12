#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import torch
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R  # 替代tf.transformations

# 导入ai-imu-dr核心模块（需确保仓库路径在PYTHONPATH中）
import sys
sys.path.append("/path/to/ai-imu-dr")  # 替换为ai-imu-dr仓库的实际路径
from src.utils_torch_filter import TORCHIEKF
from src.utils_numpy_filter import to_rpy  # 旋转矩阵转RPY

class AI_IMU_DR_Node(Node):
    def __init__(self):
        super().__init__('ai_imu_dr_node')
        
        # 订阅IMU话题（默认/imu/data，可通过参数修改）
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10  # QoS深度
        )
        
        # 发布里程计消息
        self.odom_pub = self.create_publisher(
            Odometry,
            '/ai_imu_dr/odometry',
            10
        )
        
        # TF广播器（发布base_link到odom的变换）
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 初始化算法
        self.iekf = TORCHIEKF()
        self.load_model()  # 加载预训练模型
        
        # 状态变量
        self.last_time = None  # 上一帧IMU时间戳（秒）
        self.imu_buffer = []  # 缓存IMU数据（(dt, gyro+acc)）
        self.initialized = False  # 是否完成初始化

    def load_model(self):
        # 加载预训练模型参数（默认路径：ai-imu-dr/temp/iekfnets.p）
        model_path = self.declare_parameter('model_path', '/path/to/ai-imu-dr/temp/iekfnets.p').value
        try:
            self.iekf.load_state_dict(torch.load(model_path))
            self.iekf.eval()  # 推理模式
            self.get_logger().info(f"模型加载成功：{model_path}")
        except Exception as e:
            self.get_logger().error(f"模型加载失败：{e}")

    def imu_callback(self, msg: Imu):
        # 提取IMU数据（加速度计+陀螺仪，单位：m/s²，rad/s）
        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        imu_data = np.concatenate([gyro, acc], axis=0)  # 格式：[gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z]
        
        # 计算时间差dt（当前时间 - 上一帧时间）
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9  # 转换为秒
        if self.last_time is None:
            dt = 0.01  # 初始默认dt（根据IMU频率调整，如100Hz则0.01s）
        else:
            dt = current_time - self.last_time
        self.last_time = current_time
        
        # 缓存数据（累计一定量后运行算法，平衡实时性与精度）
        self.imu_buffer.append((dt, imu_data))
        if len(self.imu_buffer) >= 5:  # 每5帧计算一次（可调整）
            self.run_estimation()
            self.imu_buffer = []  # 清空缓存（或保留部分做滑动窗口）

    def run_estimation(self):
        # 解析缓存的IMU数据
        dt_list = [item[0] for item in self.imu_buffer]
        u = torch.tensor([item[1] for item in self.imu_buffer], dtype=torch.double)  # 输入IMU数据
        
        # 初始化滤波器（首次运行时）
        if not self.initialized:
            self.iekf.initialize()  # 调用仓库初始化方法
            self.initialized = True
            self.get_logger().info("滤波器初始化完成")
        
        # 运行AI-IMU-DR算法（推理模式，禁用梯度）
        with torch.no_grad():
            # 计算噪声协方差
            measurements_covs = self.iekf.forward_nets(u)
            # 运行卡尔曼滤波，得到姿态(Rot)、速度(v)、位置(p)
            Rot, v, p, _, _, _, _ = self.iekf.run(
                t=torch.tensor(dt_list, dtype=torch.double).cumsum(0),  # 累计时间
                u=u,
                measurements_covs=measurements_covs,
                v_gt=None,  # 无真值时设为None
                p_gt=None,
                N=len(self.imu_buffer),
                init_ang=None
            )
        
        # 提取最新估计结果（最后一帧）
        latest_p = p[-1].numpy()  # 位置 [x, y, z]
        latest_rot = Rot[-1].numpy()  # 旋转矩阵
        latest_v = v[-1].numpy()  # 速度 [vx, vy, vz]
        
        # 发布里程计和TF变换
        self.publish_odometry(latest_p, latest_rot, latest_v, self.last_time)
        self.publish_tf(latest_p, latest_rot, self.last_time)

    def publish_odometry(self, pos, rot, vel, timestamp):
        odom = Odometry()
        # 头部信息
        odom.header.stamp = self.get_clock().now().to_msg()  # 或用IMU时间戳
        odom.header.frame_id = "odom"  # 父坐标系
        odom.child_frame_id = "base_link"  # 子坐标系
        
        # 位置
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        
        # 姿态（旋转矩阵→四元数）
        rpy = to_rpy(rot)  # 调用仓库的RPY转换
        quat = R.from_euler('xyz', rpy).as_quat()  # (x, y, z, w)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # 速度
        odom.twist.twist.linear.x = vel[0]
        odom.twist.twist.linear.y = vel[1]
        odom.twist.twist.linear.z = vel[2]
        
        self.odom_pub.publish(odom)

    def publish_tf(self, pos, rot, timestamp):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        # 平移
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]
        
        # 旋转（同里程计的四元数）
        rpy = to_rpy(rot)
        quat = R.from_euler('xyz', rpy).as_quat()
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = AI_IMU_DR_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点停止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

