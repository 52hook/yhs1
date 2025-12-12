#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class LioToEkfBridge(Node):
    """
    读取 TF(map->base_link) + LIO twist，
    输出：
      - /odometry/filtered   (frame: map -> base_link)
      - /ekf/fused_yaw       (Float32, rad)
    注意：不广播 TF，避免 TF 冲突。
    """

    def __init__(self):
        super().__init__('lio_to_ekf_bridge')

        # 参数
        self.declare_parameter('lio_odom_topic', '/fastlio2/lio_odom')
        self.declare_parameter('output_odom_topic', '/odometry/filtered')
        self.declare_parameter('yaw_topic', '/ekf/fused_yaw')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # tf 等待时长 & 日志节流
        self.declare_parameter('tf_timeout_sec', 0.3)          # 0.1~0.3 建议
        self.declare_parameter('warn_throttle_sec', 1.0)       # 1 秒最多 warn 一次
        self.declare_parameter('fallback_to_latest', True)

        self.lio_odom_topic = self.get_parameter('lio_odom_topic').value
        self.output_odom_topic = self.get_parameter('output_odom_topic').value
        self.yaw_topic = self.get_parameter('yaw_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self.warn_throttle = float(self.get_parameter('warn_throttle_sec').value)
        self.fallback_to_latest = bool(self.get_parameter('fallback_to_latest').value)

        # TF buffer & listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 发布话题
        self.odom_pub = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.yaw_pub = self.create_publisher(Float32, self.yaw_topic, 10)

        # 订阅 LIO odom（用它的 twist + stamp）
        self.sub = self.create_subscription(Odometry, self.lio_odom_topic, self.odom_cb, 50)

        # warn 节流
        self._last_warn_ns = 0

        self.get_logger().info(
            f"Bridge started. LIO odom: {self.lio_odom_topic} -> "
            f"{self.output_odom_topic}, {self.yaw_topic}. "
            f"TF lookup: {self.global_frame}->{self.base_frame}, timeout={self.tf_timeout}s"
        )

    def _warn_throttled(self, text: str):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_warn_ns > int(self.warn_throttle * 1e9):
            self.get_logger().warn(text)
            self._last_warn_ns = now_ns

    def odom_cb(self, msg: Odometry):
        # 用本节点 clock_type 构造时间，避免 time source 不一致
        clock_type = self.get_clock().clock_type

        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg_time = Time(clock_type=clock_type)  # 等价 latest
        else:
            msg_time = Time(
                seconds=int(msg.header.stamp.sec),
                nanoseconds=int(msg.header.stamp.nanosec),
                clock_type=clock_type
            )

        tf_map_base = None
        used_latest = False

        # 1) 优先按 msg 时间戳查（允许等待一小段时间让 TF 赶上）
        try:
            tf_map_base = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.base_frame,
                msg_time,
                timeout=Duration(seconds=self.tf_timeout)
            )
        except TransformException as ex1:
            if not self.fallback_to_latest:
                self._warn_throttled(f"TF not ready at msg time: {ex1}")
                return

            # 2) 兜底：查 latest（Time=0）
            try:
                tf_map_base = self.tf_buffer.lookup_transform(
                    self.global_frame,
                    self.base_frame,
                    Time(clock_type=clock_type),  # latest
                    timeout=Duration(seconds=self.tf_timeout)
                )
                used_latest = True
                self._warn_throttled(f"TF at msg time not ready, fallback to latest. ex: {ex1}")
            except TransformException as ex2:
                self._warn_throttled(f"Waiting for TF {self.global_frame}->{self.base_frame}: {ex2}")
                return

        # 输出 stamp：若用了 latest，则用 TF 自己的 stamp，更一致
        out_stamp = tf_map_base.header.stamp if used_latest else msg.header.stamp

        trans = tf_map_base.transform.translation
        rot = tf_map_base.transform.rotation

        # 组装 /odometry/filtered
        odom_out = Odometry()
        odom_out.header.stamp = out_stamp
        odom_out.header.frame_id = self.global_frame
        odom_out.child_frame_id = self.base_frame

        odom_out.pose.pose.position.x = trans.x
        odom_out.pose.pose.position.y = trans.y
        odom_out.pose.pose.position.z = trans.z

        odom_out.pose.pose.orientation.x = rot.x
        odom_out.pose.pose.orientation.y = rot.y
        odom_out.pose.pose.orientation.z = rot.z
        odom_out.pose.pose.orientation.w = rot.w

        # twist 直接沿用 LIO twist（注意它通常是 body/body_link 坐标系下）
        odom_out.twist = msg.twist

        self.odom_pub.publish(odom_out)

        # 发布 yaw
        siny_cosp = 2.0 * (rot.w * rot.z + rot.x * rot.y)
        cosy_cosp = 1.0 - 2.0 * (rot.y * rot.y + rot.z * rot.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        yaw_msg = Float32()
        yaw_msg.data = float(yaw)
        self.yaw_pub.publish(yaw_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LioToEkfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

