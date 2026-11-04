#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from yolo_msgs.msg import DetectionArray
from geometry_msgs.msg import Twist

class FollowPerson(Node):
    def __init__(self):
        super().__init__('follow_person')

        # 当前选择的人的跟踪编号
        self.target_id = None

        # 最近收到的目标位置：(x, y)
        self.target_position = None

        # 最近收到有效目标的时间
        self.last_seen = None

        self.subscription = self.create_subscription(
            DetectionArray,
            '/yolo/cam0/detections_3d',
            self.callback,
            qos_profile_sensor_data,
        )
        self.publisher = self.create_publisher(
            Twist,
      #      '/follow_person/cmd_vel_test',
      '/cmd_vel',
            10,
        )
        # 每0.1秒执行一次控制计算
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def callback(self, msg):
        people = []

        for d in msg.detections:
            if d.class_name != 'person':
                continue
            if d.score < 0.7:
                continue
            if d.bbox3d.frame_id != 'pelvis':
                continue

            p = d.bbox3d.center.position

            if not all(math.isfinite(v) for v in (p.x, p.y, p.z)):
                continue
            if p.x <= 0.0:
                continue
            if not d.id:
                continue

            people.append(d)

        # 还没选目标时，选择水平距离最近的人
        if self.target_id is None and people:
            nearest = min(
                people,
                key=lambda d: math.hypot(
                    d.bbox3d.center.position.x,
                    d.bbox3d.center.position.y,
                ),
            )
            self.target_id = nearest.id
        
        self.get_logger().info(
            f'received={len(msg.detections)}, '
            f'valid_ids={[d.id for d in people]}, '
            f'locked_id={self.target_id}',
            throttle_duration_sec=2.0,
        )
        # 在这一帧里寻找已选定的目标
        target = next(
            (d for d in people if d.id == self.target_id),
            None,
        )

        if target is None:
            self.target_position = None
            return

        p = target.bbox3d.center.position
        self.target_position = (p.x, p.y)
        self.last_seen = time.monotonic()

    def control_loop(self):
        linear = 0.0
        angular = 0.0
        reason = 'no target'

        if self.target_position is not None and self.last_seen is not None:
            age = time.monotonic() - self.last_seen

            if age <= 0.5:
                x, y = self.target_position

                distance_error = x - 0.5
                angle_error = math.atan2(y, x)

                self.get_logger().info(
                    f'x={x:.2f} m, y={y:.2f} m, '
                    f'angle={math.degrees(angle_error):.1f} deg',
                    throttle_duration_sec=1.0,
                )

                if abs(angle_error) > 0.05:
                    angular = max(
                        -0.3, min(0.3, 0.8 * angle_error)
                    )

                if distance_error > 0.0 and abs(angle_error) < math.radians(45):
                    linear = min(0.8, 0.4 * distance_error)

                reason = 'tracking'
            else:
                reason = 'target data timeout'

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.publisher.publish(cmd)

        self.get_logger().info(
            f'{reason}: linear.x={linear:.3f} m/s, '
            f'angular.z={angular:.3f} rad/s'
        )

def main(args=None):
    rclpy.init(args=args)
    node = FollowPerson()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
