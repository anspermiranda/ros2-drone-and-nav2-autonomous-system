import time
from math import sqrt
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle

import tf2_ros

from geometry_msgs.msg import Twist, PoseStamped
from sfr_coursework2_interface_package.action import DroneControl


MAX_LIN = 0.5   # m/s
MAX_YAW = 0.8   # rad/s  # not used here; we keep yaw = 0
SAMPLING_TIME = 0.01  # 100 Hz
TIMEOUT = 5.0         # seconds
DIST_TOL = 0.1        # meters


def clamp(value: float, min_value: float, max_value: float) -> float:
    return max(min(value, max_value), min_value)


class GammaDroidControllerNode(Node):

    def __init__(self) -> None:
        super().__init__("gamma_droid_controller_node")

        # Frame names 
        self.world_frame = "shapes"
        self.box_frame = "box"

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, "/model/box/cmd_vel", 10)

        # Action server for DroneControl
        self.action_server = ActionServer(
            self,
            DroneControl,
            "gamma_droid/set_pose",  # robot/set_pose with robot = gamma_droid
            self.execute_callback,
        )

        self.get_logger().info("gamma_droid_controller_node ready.")

    #TF helpers

    def lookup_pose_stamped(self) -> Optional[PoseStamped]:
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.box_frame,
                rclpy.time.Time(),
            )
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(
                f"TF lookup failed: {self.world_frame} -> {self.box_frame}: {ex}"
            )
            return None

        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = self.world_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    @staticmethod
    def pose_distance(a: PoseStamped, b: PoseStamped) -> float:
        
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        dz = a.pose.position.z - b.pose.position.z
        return sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def quaternion_to_rotation_matrix(q) -> Tuple[Tuple[float, float, float], ...]:

        x, y, z, w = q.x, q.y, q.z, q.w

        xx, yy, zz = x * x, y * y, z * z
        xy, xz, yz = x * y, x * z, y * z
        wx, wy, wz = w * x, w * y, w * z

        r00 = 1.0 - 2.0 * (yy + zz)
        r01 = 2.0 * (xy - wz)
        r02 = 2.0 * (xz + wy)

        r10 = 2.0 * (xy + wz)
        r11 = 1.0 - 2.0 * (xx + zz)
        r12 = 2.0 * (yz - wx)

        r20 = 2.0 * (xz - wy)
        r21 = 2.0 * (yz + wx)
        r22 = 1.0 - 2.0 * (xx + yy)

        return (
            (r00, r01, r02),
            (r10, r11, r12),
            (r20, r21, r22),
        )

    def world_error_to_body_twist(
        self, current: PoseStamped, target: PoseStamped
    ) -> Twist:
        
        k = 0.8  # proportional gain

        ex = target.pose.position.x - current.pose.position.x
        ey = target.pose.position.y - current.pose.position.y
        ez = target.pose.position.z - current.pose.position.z

        vx_w = k * ex
        vy_w = k * ey
        vz_w = k * ez

        R = self.quaternion_to_rotation_matrix(current.pose.orientation)

        # v_body = R^T * v_world
        vx_b = R[0][0] * vx_w + R[1][0] * vy_w + R[2][0] * vz_w
        vy_b = R[0][1] * vx_w + R[1][1] * vy_w + R[2][1] * vz_w
        vz_b = R[0][2] * vx_w + R[1][2] * vy_w + R[2][2] * vz_w

        twist = Twist()
        twist.linear.x = clamp(vx_b, -MAX_LIN, MAX_LIN)
        twist.linear.y = clamp(vy_b, -MAX_LIN, MAX_LIN)
        twist.linear.z = clamp(vz_b, -MAX_LIN, MAX_LIN)
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0  # any rotation is acceptable
        return twist

    def publish_stop(self) -> None:

        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)

    #Action callback

    def execute_callback(self, goal: ServerGoalHandle) -> DroneControl.Result:

        desired_pose: PoseStamped = goal.request.desired_pose

        if desired_pose.header.frame_id and desired_pose.header.frame_id != self.world_frame:
            self.get_logger().warn(
                f"desired_pose.header.frame_id is '{desired_pose.header.frame_id}', "
                f"expected '{self.world_frame}'. Using it as-is; check your client."
            )

        self.get_logger().info(
            f"Received goal to position "
            f"({desired_pose.pose.position.x:.3f}, "
            f"{desired_pose.pose.position.y:.3f}, "
            f"{desired_pose.pose.position.z:.3f}) "
            f"in frame '{desired_pose.header.frame_id}'."
        )

        feedback_msg = DroneControl.Feedback()
        result_msg = DroneControl.Result()

        max_iterations = int(TIMEOUT / SAMPLING_TIME)

        for _ in range(max_iterations):
            if not rclpy.ok():
                break

            current_pose = self.lookup_pose_stamped()
            if current_pose is None:
                time.sleep(SAMPLING_TIME)
                continue

            distance = self.pose_distance(current_pose, desired_pose)

            # Feedback: latest pose from TF
            feedback_msg.current_pose = current_pose
            goal.publish_feedback(feedback_msg)

            # Success check
            if distance <= DIST_TOL:
                self.get_logger().info(
                    f"Goal reached. Distance = {distance:.4f} m ≤ {DIST_TOL} m."
                )
                self.publish_stop()
                goal.succeed()
                result_msg.success = True
                return result_msg

            # Control command
            twist_cmd = self.world_error_to_body_twist(current_pose, desired_pose)
            self.cmd_pub.publish(twist_cmd)

            time.sleep(SAMPLING_TIME)

        # Timeout ⇒ fail
        self.get_logger().warn(
            f"Goal not reached within {TIMEOUT} seconds. Finishing with failure."
        )
        self.publish_stop()
        result_msg.success = False
        return result_msg


def main(args=None):

    try:
        rclpy.init(args=args)
        node = GammaDroidControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
