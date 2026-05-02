import time
from typing import Optional

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action.client import ClientGoalHandle

# Initial pose
INITIAL_X = 0.0076481484
INITIAL_Y = 0.0135093027
INITIAL_YAW = -0.00115888599  # radians, from quaternion z/w

# Target pose (updated from latest /amcl_pose)
TARGET_X = 22.007996869781875
TARGET_Y = -7.096489856350751
TARGET_YAW = -0.1159362511  # radians, from quaternion z/w

# linear variance 0.25 for x, y, z
# angular variance 0.068 for z-axis of quaternion (yaw)
LIN_VAR = 0.25
YAW_VAR = 0.068


def yaw_to_quaternion(yaw: float):
    
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    # Simple struct replacement for orientation
    class Q:
        pass

    q = Q()
    q.x = qx
    q.y = qy
    q.z = qz
    q.w = qw
    return q


class GammaDroidNavigationNode(Node):

    def __init__(self) -> None:
        super().__init__("gamma_droid_navigation_node")

        # Topics / action names used by nav2
        self.initialpose_topic = "/initialpose"
        self.nav_action_name = "/navigate_to_pose"

        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            10,
        )

        # Action client for nav2
        self.action_client = ActionClient(
            self,
            NavigateToPose,
            self.nav_action_name,
        )

        # State flags
        self.initial_pose_sent = False
        self.goal_sent = False
        self.navigation_done = False

        # Retry + timeout control
        self.attempts = 0
        self.max_attempts = 3
        self.first_goal_start_wall_time: Optional[float] = None

        # Timer at 100 Hz (as in course notes)
        self.timer = self.create_timer(0.01, self._timer_callback)

        self.get_logger().info("gamma_droid_navigation_node started.")

    #Initial pose publisher

    def _publish_initial_pose_if_ready(self) -> None:

        subs = self.count_subscribers(self.initialpose_topic)
        if subs < 1:
            # nav2 not listening yet
            self.get_logger().debug(
                f"Waiting for subscribers on {self.initialpose_topic}..."
            )
            return

        self.get_logger().info(
            f"Subscribers detected on {self.initialpose_topic} "
            f"(count={subs}). Publishing initial pose."
        )

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Position
        msg.pose.pose.position.x = INITIAL_X
        msg.pose.pose.position.y = INITIAL_Y
        msg.pose.pose.position.z = 0.0

        # Orientation from INITIAL_YAW
        q = yaw_to_quaternion(INITIAL_YAW)
        msg.pose.pose.orientation.x = q.x
        msg.pose.pose.orientation.y = q.y
        msg.pose.pose.orientation.z = q.z
        msg.pose.pose.orientation.w = q.w

        # Covariance 6x6, flattened row-major
        # indices:
        #   (0,0) -> 0   : x variance
        #   (1,1) -> 7   : y variance
        #   (2,2) -> 14  : z variance
        #   (5,5) -> 35  : yaw variance
        for i in range(36):
            msg.pose.covariance[i] = 0.0

        msg.pose.covariance[0] = LIN_VAR      # var(x)
        msg.pose.covariance[7] = LIN_VAR      # var(y)
        msg.pose.covariance[14] = LIN_VAR     # var(z)
        msg.pose.covariance[35] = YAW_VAR     # var(yaw)

        self.initial_pose_pub.publish(msg)
        self.initial_pose_sent = True
        self.get_logger().info("Initial pose with coursework covariance published.")

    #NavigateToPose goal handling

    def _send_navigation_goal(self) -> None:

        # Make sure nav2 action server is up
        if not self.action_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info(
                f"Action server {self.nav_action_name} not available yet..."
            )
            return

        if self.attempts >= self.max_attempts:
            self.get_logger().warn(
                "Maximum navigation attempts reached. Not sending more goals."
            )
            self.navigation_done = True
            return

        self.attempts += 1

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = "map"

        goal_msg.pose.pose.position.x = TARGET_X
        goal_msg.pose.pose.position.y = TARGET_Y
        goal_msg.pose.pose.position.z = 0.0

        q = yaw_to_quaternion(TARGET_YAW)
        goal_msg.pose.pose.orientation.x = q.x
        goal_msg.pose.pose.orientation.y = q.y
        goal_msg.pose.pose.orientation.z = q.z
        goal_msg.pose.pose.orientation.w = q.w

        self.get_logger().info(
            f"Sending navigation goal attempt {self.attempts} "
            f"to ({TARGET_X:.3f}, {TARGET_Y:.3f})."
        )

        if self.first_goal_start_wall_time is None:
            self.first_goal_start_wall_time = time.time()

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._goal_response_callback)

        self.goal_sent = True

    def _goal_response_callback(self, future: Future) -> None:

        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation goal was rejected by nav2.")
            # Allow another attempt if within time
            self.goal_sent = False
            return

        self.get_logger().info("Navigation goal accepted by nav2.")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future: Future) -> None:
        
        result = future.result().result
        error_code = result.error_code

        if error_code == NavigateToPose.Result.NONE:
            self.get_logger().info(
                "Navigation succeeded. Robot reached the target."
            )
            self.navigation_done = True
            return

        self.get_logger().warn(
            f"Navigation attempt {self.attempts} failed with "
            f"error_code={error_code}, message='{result.error_msg}'."
        )

        # Allow another attempt, as long as we are still under 60 s total
        self.goal_sent = False

    #State machine at 100 Hz
    def _timer_callback(self) -> None:

        if self.navigation_done:
            return

        # Step 1: publish initial pose once nav2 is ready
        if not self.initial_pose_sent:
            self._publish_initial_pose_if_ready()
            return

        # Step 2: send nav goal if none in flight
        now = time.time()
        if not self.goal_sent:
            # Check 60 sec
            if (
                self.first_goal_start_wall_time is not None
                and now - self.first_goal_start_wall_time > 60.0
            ):
                self.get_logger().warn(
                    "Overall navigation time exceeded 60 seconds. "
                    "Not sending more goals."
                )
                self.navigation_done = True
                return

            self._send_navigation_goal()
            return

        # Step 3: goal in progress, check timeout
        if (
            self.first_goal_start_wall_time is not None
            and now - self.first_goal_start_wall_time > 60.0
        ):
            self.get_logger().warn(
                "Navigation taking longer than 60 seconds. "
                "Marking navigation as done from node perspective."
            )
            self.navigation_done = True


def main(args=None):
    try:
        rclpy.init(args=args)
        node = GammaDroidNavigationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
