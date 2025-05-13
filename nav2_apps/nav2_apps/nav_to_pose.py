import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

class NavigateToPoseClient(Node):
    def __init__(self):
        super().__init__('nav_to_pose')
        self._action_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'  # Nav2 default action server name
        )

    def send_goal(self, goal_pose):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return

        self.get_logger().info("Goal accepted. Navigating...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        status_str = self._status_to_string(status)
        self.get_logger().info(f"Navigation finished. Status: {status_str}")
        rclpy.shutdown()  # Optional: Shutdown after completion

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Distance remaining: {feedback.distance_remaining:.2f} meters"
        )

    def _status_to_string(self, status):
        return {
            GoalStatus.STATUS_UNKNOWN: "Unknown",
            GoalStatus.STATUS_ACCEPTED: "Accepted",
            GoalStatus.STATUS_EXECUTING: "Executing",
            GoalStatus.STATUS_CANCELED: "Canceled",
            GoalStatus.STATUS_SUCCEEDED: "Succeeded",  # Status 4
            GoalStatus.STATUS_ABORTED: "Aborted",
        }.get(status, "Invalid Status")

def main(args=None):
    rclpy.init(args=args)
    client = NavigateToPoseClient()

    # Define goal pose (replace with your target coordinates)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"  # Must match your map frame
    goal_pose.header.stamp = client.get_clock().now().to_msg()  # Critical!
    goal_pose.pose.position.x = 5.8
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.orientation.z = -0.706825181105366  # No rotation
    goal_pose.pose.orientation.w = 0.7073882691671998  # No rotation

    # Send goal and spin
    if client.send_goal(goal_pose):
        rclpy.spin(client)
    else:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()