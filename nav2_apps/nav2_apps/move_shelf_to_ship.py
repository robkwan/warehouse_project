from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from attach_shelf.srv import GoToLoading
from std_srvs.srv import Empty  # Import the Empty service
from rcl_interfaces.msg import ParameterDescriptor
import time
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

class MoveShelfToShip(Node):

    def __init__(self):
        super().__init__('move_shelf_to_ship',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self._action_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'  # Nav2 default action server name
        )
        self.initial_pose_publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        #self.amcl_pose_subscriber_ = self.create_subscription(
        #    PoseWithCovarianceStamped,
        #    '/amcl_pose',
        #    self.amcl_pose_callback,
        #    10,
        #    callback_group=MutuallyExclusiveCallbackGroup())
        self.approach_client = self.create_client(GoToLoading, '/approach_shelf')
        self.reinit_client_ = self.create_client(Empty, '/reinitialize_global_localization')
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)

        self.amcl_pose_received_ = False
        self.goal_reached_ = False
        self.initial_amcl_pose_ = None
        self.posx = 0.0
        self.posy = 0.0
        self.posz = 0.0
        self.ox = 0.0
        self.oy = 0.0
        self.oz = 0.0
        self.ow = 1.0
        # Add parameter client for dynamic footprint changes
        self.param_client = self.create_client(
            SetParameters,
            '/global_costmap/global_costmap/set_parameters'
        )

    async def update_footprint(self):
        new_footprint = "[ [0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5] ]"  # Your new footprint coordinates
        
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name="footprint",
            value=new_footprint
        ).to_parameter_msg()]

        future = self.param_client.call_async(req)
        await future
        
        if future.result() is not None:
            self.get_logger().info("Footprint updated successfully")
            return True
        else:
            self.get_logger().error("Failed to update footprint")
            return False

    def find_spots(self, name):
        param_prefix = f'move_to_spot.{name}'
        param_names = ['x', 'y', 'z', 'ox', 'oy', 'oz', 'ow']
        descriptor = ParameterDescriptor(dynamic_typing=True)

        for param in param_names:
            full_name = f'{param_prefix}.{param}'
            if not self.has_parameter(full_name):
                self.declare_parameter(full_name, 0.0, descriptor=descriptor)  # Use 0.0 instead of None

        # Retrieve parameters
        self.posx = self.get_parameter(f'{param_prefix}.x').value
        self.posy = self.get_parameter(f'{param_prefix}.y').value
        self.posz = self.get_parameter(f'{param_prefix}.z').value
        self.ox = self.get_parameter(f'{param_prefix}.ox').value
        self.oy = self.get_parameter(f'{param_prefix}.oy').value
        self.oz = self.get_parameter(f'{param_prefix}.oz').value
        self.ow = self.get_parameter(f'{param_prefix}.ow').value

        self.get_logger().info(f"Going to spot '{name}': x={self.posx}, y={self.posy}, oz={self.oz}")


    def amcl_pose_callback(self, msg):
        self.amcl_pose_received_ = True
        self.get_logger().info(f"Received pose from AMCL: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y
        self.amcl_cov0 = msg.pose.covariance[0]
        self.amcl_cov1 = msg.pose.covariance[1]
        self.amcl_cov6 = msg.pose.covariance[6]
        self.amcl_cov7 = msg.pose.covariance[7]

    def is_amcl_node_active(self):
        try:
            # Replace 'amcl_node_name' with the actual name of your AMCL node
            node_info = self.get_node_names()  # List all active nodes
            return 'amcl' in node_info
        except Exception as e:
            self.get_logger().error(f"Error checking AMCL node status: {e}")
            return False

    def set_init_pose(self, x=0.0, y=0.0, yaw=0.0, map_frame='map'):
        x = self.posx
        y = self.posy
        #yaw = self.oz
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = map_frame
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        #initial_pose_msg.header.stamp.sec = 0
        #initial_pose_msg.header.stamp.nanosec = 0
        initial_pose_msg.pose.pose.position.x = x
        initial_pose_msg.pose.pose.position.y = y
        #q = quaternion_from_euler(0.0, 0.0, yaw)
        initial_pose_msg.pose.pose.orientation.x = self.ox
        initial_pose_msg.pose.pose.orientation.y = self.oy
        initial_pose_msg.pose.pose.orientation.z = self.oz
        initial_pose_msg.pose.pose.orientation.w = self.ow
        #self.get_logger().info(f"q0:{q[0]},q1:{q[1]},q2:{q[2]},q3:{q[3]}")
        initial_pose_msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]
        
        while not self.initial_pose_publisher_.get_subscription_count():
            self.get_logger().info("Waiting for subscribers to connect...")
            time.sleep(1)

        max_retries = 5
        for attempt in range(max_retries):
            try:
                self.initial_pose_publisher_.publish(initial_pose_msg)
                time.sleep(1)
                self.get_logger().info("Initial pose published successfully.")
                break  # Exit the loop if successful
            except Exception as e:
                self.get_logger().error(f"Failed to publish initial pose (attempt {attempt + 1}): {e}")
                time.sleep(1)  # Wait before retrying
        else:
            self.get_logger().warn("Exceeded maximum retries to publish initial pose.")

    def wait_for_amcl_localization(self, timeout_sec=5.0):
        start_time = self.get_clock().now()
        while rclpy.ok() and not self.amcl_pose_received_:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed_time > timeout_sec:
                self.get_logger().warn("AMCL did not publish a pose within the timeout.")
                return False
        self.get_logger().info("AMCL is localized.")
        return True

    def reinitialize_global_localization(self):
        self.get_logger().info("Calling /reinitialize_global_localization service...")
        request = Empty.Request()
        future = self.reinit_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Successfully called /reinitialize_global_localization")
            return True
        else:
            self.get_logger().error("Failed to call /reinitialize_global_localization")
            return False

    def rotate_robot(self, rotation_duration=30.0, angular_velocity=0.5):
        self.get_logger().info("Rotating robot to assist localization...")
        twist_msg = Twist()
        twist_msg.angular.z = float(angular_velocity)
        start_time = self.get_clock().now()

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > rotation_duration:
                break
            self.cmd_vel_publisher_.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.amcl_pose_received_:
                if (self.amcl_cov0 < 0.25 and self.amcl_cov1 < 0.25 and self.amcl_cov6 < 0.25):
                    break

        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(twist_msg)
        self.get_logger().info("Finished rotating robot.")

    def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available!")
            return False
        
        # Define goal pose (replace with your target coordinates)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"  # Must match your map frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()  # Critical!
        goal_pose.pose.position.x = self.posx
        goal_pose.pose.position.y = self.posy
        goal_pose.pose.orientation.z = self.oz  # No rotation
        goal_pose.pose.orientation.w = self.ow  # No rotation

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
    
        if status == GoalStatus.STATUS_SUCCEEDED:
            if not self.goal_reached_:
                # First goal completion (attach shelf)
                self.goal_reached_ = True
                if self.attach_shelf():
                    # Update footprint and send second goal
                    self.get_logger().info("Updating footprint...")
                    rclpy.get_default_context().call_soon(self.handle_footprint_update)
                else:
                    self.clean_shutdown()
            elif self.footprint_updated:
                # Second goal completion
                self.get_logger().info("All tasks completed!")
                self.clean_shutdown()
        else:
            self.clean_shutdown()

    async def handle_footprint_update(self):
        success = await self.update_footprint()
        if success:
            self.footprint_updated = True
            self.find_spots('ship_pt')  # Load second goal params
            self.send_goal()
        else:
            self.clean_shutdown()

    def clean_shutdown(self):
        self.destroy_node()
        rclpy.shutdown()

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
    
    def attach_shelf(self):
        if not self.goal_reached_:
            self.get_logger().warn("attach_shelf called before goal was reached!")
            return
        while not self.approach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /approach_shelf service...")
        req = GoToLoading.Request()
        req.attach_to_shelf = True
        future = self.approach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveShelfToShip()

    action_client.find_spots('init_pos')
    action_client.set_init_pose()

    #if not action_client.wait_for_amcl_localization():
    #    action_client.rotate_robot(rotation_duration=30.0)
    #    if not action_client.wait_for_amcl_localization():
    #        if action_client.reinitialize_global_localization():
    #            action_client.rotate_robot(rotation_duration=30.0)
    #            action_client.wait_for_amcl_localization()
    #        else:
    #            action_client.get_logger().error("Failed to reinitialize global localization. Aborting.")
    #            rclpy.shutdown()
    #            return
    action_client.find_spots('turn_pt')

    # Send goal and spin
    if action_client.send_goal():
        rclpy.spin(action_client)
    else:
        action_client.destroy_node()
        rclpy.shutdown()

