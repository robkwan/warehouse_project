#! /usr/bin/env python3

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
from std_msgs.msg import String  # Added import

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

        # Check if 'use_sim_time' is True or False
        #self.use_sim_time = self.get_parameter('use_sim_time').value
        #self.get_logger().info(f"use_sim_time is set to: {self.use_sim_time}")

        self.initial_pose_publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.amcl_pose_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.approach_client = self.create_client(GoToLoading, '/approach_shelf')
        self.reinit_client_ = self.create_client(Empty, '/reinitialize_global_localization')
        #if (self.use_sim_time):
        #self.cmd_vel_publisher_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)
        #else:
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Hardcoded spot parameters (replaces spot-list.yaml)
        self.spot_params = {
            'init_pos': {
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0
            },
            'turn_pt': {
                'x': -3.0, 'y': 4.2, 'z': 0.0,
                #'ox': 0.0, 'oy': 0.0, 'oz': -0.707, 'ow': 0.707
                'ox': 0.0, 'oy': 0.0, 'oz': 0.0, 'ow': 1.0
            },
            'ship_pt': {
                'x': -3.0, 'y': 1.6, 'z': 0.0,
                'ox': 0.0, 'oy': 0.0, 'oz': -0.707, 'ow': 0.707
            }
        }

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
        self._goal_handle = None
        self.goal_accepted = False
        # Add parameter client for dynamic footprint changes
        self.gparam_client = self.create_client(
            SetParameters,
            '/global_costmap/global_costmap/set_parameters'
        )
        self.lparam_client = self.create_client(
            SetParameters,
            '/local_costmap/local_costmap/set_parameters'
        )
        # Publisher for /elevator_down
        self.elevator_pub_ = self.create_publisher(String, '/elevator_down', 10)

    def update_footprint(self):
        new_footprint = "[ [0.45, 0.45], [0.45, -0.45], [-0.45, -0.45], [-0.45, 0.45] ]"
        new_inflation_radius = 1.05  # Set your desired value (default is 0.55)

        req = SetParameters.Request()
        req.parameters = [
        Parameter(name="footprint", value=new_footprint).to_parameter_msg(),
        #Parameter(name="inflation_layer.inflation_radius", value=new_inflation_radius).to_parameter_msg()
        ]

        future = self.gparam_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)  # Block until done
    
        if future.result() is not None:
            self.get_logger().info("G Footprint updated successfully")
            #return True
        else:
            self.get_logger().error("Failed to update G footprint")
            return False

        req = SetParameters.Request()
        req.parameters = [
        Parameter(name="footprint", value=new_footprint).to_parameter_msg(),
        #Parameter(name="inflation_layer.inflation_radius", value=new_inflation_radius).to_parameter_msg()
        ]

        future = self.lparam_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)  # Block until done
    
        if future.result() is not None:
            self.get_logger().info("L Footprint updated successfully")
            
            # Reset costmaps
            #clear_global = self.create_client(Empty, '/global_costmap/clear_entirely_global_costmap')
            #clear_local = self.create_client(Empty, '/local_costmap/clear_entirely_local_costmap')
            #clear_global.call_async(Empty.Request())
            #clear_local.call_async(Empty.Request())
            #time.sleep(1.0)  # Allow costmaps to regenerate

            return True
        else:
            self.get_logger().error("Failed to update L footprint")
            return False

    def return_footprint(self):
        orig_footprint = "[ [0.2575, 0.169], [0.2575, -0.169], [-0.2575, -0.169], [-0.2575, 0.169] ]"

        req = SetParameters.Request()
        req.parameters = [
        Parameter(name="footprint", value=orig_footprint).to_parameter_msg(),
        ]

        future = self.gparam_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)  # Block until done
    
        if future.result() is not None:
            self.get_logger().info("G Footprint returned successfully")
            #return True
        else:
            self.get_logger().error("Failed to return G footprint")
            return False

        req = SetParameters.Request()
        req.parameters = [
        Parameter(name="footprint", value=orig_footprint).to_parameter_msg(),
        ]

        future = self.lparam_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)  # Block until done
    
        if future.result() is not None:
            self.get_logger().info("L Footprint returned successfully")
            return True
        else:
            self.get_logger().error("Failed to return L footprint")
            return False


    def find_spots(self, name):
        if name not in self.spot_params:
            self.get_logger().error(f"Spot '{name}' not defined!")
            return

        spot = self.spot_params[name]
        self.posx = spot['x']
        self.posy = spot['y']
        self.posz = spot['z']
        self.ox = spot['ox']
        self.oy = spot['oy']
        self.oz = spot['oz']
        self.ow = spot['ow']

        self.get_logger().info(f"Going to spot '{name}': x={self.posx}, y={self.posy}, oz={self.oz}")
        
    def amcl_pose_callback(self, msg):
        self.amcl_pose_received_ = True
        self.get_logger().info(f"Received pose from AMCL: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y
        self.amcl_cov0 = msg.pose.covariance[0]
        self.amcl_cov6 = msg.pose.covariance[6]
        self.amcl_cov35 = msg.pose.covariance[35]

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

        for _ in range(3):
            self.initial_pose_publisher_.publish(initial_pose_msg)
            time.sleep(0.5)
            
        #max_retries = 5
        #for attempt in range(max_retries):
        #    try:
        #        self.initial_pose_publisher_.publish(initial_pose_msg)
        #        time.sleep(1)
        #        self.get_logger().info("Initial pose published successfully.")
        #        break  # Exit the loop if successful
        #    except Exception as e:
        #        self.get_logger().error(f"Failed to publish initial pose (attempt {attempt + 1}): {e}")
        #        time.sleep(1)  # Wait before retrying
        #else:
        #    self.get_logger().warn("Exceeded maximum retries to publish initial pose.")
        
        if not self.wait_for_amcl_localization():
            return

    def wait_for_amcl_localization(self, timeout_sec=5.0):
        start_time = self.get_clock().now()
        while rclpy.ok() and not self.amcl_pose_received_:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed_time > timeout_sec:
                self.get_logger().warn("AMCL did not publish a pose within the timeout.")
                return False
        self.get_logger().info("AMCL is localized.")
        if (self.amcl_cov0 < 0.25 and self.amcl_cov6 < 0.25 and self.amcl_cov35 < 0.25):
            self.get_logger().info("Within all covariance requirements!")
            return True
        else:
            self.get_logger().info("Outside the covariance requirements!")
            return False

    def reinitialize_global_localization(self):
        self.get_logger().info("Calling /reinitialize_global_localization service...")
    
        # Wait for the service to be available before sending the request
        if not self.reinit_client_.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service /reinitialize_global_localization not available!")
            return False

        request = Empty.Request()
        future = self.reinit_client_.call_async(request)
    
        # Increase timeout and ensure proper spinning
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        except Exception as e:
            self.get_logger().error(f"Error during spin: {e}")
            return False

        if future.done():
            if future.result() is not None:
                self.get_logger().info("Successfully reinitialized global localization")
                return True
            else:
                self.get_logger().error("Service call failed with no result")
                return False
        else:
            self.get_logger().error("Service call timed out")
            return False

    def rotate_robot(self, rotation_duration=4.0, angular_velocity=0.5):
        self.get_logger().info(f"Rotating robot to assist localization...{angular_velocity}")
        twist_msg = Twist()
        twist_msg.angular.z = float(angular_velocity)
        start_time = self.get_clock().now()

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > rotation_duration:
                break
            self.cmd_vel_publisher_.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            #if self.amcl_pose_received_:
            #    if (self.amcl_cov0 < 0.25 and self.amcl_cov1 < 0.25 and self.amcl_cov6 < 0.25):
            #        break

        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher_.publish(twist_msg)
        self.get_logger().info("Finished rotating robot.")

    def move_robot(self, duration=5.0, linear_velocity=0.5):
        self.get_logger().info(f"Moving robot to assist localization...{linear_velocity}")
        twist_msg = Twist()
        twist_msg.linear.x = float(linear_velocity)
        start_time = self.get_clock().now()

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > duration:
                break
            self.cmd_vel_publisher_.publish(twist_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
            #if self.amcl_pose_received_:
            #    if (self.amcl_cov0 < 0.25 and self.amcl_cov1 < 0.25 and self.amcl_cov6 < 0.25):
            #        break

        twist_msg.linear.x = 0.0
        self.cmd_vel_publisher_.publish(twist_msg)
        self.get_logger().info("Finished moving robot.")

    def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available!")
            return False

        self.goal_reached_ = False

        # Cancel existing goals before sending new ones
        #if self._goal_handle:
        #    self._goal_handle.cancel_goal_async()

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

        time.sleep(5)

        self._send_goal_future = self._action_client.send_goal_async(
                goal_msg, 
                feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

        # Wait for the result - Not working below!!!
        #rclpy.spin_until_future_complete(self, self._send_goal_future)
        # Do not block here; allow the rest of the program to run
        return True
 
    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            self.goal_accepted = False
            return
        self.get_logger().info("Goal accepted.")
        # Request result future and bind get_result_callback
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result is None:
            self.get_logger().error("Result is None!")
            return

        # Update the goal handle
        self._goal_handle = future.result()  # Store the goal handle

        status = result.status
        status_str = self._status_to_string(status)
        self.get_logger().info(f"Navigation finished. Status: {status_str}")
    
        if status == GoalStatus.STATUS_SUCCEEDED:
            if not self.goal_reached_:
                # First goal completion (attach shelf)
                self.goal_reached_ = True
                time.sleep(1)
        else:
            self.goal_reached_ = False
            self.get_logger().info("Goal status does not return SUCCEEDED.")
 
    def handle_footprint_update(self):  
        success = self.update_footprint()
        #success = True  # Testing purpose only now!!!
        if success:
            self.footprint_updated = True
            self.move_robot(linear_velocity=-0.21)
            time.sleep(2)
            self.rotate_robot(angular_velocity=-0.393)
            time.sleep(2)
        else:
            self.get_logger().info("Handle footprint update Failed.")

    def handle_cart_dropoff(self):  
        self.move_robot(linear_velocity=-0.41)
        elevator_msg = String()
        elevator_msg.data = ""  # Empty message?!
        self.elevator_pub_.publish(elevator_msg)
        self.get_logger().info("Published to /elevator_down")
        time.sleep(0.5)  # Ensure message is sent
        success = self.return_footprint()
        if success:
            self.footprint_updated = False
            self.move_robot(linear_velocity=0.41)
        else:
            self.get_logger().info("Handle footprint back to original Failed.")

    def clean_shutdown(self):
        self.destroy_node()
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f"Navigation Feedback:\n"
            f"Distance: {feedback.distance_remaining:.2f}m\n"
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
        self.get_logger().info("attach_shelf() In.")
        
#        if not self.goal_reached_:
#            self.get_logger().info("attach_shelf called before goal was reached!")
#            return

        while not self.approach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /approach_shelf service...")
        
        req = GoToLoading.Request()
        req.attach_to_shelf = True
        
        future = self.approach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().complete

def main(args=None):
    rclpy.init(args=args)
    action_client = MoveShelfToShip()

    # Approach #1 to find initial position
    #action_client.find_spots('init_pos')
    #action_client.set_init_pose()
    #action_client.wait_for_amcl_localization()

    # Approach #2 to find initial position

    #if action_client.reinitialize_global_localization():
    #    while not action_client.wait_for_amcl_localization():
    #        action_client.rotate_robot(angular_velocity=0.5)
    #        action_client.rotate_robot(angular_velocity=-0.5)
    #        action_client.wait_for_amcl_localization()
    #        action_client.move_robot(linear_velocity=0.3)
    #        action_client.move_robot(linear_velocity=-0.3)
    #        action_client.rotate_robot(angular_velocity=-0.5)
    #        action_client.rotate_robot(angular_velocity=0.5)
    
    #action_client.find_spots('turn_pt')
    #action_client.send_goal()

    #while rclpy.ok() and not action_client.goal_reached_:
    # Process ROS events to update goal status
    #    rclpy.spin_once(action_client, timeout_sec=0.1)

    ##time.sleep(2.0)

    ##action_client.rotate_robot(angular_velocity=-0.405)
    #time.sleep(2.0)

    #action_client.attach_shelf()
    #time.sleep(2.0)  # Ensure message is sent

    #action_client.handle_footprint_update()
    #time.sleep(2.0)  # Ensure message is sent

    action_client.find_spots('ship_pt')
    action_client.send_goal()

    while rclpy.ok() and not action_client.goal_reached_:
        # Process ROS events to update goal status
        rclpy.spin_once(action_client, timeout_sec=0.1)

    action_client.rotate_robot(angular_velocity=0.404)
    time.sleep(2.0)  # Ensure message is sent

    #action_client.handle_cart_dropoff()
    time.sleep(2.0)  # Ensure message is sent

    action_client.find_spots('init_pos')
    action_client.send_goal()

    while rclpy.ok() and not action_client.goal_reached_:
        # Process ROS events to update goal status
        rclpy.spin_once(action_client, timeout_sec=0.1)

    action_client.get_logger().info("All Tasks Completed!")

    #action_client.rotate_robot(angular_velocity=-0.786)

    #rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()



 