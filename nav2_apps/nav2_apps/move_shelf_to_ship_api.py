#! /usr/bin/env python3
import time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from attach_shelf.srv import GoToLoading
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from std_srvs.srv import Empty  # Import the Empty service
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import time

# Shelf and shipping positions (hardcoded as in move_shelf_to_ship_new.py)
shelf_positions = {
    "init_pos": [0.0, 0.0, 0.0, 1.0],
    "end_pos": [0.0, 0.0, 0.0, 1.0]
}

shipping_destinations = {
    "load_pos": [5.8, 0.0, -0.707, 0.707],
    "ship_pos": [2.3, 0.0, -0.707, 0.707]
}

class MoveShelfToShip(BasicNavigator):
    def __init__(self):
        super().__init__("move_shelf_to_ship")
        self.approach_client = self.create_client(GoToLoading, '/approach_shelf')
        self.gparam_client = None
        self.lparam_client = None
        #self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.amcl_pose_subscriber_ = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup())
        self.reinit_client_ = self.create_client(Empty, '/reinitialize_global_localization')
        self.cmd_vel_publisher_ = self.create_publisher(Twist, '/diffbot_base_controller/cmd_vel_unstamped', 10)

        self.amcl_pose_received_ = False
        self.goal_reached_ = False
        self.initial_amcl_pose_ = None
        self.amcl_pose_x = 0.0
        self.amcl_pose_y = 0.0
        self.amcl_or_z = 0.0
        self.amcl_or_w = 1.0
        self.reinit_x = 0.0
        self.reinit_y = 0.0
        self.reinit_oz = 0.0
        self.reinit_ow = 0.0

    def attach_shelf(self):
        """Call service to attach shelf mechanism"""
        #if not self.goal_reached_:
        #    self.get_logger().warn("attach_shelf called before goal was reached!")
        #    return
        while not self.approach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /approach_shelf service...")
        req = GoToLoading.Request(attach_to_shelf=True)
        future = self.approach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().complete

    def update_footprint(self):
        """Update costmap footprints after shelf attachment"""
        self.gparam_client = self.create_client(SetParameters, '/global_costmap/global_costmap/set_parameters')
        self.lparam_client = self.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')

        new_footprint = "[ [0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5] ]"
        req = SetParameters.Request(parameters=[
            Parameter(name="footprint", value=new_footprint).to_parameter_msg()
        ])

        # Update global costmap
        if not self.gparam_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Global costmap service unavailable")
            return False
        future = self.gparam_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        # Update local costmap
        if not self.lparam_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("Local costmap service unavailable")
            return False
        future = self.lparam_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        self.get_logger().info("Footprint updated successfully")
        return True

    def create_pose(self, position):
    #"""Helper to create PoseStamped messages"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.orientation.z = position[2]
        pose.pose.orientation.w = position[3]
        return pose

    def publish_initial_pose(self):
        """Publish the initial pose of the robot."""
        #initial_pose = PoseWithCovarianceStamped()
        initial_pose = PoseStamped()
    
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
    
        # Set the position and orientation
        initial_pose.pose.position.x = self.reinit_x 
        #shelf_positions["init_pos"][0]
        initial_pose.pose.position.y = self.reinit_y 
        #shelf_positions["init_pos"][1]
        initial_pose.pose.orientation.z = self.reinit_oz #shelf_positions["init_pos"][2]
        initial_pose.pose.orientation.w = self.reinit_ow #shelf_positions["init_pos"][3]
    
        # Set covariance (optional, can be set to zero)
        #initial_pose.pose.covariance = [0] * 36  # Example covariance
        #initial_pose.pose.covariance = [
        #        0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
        #        0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
        #        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #        0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        #    ]

        #while not self.initial_pose_pub.get_subscription_count():
        #    self.get_logger().info("Waiting for subscribers to connect...")
        #    time.sleep(1)

        # Publish the initial pose
        #for _ in range(3):
        #    self.initial_pose_pub.publish(initial_pose)
        #    time.sleep(0.5)
        self.setInitialPose(initial_pose)
        self.get_logger().info("Initial pose published after Reinitialize.")

    def amcl_pose_callback(self, msg):
        self.amcl_pose_received_ = True
        self.get_logger().info(f"Received pose from AMCL: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y
        self.amcl_or_z = msg.pose.pose.orientation.z
        self.amcl_or_w = msg.pose.pose.orientation.w
       
        self.amcl_cov0 = msg.pose.covariance[0]
        self.amcl_cov6 = msg.pose.covariance[6]
        self.amcl_cov35 = msg.pose.covariance[35]

    def wait_for_amcl_localization(self, timeout_sec=5.0):
        start_time = self.get_clock().now()
        while rclpy.ok() and not self.amcl_pose_received_:
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed_time > timeout_sec:
                self.get_logger().warn("AMCL did not publish a pose within the timeout.")
                return False
        self.get_logger().info("Check if AMCL is localized.")
        if (self.amcl_cov0 < 0.25 and self.amcl_cov6 < 0.25 and self.amcl_cov35 < 0.25):
            self.get_logger().info("Within all covariance requirements!")
            self.reinit_x = self.amcl_pose_x
            self.reinit_y = self.amcl_pose_y
            self.get_logger().info(f"reinit x: {self.reinit_x}, y: {self.reinit_y}")
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

def main():
    rclpy.init()
    navigator = MoveShelfToShip()
    # Wait for Nav2 activation
    navigator.waitUntilNav2Active()

    #navigator.publish_initial_pose()
    if navigator.reinitialize_global_localization():
        while not navigator.wait_for_amcl_localization():
            navigator.rotate_robot(angular_velocity=0.5)
            navigator.rotate_robot(angular_velocity=-0.5)
            navigator.wait_for_amcl_localization()
            navigator.move_robot(linear_velocity=0.5)
            navigator.move_robot(linear_velocity=-0.5)
            navigator.rotate_robot(angular_velocity=-0.5)
            navigator.rotate_robot(angular_velocity=0.5)

    navigator.publish_initial_pose()
  
    try:
        # Phase 1: Navigate to shelf
        move_pose = navigator.create_pose(shipping_destinations["load_pos"])
        navigator.goToPose(move_pose)
        
        while not navigator.isTaskComplete():
            pass  # Add optional feedback handling here
        
        if navigator.getResult() != TaskResult.SUCCEEDED:
            navigator.get_logger().error("Failed to reach shelf!")
            return

        # Phase 2: Attach shelf
        if not navigator.attach_shelf():
            navigator.get_logger().error("Shelf attachment failed")
            return

        # Phase 3: Update footprint
        if not navigator.update_footprint():
            navigator.get_logger().error("Footprint update failed")
            return

        # 2. Allow footprint to propagate
        time.sleep(2.0)

        # Phase 4: Navigate to shipping destination
        navigator.move_robot(linear_velocity=-0.23)
        navigator.rotate_robot(angular_velocity=-0.392699)  #Right turn 90 deg

        #if not navigator.wait_for_amcl_localization():
        #    raise RuntimeError("Localization failed")

        #navigator.publish_initial_pose()
  
        ship_pose = navigator.create_pose(shipping_destinations["ship_pos"])
        navigator.goToPose(ship_pose)
        
        while not navigator.isTaskComplete():
            pass
        
        if navigator.getResult() == TaskResult.SUCCEEDED:
            navigator.get_logger().info("Successfully delivered shelf!")
        else:
            navigator.get_logger().error("Failed to reach shipping destination")

    except Exception as e:
        navigator.get_logger().error(f"Critical error: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

