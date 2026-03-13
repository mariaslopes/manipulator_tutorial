#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
import math
import threading

class FKCalculator(Node):
    def __init__(self):
        super().__init__('fk_calculator_node')
        
        # Service client for FK
        self.client = self.create_client(GetPositionFK, '/compute_fk')
        
        # Subscriber to get current joint positions
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.current_joint_state = None
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_fk service...')

    def joint_state_callback(self, msg):
        # We only want to save the state if it contains the panda joints
        if 'panda_joint1' in msg.name:
            self.current_joint_state = msg

    def run_fk(self):
        # Wait until we have received at least one joint state message
        while self.current_joint_state is None:
            self.get_logger().info('Waiting for current joint states...')
            rclpy.spin_once(self, timeout_sec=1.0)

        request = GetPositionFK.Request()
        
        # 1. Setup the Header
        request.header.frame_id = "panda_link0" # The frame you want the coordinates in
        
        # 2. Specify which links you want the position for
        request.fk_link_names = ["panda_hand"]
        
        # 3. Use the current robot state as the input
        request.robot_state.joint_state = self.current_joint_state

        self.get_logger().info("Calculating FK for panda_hand based on current joint states...")
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = FKCalculator()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    
    result = node.run_fk()

    if result and result.error_code.val == 1:
        # The result returns a list of poses (one for each link requested in fk_link_names)
        pose_stamped = result.pose_stamped[0]
        pos = pose_stamped.pose.position
        ori = pose_stamped.pose.orientation

        print("\n✅ FORWARD KINEMATICS SOLUTION:")
        print("-" * 40)
        print(f"Reference Frame: {pose_stamped.header.frame_id}")
        print(f"Link:            panda_hand")
        print("-" * 40)
        print(f"Position (meters):")
        print(f"  X: {pos.x:.4f}")
        print(f"  Y: {pos.y:.4f}")
        print(f"  Z: {pos.z:.4f}")
        print(f"Orientation (quaternion):")
        print(f"  [x, y, z, w]: [{ori.x:.4f}, {ori.y:.4f}, {ori.z:.4f}, {ori.w:.4f}]")
        print("-" * 40)
    else:
        err = result.error_code.val if result else "No response"
        print(f"\n❌ Failed to calculate FK. Error code: {err}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()