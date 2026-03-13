#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from ament_index_python.packages import get_package_share_directory
import yaml
import math
import os
import tf2_ros
from geometry_msgs.msg import Pose, TransformStamped
import tf_transformations


class IKRosPath(Node):
    def __init__(self):
        super().__init__('ik_ros_path_node')
        self.client = self.create_client(GetPositionIK, '/compute_ik')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # --- DYNAMIC PATH LOGIC ---
        # This finds the path to your package automatically
        package_name = 'manipulator_tutorial'
        try:
            package_share = get_package_share_directory(package_name)
            self.yaml_path = os.path.join(package_share, 'config', 'request.yaml')
        except Exception as e:
            self.get_logger().error(f"Could not find package {package_name}: {e}")
            self.yaml_path = None

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik...')

    def run_ik(self):
        if not self.yaml_path or not os.path.exists(self.yaml_path):
            self.get_logger().error(f"YAML file not found at: {self.yaml_path}")
            return None

        with open(self.yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        request = GetPositionIK.Request()
        ik_req = data['ik_request']
        
        # Target Pose
        request.ik_request.group_name = ik_req['group_name']
        request.ik_request.pose_stamped.header.frame_id = ik_req['pose_stamped']['header']['frame_id']
        p = ik_req['pose_stamped']['pose']

        pose = Pose()
        pose.position.x = float(p['position']['x'])
        pose.position.y = float(p['position']['y'])
        pose.position.z = float(p['position']['z'])+0.107

        original_quat = [
            float(p['orientation']['x']),
            float(p['orientation']['y']),
            float(p['orientation']['z']),
            float(p['orientation']['w'])
        ]

        # Retrived from ros2 run tf2_ros tf2_echo panda_link8 panda_hand
        inverse_rotation = [0.0, 0.0, -0.383, 0.924] 

        new_quat = tf_transformations.quaternion_multiply(inverse_rotation, original_quat)
        pose.orientation.x = new_quat[0]
        pose.orientation.y = new_quat[1]
        pose.orientation.z = new_quat[2]
        pose.orientation.w = new_quat[3]

        request.ik_request.pose_stamped.pose = pose

        # Call Service
        self.get_logger().info(f"Processing IK from: {self.yaml_path}")
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main():
    rclpy.init()
    node = IKRosPath()
    result = node.run_ik()

    if result and result.error_code.val == 1:
        print("\n✅ IK SOLUTION FOUND (DEGREES):")
        print("-" * 30)
        for name, rad in zip(result.solution.joint_state.name, result.solution.joint_state.position):
            if "panda_joint" in name:
                print(f"{name:<15}: {math.degrees(rad):>8.2f}°")
    else:
        print(f"\n❌ Failed. Error code: {result.error_code.val if result else 'No response'}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()