#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import math
from rclpy import qos

class SceneToTF(Node):
    def __init__(self):
        super().__init__('scene_to_tf_node')
        self.br = TransformBroadcaster(self)
        
        # Storage for the latest known poses of collision objects
        self.stored_objects = {}

        self.sub = self.create_subscription(
            PlanningScene, 
            '/monitored_planning_scene', 
            self.cb, 
            10
        )

        # Create a timer to publish TFs at 30Hz regardless of incoming messages
        self.timer = self.create_timer(1.0 / 30.0, self.broadcast_stored_tfs)

        self.get_logger().info('TF Bridge Node started. Continuous broadcasting at 30Hz...')

    def cb(self, msg):
        # Update stored world collision objects (box_left, box_right)
        for obj in msg.world.collision_objects:
            self.stored_objects[obj.id] = (obj.header.frame_id, obj.pose)
        
        # Update stored attached objects
        for attached in msg.robot_state.attached_collision_objects:
            self.stored_objects[attached.object.id] = (attached.link_name, attached.object.pose)

    def broadcast_stored_tfs(self):
        # Iterate through all known objects and broadcast their latest pose
        for child_id, (parent_id, pose) in self.stored_objects.items():
            self.publish_tf(parent_id, child_id, pose)

    def publish_tf(self, parent, child, pose):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child

        original_rot = R.from_quat([
            pose.orientation.x, 
            pose.orientation.y, 
            pose.orientation.z, 
            pose.orientation.w
        ])
        extra_rot = R.from_euler('x', math.pi)
        final_rot = original_rot * extra_rot
        new_quat = final_rot.as_quat()
        
        # Handle position
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        
        # Handle orientation
        t.transform.rotation.x = new_quat[0]
        t.transform.rotation.y = new_quat[1]
        t.transform.rotation.z = new_quat[2]
        t.transform.rotation.w = new_quat[3]

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = SceneToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()