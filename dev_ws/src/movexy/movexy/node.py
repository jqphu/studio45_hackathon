from time import sleep
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import pdb
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose, Point, Quaternion

def rotate_pose(pose: Pose, degrees: float, axis='x'):
    # Convert quaternion to euler
    euler = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
    euler = euler.as_euler('xyz', degrees=True) # convert to degrees

    # Increment angle
    if axis=='x':
        euler[0] += degrees
    elif axis=='y':
        euler[1] += degrees
    elif axis=='z':
        euler[2] += degrees

    # Convert euler back to quaternion
    q = R.from_euler('xyz', euler, degrees=True).as_quat()

    # Update pose orientation
    pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    return pose


class SusanMoveRelPublisher(Node):

    def __init__(self):
        super().__init__('movexy')
        self.publisher_ = self.create_publisher(Pose, '/susan_move_rel', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.tfBuffer = Buffer()
        self.tfListener = TransformListener(self.tfBuffer, self)

    def get_link6_pose(self):
        try:
            transform = self.tfBuffer.lookup_transform('link_base', 'link5', rclpy.time.Time())
            return transform.transform
        except Exception as e:
            self.get_logger().info(f"Could not find transform to link5. {str(e)}")
            return None

    def publish_relxy(self, curr_transform):
        ct = curr_transform
        msg = Pose()
        rx, ry = map(float, input("Enter x and y: ").split()) #Giving input while running the node
        msg.position.x = ct.translation.x + rx / 10
        msg.position.y = ct.translation.y + ry / 10
        msg.position.z = ct.translation.z
        msg.orientation.x = ct.rotation.x
        msg.orientation.y = ct.rotation.y
        msg.orientation.z = ct.rotation.z
        msg.orientation.w = ct.rotation.w
        # msg.orientation.x = 0.0
        # msg.orientation.y = 0.0
        # msg.orientation.z = 0.0
        # msg.orientation.w = 1.0

        msg = rotate_pose(msg, 90, "y")

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "{}"'.format(msg))

def main(args=None):
    rclpy.init(args=args)

    node = SusanMoveRelPublisher()

    try:
        while rclpy.ok():
            # rclpy.spin(susan_move_rel_publisher)
            curr_pose = node.get_link6_pose()
            # curr_pose = None
            if curr_pose:
                node.get_logger().info('Current pose: {}'.format(curr_pose))
                node.publish_relxy(curr_pose)
            sleep(1)

            rclpy.spin_once(node)
    except Exception:
        # pdb.post_mortem()
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()