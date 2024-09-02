import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped, Quaternion
import tf2_ros
from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion, quaternion_from_euler
import math

class SkeletonVisualizer:
    def __init__(self):
        rospy.init_node('skeleton_visualizer', anonymous=True)
        self.marker_pub = rospy.Publisher('skeleton_markers', MarkerArray, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.rate = rospy.Rate(50)  # 50Hz

        # Define left hand skeleton structure
        self.bones = [
            ("left_hand", "left_HN_ThumbFinger"),
            ("left_HN_ThumbFinger", "left_HN_ThumbFinger1"),
            ("left_HN_ThumbFinger1", "left_HN_ThumbFinger2"),
            
            ("left_hand", "left_HN_IndexFinger"),
            ("left_HN_IndexFinger", "left_HN_IndexFinger1"),
            ("left_HN_IndexFinger1", "left_HN_IndexFinger2"),
            ("left_HN_IndexFinger2", "left_HN_IndexFinger3"),
            
            ("left_hand", "left_HN_MiddleFinger"),
            ("left_HN_MiddleFinger", "left_HN_MiddleFinger1"),
            ("left_HN_MiddleFinger1", "left_HN_MiddleFinger2"),
            ("left_HN_MiddleFinger2", "left_HN_MiddleFinger3"),
            
            ("left_hand", "left_HN_RingFinger"),
            ("left_HN_RingFinger", "left_HN_RingFinger1"),
            ("left_HN_RingFinger1", "left_HN_RingFinger2"),
            ("left_HN_RingFinger2", "left_HN_RingFinger3"),
            
            ("left_hand", "left_HN_PinkyFinger"),
            ("left_HN_PinkyFinger", "left_HN_PinkyFinger1"),
            ("left_HN_PinkyFinger1", "left_HN_PinkyFinger2"),
            ("left_HN_PinkyFinger2", "left_HN_PinkyFinger3"),
        ]
        self.publish_static_transform()

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = rospy.Time.now()
        static_transform.header.frame_id = "map"
        static_transform.child_frame_id = "left_hand"
        static_transform.transform.translation.x = 0
        static_transform.transform.translation.y = 0
        static_transform.transform.translation.z = 0
        q = quaternion_from_euler(0, 0, 0)  # No rotation
        static_transform.transform.rotation.x = q[0]
        static_transform.transform.rotation.y = q[1]
        static_transform.transform.rotation.z = q[2]
        static_transform.transform.rotation.w = q[3]
        self.static_tf_broadcaster.sendTransform(static_transform)

    def create_joint_marker(self, joint_id, position):
        marker = Marker()
        marker.header.frame_id = "left_hand"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "joints"
        marker.id = joint_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def create_bone_marker(self, bone_id, start, end):
        marker = Marker()
        marker.header.frame_id = "left_hand"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "bones"
        marker.id = bone_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = [start, end]
        marker.scale.x = 0.005  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        return marker

    def broadcast_joint_tf(self, joint_name, position, orientation):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "left_hand"
        t.child_frame_id = f"joint_{joint_name}"
        t.transform.translation.x = position.x
        t.transform.translation.y = position.y
        t.transform.translation.z = position.z
        t.transform.rotation = orientation
        self.tf_broadcaster.sendTransform(t)

    def visualize_skeleton(self, joint_positions, joint_orientations):
        marker_array = MarkerArray()

        # Create joint markers and broadcast TF
        for i, (joint_name, pos) in enumerate(joint_positions.items()):
            joint_marker = self.create_joint_marker(i, pos)
            marker_array.markers.append(joint_marker)
            self.broadcast_joint_tf(joint_name, pos, joint_orientations[joint_name])

        # Create bone markers
        for i, (start_joint, end_joint) in enumerate(self.bones):
            if start_joint in joint_positions and end_joint in joint_positions:
                bone_marker = self.create_bone_marker(
                    i + len(joint_positions),
                    joint_positions[start_joint],
                    joint_positions[end_joint]
                )
                marker_array.markers.append(bone_marker)

        self.marker_pub.publish(marker_array)

    def calculate_relative_pose(self, root_pos, root_quat, joint_pos, joint_quat):
        # Calculate relative position
        rel_pos = Point(
            joint_pos.x - root_pos.x,
            joint_pos.y - root_pos.y,
            joint_pos.z - root_pos.z
        )

        # Calculate relative orientation
        inv_root_quat = quaternion_inverse([root_quat.x, root_quat.y, root_quat.z, root_quat.w])
        rel_quat = quaternion_multiply(inv_root_quat, [joint_quat.x, joint_quat.y, joint_quat.z, joint_quat.w])

        return rel_pos, Quaternion(*rel_quat)