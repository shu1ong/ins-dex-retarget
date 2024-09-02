import rospy
from BoneFrame import SkeletonVisualizer
from virdyn.msg import vdmsg
from geometry_msgs.msg import Point, Quaternion

class SkeletonSubscriber:
    def __init__(self):
        self.visualizer = SkeletonVisualizer()
        rospy.Subscriber("/vdmsg", vdmsg, self.vdmsg_callback)

    def vdmsg_callback(self, msg):
        left_hand_pos = msg.position_lHand[0]
        left_hand_quat = msg.quaternion_lHand[0]

        joint_positions = {
            'left_hand': Point(0, 0, 0),  # Root node at origin
            'left_HN_ThumbFinger': msg.position_lHand[1],
            'left_HN_ThumbFinger1': msg.position_lHand[2],
            'left_HN_ThumbFinger2': msg.position_lHand[3],
            'left_HN_IndexFinger': msg.position_lHand[4],
            'left_HN_IndexFinger1': msg.position_lHand[5],
            'left_HN_IndexFinger2': msg.position_lHand[6],
            'left_HN_IndexFinger3': msg.position_lHand[7],
            'left_HN_MiddleFinger': msg.position_lHand[8],
            'left_HN_MiddleFinger1': msg.position_lHand[9],
            'left_HN_MiddleFinger2': msg.position_lHand[10],
            'left_HN_MiddleFinger3': msg.position_lHand[11],
            'left_HN_RingFinger': msg.position_lHand[12],
            'left_HN_RingFinger1': msg.position_lHand[13],
            'left_HN_RingFinger2': msg.position_lHand[14],
            'left_HN_RingFinger3': msg.position_lHand[15],
            'left_HN_PinkyFinger': msg.position_lHand[16],
            'left_HN_PinkyFinger1': msg.position_lHand[17],
            'left_HN_PinkyFinger2': msg.position_lHand[18],
            'left_HN_PinkyFinger3': msg.position_lHand[19],
        }
        
        joint_orientations = {
            'left_hand': Quaternion(0, 0, 0, 1),  # Identity quaternion for root
            'left_HN_ThumbFinger': msg.quaternion_lHand[1],
            'left_HN_ThumbFinger1': msg.quaternion_lHand[2],
            'left_HN_ThumbFinger2': msg.quaternion_lHand[3],
            'left_HN_IndexFinger': msg.quaternion_lHand[4],
            'left_HN_IndexFinger1': msg.quaternion_lHand[5],
            'left_HN_IndexFinger2': msg.quaternion_lHand[6],
            'left_HN_IndexFinger3': msg.quaternion_lHand[7],
            'left_HN_MiddleFinger': msg.quaternion_lHand[8],
            'left_HN_MiddleFinger1': msg.quaternion_lHand[9],
            'left_HN_MiddleFinger2': msg.quaternion_lHand[10],
            'left_HN_MiddleFinger3': msg.quaternion_lHand[11],
            'left_HN_RingFinger': msg.quaternion_lHand[12],
            'left_HN_RingFinger1': msg.quaternion_lHand[13],
            'left_HN_RingFinger2': msg.quaternion_lHand[14],
            'left_HN_RingFinger3': msg.quaternion_lHand[15],
            'left_HN_PinkyFinger': msg.quaternion_lHand[16],
            'left_HN_PinkyFinger1': msg.quaternion_lHand[17],
            'left_HN_PinkyFinger2': msg.quaternion_lHand[18],
            'left_HN_PinkyFinger3': msg.quaternion_lHand[19],
        }

        # Calculate relative poses
        relative_positions = {}
        relative_orientations = {}
        for joint_name in joint_positions.keys():
            if joint_name != 'left_hand':
                rel_pos, rel_quat = self.visualizer.calculate_relative_pose(
                    left_hand_pos, left_hand_quat,
                    joint_positions[joint_name], joint_orientations[joint_name]
                )
                relative_positions[joint_name] = rel_pos
                relative_orientations[joint_name] = rel_quat
            else:
                relative_positions[joint_name] = Point(0, 0, 0)
                relative_orientations[joint_name] = Quaternion(0, 0, 0, 1)

        self.visualizer.visualize_skeleton(relative_positions, relative_orientations)

if __name__ == '__main__':
    subscriber = SkeletonSubscriber()
    rospy.spin()