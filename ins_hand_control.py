import rosbag
import rospy
from std_msgs.msg import String
import numpy as np
from hand_retarget import HandRetarget
from communication import HandCommunication
import time

# 打开 rosbag 文件
# bag = rosbag.Bag('twist_the_tupe.bag')
bag = rosbag.Bag('take_items.bag')



def point_to_numpy(point):
    return np.array([point.x, point.y, point.z])

def get_joint_positions(msg):
    left_hand_position = [
    point_to_numpy(msg.position_body[22]),  # 左手
    point_to_numpy(msg.position_lHand[1]),  # 左手拇指指根
    point_to_numpy(msg.position_lHand[2]),  # 左手拇指第一关节
    point_to_numpy(msg.position_lHand[3]),  # 左手拇指指尖
    point_to_numpy(msg.position_lHand[4]),  # 左手食指指根
    point_to_numpy(msg.position_lHand[5]),  # 左手食指第一关节
    point_to_numpy(msg.position_lHand[6]),  # 左手食指第二关节
    point_to_numpy(msg.position_lHand[7]),  # 左手食指指尖
    point_to_numpy(msg.position_lHand[8]),  # 左手中指指根
    point_to_numpy(msg.position_lHand[9]),  # 左手中指第一关节
    point_to_numpy(msg.position_lHand[10]),  # 左手中指第二关节
    point_to_numpy(msg.position_lHand[11]),  # 左手中指指尖
    point_to_numpy(msg.position_lHand[12]),  # 左手无名指指根
    point_to_numpy(msg.position_lHand[13]),  # 左手无名指第一关节
    point_to_numpy(msg.position_lHand[14]),  # 左手无名指第二关节
    point_to_numpy(msg.position_lHand[15]),  # 左手无名指指尖
    point_to_numpy(msg.position_lHand[16]),  # 左手小指指根
    point_to_numpy(msg.position_lHand[17]),  # 左手小指第一关节
    point_to_numpy(msg.position_lHand[18]),  # 左手小指第二关节
    point_to_numpy(msg.position_lHand[19]),  # 左手小指指尖
    ]
    
    rel_left_hand_position = left_hand_position - point_to_numpy(msg.position_body[22])

    right_hand_position = [
    point_to_numpy(msg.position_body[18]),  # 右手
    point_to_numpy(msg.position_rHand[1]),  # 右手拇指指根
    point_to_numpy(msg.position_rHand[2]),  # 右手拇指第一关节
    point_to_numpy(msg.position_rHand[3]),  # 右手拇指指尖
    point_to_numpy(msg.position_rHand[4]),  # 右手食指指根
    point_to_numpy(msg.position_rHand[5]),  # 右手食指第一关节
    point_to_numpy(msg.position_rHand[6]),  # 右手食指第二关节
    point_to_numpy(msg.position_rHand[7]),  # 右手食指指尖
    point_to_numpy(msg.position_rHand[8]),  # 右手中指指根
    point_to_numpy(msg.position_rHand[9]),  # 右手中指第一关节
    point_to_numpy(msg.position_rHand[10]),  # 右手中指第二关节
    point_to_numpy(msg.position_rHand[11]),  # 右手中指指尖
    point_to_numpy(msg.position_rHand[12]),  # 右手无名指指根
    point_to_numpy(msg.position_rHand[13]),  # 右手无名指第一关节
    point_to_numpy(msg.position_rHand[14]),  # 右手无名指第二关节
    point_to_numpy(msg.position_rHand[15]),  # 右手无名指指尖
    point_to_numpy(msg.position_rHand[16]),  # 右手小指指根
    point_to_numpy(msg.position_rHand[17]),  # 右手小指第一关节
    point_to_numpy(msg.position_rHand[18]),  # 右手小指第二关节
    point_to_numpy(msg.position_rHand[19]),  # 右手小指指尖
    ]
    
    rel_right_hand_position = right_hand_position - point_to_numpy(msg.position_body[18])
    
    relevent_position = (left_hand_position,right_hand_position)
    return relevent_position


HR = HandRetarget()
HC = HandCommunication()

# 遍历 bag 中的消息
for topic, msg, t in bag.read_messages():
    # 打印消息信息
    relevent_position =  get_joint_positions(msg)
    left_hand_angle,right_hand_angle = HR.solve_fingers_angles(relevent_position)
    
    print(left_hand_angle)
    HC.send_single_hand_cmd(left_hand_angle)

    # break
    # 可以根据需要处理其他类型的消息
    time.sleep(0.015)
# 关闭 bag 文件
bag.close()