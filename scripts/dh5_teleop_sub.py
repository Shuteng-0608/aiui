#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from arm_teleop.msg import DualHandTele
from aiui.srv import DH5SetPosition, DH5SetPositionRequest

class DH5TeleopSubscriber:
    def __init__(self):

        rospy.init_node('dual_hand_tele_listener', anonymous=True)
        rospy.wait_for_service('/dh5/set_all_position')
        self.dh5_service = rospy.ServiceProxy('/dh5/set_all_position', DH5SetPosition)
        
        # 使用 queue_size=1 避免积压
        self.dh5_subscriber = rospy.Subscriber("/arm_teleop/dual_hand_tele", DualHandTele, self.callback, queue_size=1)
        rospy.loginfo("正在监听 /arm_teleop/dual_hand_tele 话题...")
        
        self.dh5_req = DH5SetPositionRequest()
        self.dh5_req.hand_type = 'both'
        self.dh5_req.hand_mode = 'hand'

        self.latest_msg = None
        # 30Hz 控制循环 (0.033s)，将发送频率与接收频率解耦
        self.timer = rospy.Timer(rospy.Duration(0.033), self.control_loop)


    def callback(self, data):
        """
        回调函数，当接收到 DualHandTele 消息时触发
        只负责更新数据，不进行耗时操作
        """
        self.latest_msg = data

    def control_loop(self, event):
        """
        定时控制循环，负责调用服务
        """
        if self.latest_msg is None:
            return

        try: 
            self.dh5_req.right_position_list = self.latest_msg.right_position_list
            self.dh5_req.left_position_list = self.latest_msg.left_position_list
            self.dh5_service.call(self.dh5_req)
            
        except rospy.ServiceException as e:
            rospy.logerr_throttle(1.0, f"调用灵巧手服务失败: {e}")

# def listener():
#     # 初始化节点，命名为 dual_hand_tele_listener
#     rospy.init_node('dual_hand_tele_listener', anonymous=True)
#     rospy.wait_for_service('/dh5/set_all_position')
#     dh5_service = rospy.ServiceProxy('/dh5/set_all_position', DH5SetPosition)

#     # 订阅话题 /arm_teleop/dual_hand_tele
#     # 消息类型: DualHandTele
#     # 回调函数: callback
#     rospy.Subscriber("/arm_teleop/dual_hand_tele", DualHandTele, callback)

#     rospy.loginfo("正在监听 /arm_teleop/dual_hand_tele 话题...")

#     # 保持节点运行，直到节点被关闭
#     rospy.spin()

if __name__ == '__main__':
    try:
        dh5_sub = DH5TeleopSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass