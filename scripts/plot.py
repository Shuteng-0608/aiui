#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
from datetime import datetime
from arm_teleop.msg import DualArmMovej, DualHandTele

class DataRecorder:
    def __init__(self):
        rospy.init_node('teleop_data_recorder', anonymous=True)
        
        # 生成带时间戳的文件名
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.arm_csv_file = "arm_data_{}.csv".format(timestamp)
        self.hand_csv_file = "hand_data_{}.csv".format(timestamp)
        
        # 打开文件
        self.arm_file = open(self.arm_csv_file, 'w') # 如果是 Python 3 请使用: open(..., 'w', newline='')
        self.hand_file = open(self.hand_csv_file, 'w')
        
        self.arm_writer = None
        self.hand_writer = None
        
        # 订阅话题
        rospy.Subscriber('/arm_teleop/dual_arm_movej', DualArmMovej, self.arm_callback)
        rospy.Subscriber('/arm_teleop/dual_hand_tele', DualHandTele, self.hand_callback)
        
        rospy.loginfo("开始监听数据并录制...")
        rospy.loginfo("双臂数据将保存至: %s", self.arm_csv_file)
        rospy.loginfo("双手数据将保存至: %s", self.hand_csv_file)

    def arm_callback(self, msg):
        """
        精确解析 DualArmMovej 并将 7 自由度数组展开为独立列
        """
        # 提取基础数据和头部数据
        data = {
            'ros_time': msg.header.stamp.to_sec(), # 使用消息自带的准确时间戳
            'sequence': msg.sequence,
            'head_z_rotation': msg.head_z_rotation,
            'left_arm_id': msg.left_arm.arm_id,
            'right_arm_id': msg.right_arm.arm_id
        }
        
        # 展开左臂 7 个关节角度
        for i in range(7):
            data['left_joint_{}'.format(i+1)] = msg.left_arm.arm_joints[i]
            
        # 展开右臂 7 个关节角度
        for i in range(7):
            data['right_joint_{}'.format(i+1)] = msg.right_arm.arm_joints[i]

        # 初始化 CSV 表头 (仅执行一次)
        if self.arm_writer is None:
            # 保证列的顺序固定
            fieldnames = ['ros_time', 'sequence', 'head_z_rotation', 'left_arm_id'] + \
                         ['left_joint_{}'.format(i+1) for i in range(7)] + \
                         ['right_arm_id'] + \
                         ['right_joint_{}'.format(i+1) for i in range(7)]
            
            self.arm_writer = csv.DictWriter(self.arm_file, fieldnames=fieldnames)
            self.arm_writer.writeheader()
            
        self.arm_writer.writerow(data)

    def hand_callback(self, msg):
        """
        双手数据暂且保留动态反射解析，直到提供具体结构
        """
        data = {'recv_time': rospy.get_time()}
        for slot in msg.__slots__:
            # 排除复杂头部信息防止 CSV 错乱
            if slot != 'header':
                val = getattr(msg, slot)
                data[slot] = str(val)
                
        if self.hand_writer is None:
            self.hand_writer = csv.DictWriter(self.hand_file, fieldnames=data.keys())
            self.hand_writer.writeheader()
            
        self.hand_writer.writerow(data)

    def close(self):
        if self.arm_file:
            self.arm_file.close()
        if self.hand_file:
            self.hand_file.close()
        rospy.loginfo("数据已安全保存并关闭文件。")

if __name__ == '__main__':
    recorder = DataRecorder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()