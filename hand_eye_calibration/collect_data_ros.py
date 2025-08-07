#!/usr/bin/env python
# coding=utf-8
import rospy
import json
import logging
import os
import socket
import time
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from aiui.srv import FeedbackService, FeedbackServiceRequest, FeedbackServiceResponse


# 创建日志
class CustomLog:
    def __init__(self, logger_name):
        self.logger = logging.getLogger(logger_name)
        self.logger.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        
        # 文件处理器
        current_date = time.strftime("%Y%m%d")
        logs_dir = os.path.join(os.getcwd(), 'logs')
        os.makedirs(logs_dir, exist_ok=True)
        log_file = os.path.join(logs_dir, f'{current_date}.log')
        file_handler = logging.FileHandler(log_file)
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)
        
        # 控制台处理器
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)
    
    def info(self, msg):
        self.logger.info(msg)
    
    def error(self, msg):
        self.logger.error(msg)
    
    def debug(self, msg):
        self.logger.debug(msg)

# 日志实例
logger = CustomLog(__name__)

# 建立存储目录
def create_folder_with_date():
    """创建以日期命名的文件夹"""
    current_date = time.strftime("%Y%m%d")
    folder_name = os.path.join(os.getcwd(), 'calib_data', current_date)
    os.makedirs(folder_name, exist_ok=True)
    return folder_name

class HandEyeCalibrationNode:
    def __init__(self):
        rospy.init_node('hand_eye_calibration_node', anonymous=True)
        self.bridge = CvBridge()
        self.cam0_origin_path = create_folder_with_date()
        self.count = 1
        
        # 初始化相机
        rospy.loginfo("订阅相机图像话题 /camera/color/image_raw")
        self.cv_image = None
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        
        # 连接机械臂
        # self.robot_ip = rospy.get_param('~robot_ip', '192.168.1.100')
        # self.client = self.connect_to_robot(self.robot_ip)
        
        # 创建OpenCV窗口
        cv2.namedWindow("Capture_Video", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Capture_Video", 1280, 960)
        
        # 主循环
        self.main_loop()
    
    def quaternion_to_euler(self, w, x, y, z):
        """
        将单位四元数转换为欧拉角（Roll, Pitch, Yaw）
        旋转顺序：Z-Y-X（先Yaw，再Pitch，最后Roll）
        
        参数:
            w, x, y, z: 单位四元数的四个分量
        
        返回:
            roll, pitch, yaw: 欧拉角（弧度）
        """
        # 确保四元数已归一化
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        w, x, y, z = w/norm, x/norm, y/norm, z/norm
        
        # 计算俯仰角Pitch（y轴旋转）
        sin_p = 2.0 * (w*y - z*x)
        # 避免数值误差导致的超出[-1,1]范围
        sin_p = np.clip(sin_p, -1.0, 1.0)
        pitch = np.arcsin(sin_p)
        
        # 计算偏航角Yaw（z轴旋转）和滚转角Roll（x轴旋转）
        sin_r_cosp = 2.0 * (w*x + y*z)
        cos_r_cosp = 1.0 - 2.0 * (x*x + y*y)
        roll = np.arctan2(sin_r_cosp, cos_r_cosp)
        
        sin_y_cosp = 2.0 * (w*z + x*y)
        cos_y_cosp = 1.0 - 2.0 * (y*y + z*z)
        yaw = np.arctan2(sin_y_cosp, cos_y_cosp)
        
        return roll, pitch, yaw
        
    def connect_to_robot(self, ip):
        """连接机械臂并设置工作坐标系"""
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((ip, 8080))
            socket_command = '{"command":"set_change_work_frame","frame_name":"Base"}'
            self.send_cmd(client, socket_command, get_pose=False)
            rospy.loginfo(f"成功连接到机械臂: {ip}")
            return client
        except Exception as e:
            logger.error(f"连接机械臂失败: {str(e)}")
            rospy.signal_shutdown("无法连接机械臂")
            return None
    
    def send_cmd(self, client, cmd, get_pose=True):
        """发送命令到机械臂并处理响应"""
        try:
            client.send(cmd.encode('utf-8'))
            
            if not get_pose:
                response = client.recv(1024).decode('utf-8')
                logger.info(f"response:{response}")
                return True

            time.sleep(0.1)
            response = client.recv(4096).decode('utf-8')
            logger.info(f'response:{response}')

            # 解析响应数据...
            # 此处保持原代码中的JSON解析逻辑不变
            # 因长度限制，实际实现需包含完整的解析逻辑
            
            # 伪代码示例：
            # data = json.loads(response)
            # 提取pose数据并转换单位
            pose_converted = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # 示例数据
            return True, pose_converted
            
        except Exception as e:
            logger.error(f"处理命令时出错: {str(e)}")
            return False, str(e)
    
    def image_callback(self, msg):
        """相机图像回调函数"""
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            logger.error(f"图像转换失败: {str(e)}")
    
    def main_loop(self):
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                # 显示图像
                cv_img = cv2.resize(self.cv_image, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_AREA)
                cv2.imshow("Capture_Video", cv_img)
                
                # 处理键盘事件
                key = cv2.waitKey(1) & 0xFF
                if key == ord('s'):
                    # 保存图像和位姿
                    # success, pose = self.send_cmd(self.client, '{"command": "get_current_arm_state"}')
                    # TODO: ROS节点通信获取机械臂状态
                    arm_client = rospy.ServiceProxy("/aris_node/feedback_srv", FeedbackService)
                    req = FeedbackServiceRequest()
                    req.request = ''
                    arm_client.wait_for_service()
                    response = arm_client.call(req)
                    pose = response.ee_pq_right
                    success = response.response
                    # [roll, pitch, yaw] = cv2.Rodrigues(np.array([pose[3], pose[4], pose[5]]))[0].flatten()
                    [roll , pitch, yaw] = self.quaternion_to_euler(pose[3], pose[4], pose[5], pose[6])
                    [x, y, z, rx, ry, rz] = [pose[0], pose[1], pose[2], roll, pitch, yaw]
                    pose_converted = [x, y, z, rx, ry, rz]
                    rospy.loginfo(f"机械臂当前位姿：{pose_converted}")
                    
                    if success == '':
                        self.save_data(pose_converted, cv_img)
                        self.count += 1
                
                # 退出检测
                if key == 27:  # ESC键
                    rospy.signal_shutdown("用户退出")
            rate.sleep()
        
        # 清理资源
        cv2.destroyAllWindows()
        # if self.client:
        #     self.client.close()
    
    def save_data(self, pose, image):
        """保存位姿数据和图像"""
        # 保存位姿
        filename = os.path.join(self.cam0_origin_path, "poses.txt")
        pose_str = ",".join([str(x) for x in pose])
        
        with open(filename, 'a') as f:
            f.write(f"{pose_str}\n")
        
        # 保存图像
        img_path = os.path.join(self.cam0_origin_path, f"{self.count}.jpg")
        cv2.imwrite(img_path, image)
        rospy.loginfo(f"采集第{self.count}组数据 - 位姿: {pose_str}")

if __name__ == '__main__':
    try:
        HandEyeCalibrationNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass