#!/usr/bin/env python
# -*- coding:utf-8 -*-\
# export LD_PRELOAD=/lib/x86_64-linux-gnu/libtiff.so.5
import rospy
from aiui.srv import VLAProcess, VLAProcessResponse
from aiui.srv import FeedbackService, FeedbackServiceRequest
from aiui.srv import IkOptService, IkOptServiceRequest
from aiui.srv import TrajectoryService, TrajectoryServiceRequest
from aiui.srv import TTS, TTSRequest
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import requests
import base64
import ast
import cv2
import numpy as np
import traceback
from threading import Thread
from scipy.spatial.transform import Rotation as R
import numpy as np

class VLAServiceServer:
    def __init__(self):
        # 初始化CV桥接器
        self.bridge = CvBridge()
        # API配置
        self.api_url = rospy.get_param('~api_url', 'http://172.18.35.200:50002/process_image')
        # 初始化服务
        self.service = rospy.Service('vla_service', VLAProcess, self.handle_vla_request)
        self.intr = {
            'cx': 640.4387817382812,
            'cy': 357.5877990722656,
            'fx': 613.6777954101562,
            'fy': 613.6703491210938
        }
        self.tts_client = rospy.ServiceProxy('/tts_service/player', TTS)
        rospy.loginfo("VLA Service Server is ready")
        """
        K: [613.6777954101562, 0.0,                 640.4387817382812, 
            0.0,                613.6703491210938, 357.5877990722656, 
            0.0,                0.0,                1.0]

        # Intrinsic camera matrix for the raw (distorted) images.
        #     [fx  0 cx]
        # K = [ 0 fy cy]
        #     [ 0  0  1]
        """
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        将欧拉角(roll, pitch, yaw)转换为四元数(w, x, y, z)
        单位：弧度
        旋转顺序：Z-Y-X (先yaw，再pitch，最后roll)
        """
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return np.array([w, x, y, z])  
    
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
    def handle_vla_request(self, req):
        """仅在服务调用时订阅一帧图像"""
        try:
            # 阻塞等待直到收到一帧图像（超时5秒）
            rospy.loginfo("Waiting for image messages...")
            image_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)

            depth_msg = rospy.wait_for_message("/camera/depth/image_raw", Image, timeout=5.0)
            # 转换为OpenCV格式 (统一转为16位毫米单位)
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
            
            # 处理异常值（将无效值设为0）
            depth_array = np.array(cv_depth, dtype=np.float32)
            depth_array[np.isnan(depth_array)] = 0
            depth_array[np.isinf(depth_array)] = 0
            cv_depth = depth_array.astype(np.uint16)
            # 保存原始深度为PNG文件（保留完整深度信息）
            cv2.imwrite("/home/whc/aiui_ws/src/aiui/depth_image_16bit.png", cv_depth)
            cv2.imwrite("/home/whc/aiui_ws/src/aiui/color_image.jpg", self.bridge.imgmsg_to_cv2(image_msg, "bgr8"))
            
            


        except rospy.ROSException as e:
            error_msg = f"Failed to capture image: {str(e)}"
            rospy.logerr(error_msg)
            return VLAProcessResponse(error_msg)
        
        try:
            # 转换ROS图像为OpenCV格式并编码
            rospy.loginfo("Processing image and depth data...")
            # cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            # _, buffer = cv2.imencode('.jpg', cv_image)
            # encoded_image = base64.b64encode(buffer).decode('utf-8')
            image_msg = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            
            ret, jpeg_buffer = cv2.imencode('.jpg', image_msg, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            if ret:
                rgb_bytes = jpeg_buffer.tobytes()

            # cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, "z16")
            # cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, depth_msg.encoding)
            # _, buffer = cv2.imencode('.png', cv_depth)
            # encoded_depth = base64.b64encode(buffer).decode('utf-8')
            ret, png_buffer = cv2.imencode('.png', cv_depth, [int(cv2.IMWRITE_PNG_COMPRESSION), 3])
            if ret:
                png_bytes = png_buffer.tobytes()
        except Exception as e:
            error_msg = f"Image processing error: {str(e)}"
            rospy.logerr(error_msg)
            return VLAProcessResponse(error_msg)

        # 获取机械臂当前位姿
        # rospy.loginfo("Getting current arm pose...")
        # arm_client = rospy.ServiceProxy("/aris_node/feedback_srv", FeedbackService)
        # arm_req = FeedbackServiceRequest()
        # arm_req.request = ''
        # arm_client.wait_for_service()
        # response = arm_client.call(arm_req)
        # pose = response.ee_pq_right
        # [roll , pitch, yaw] = self.quaternion_to_euler(pose[3], pose[4], pose[5], pose[6])
        # [x, y, z, rx, ry, rz] = [pose[0], pose[1], pose[2], roll, pitch, yaw]
        # pose_converted = [x, y, z, rx, ry, rz]
        pose_converted = [0, 0, 0, 0, 0, 0]  # 初始化为零
        rospy.loginfo(f"机械臂当前位姿：{pose_converted}")
        # 包装数据并发送请求到VLA API
        files = {
            'rgb':rgb_bytes,
            'depth':png_bytes
        }

        text_data = {
            "ins": req.prompt,
            'pose': str(pose_converted),
            'intr': str(self.intr)
        }

        try:
            rospy.loginfo(f"Sending request to API: {self.api_url}")
            response = requests.post(self.api_url, files=files, data=text_data, timeout=30)
            rospy.loginfo(f"API Response Status: {response.status_code}")
            if response.ok:
                data = response.json()
                rospy.loginfo(f"API Response Data: {data}")
                # result = data.get("read_message", "Wrong Name for Getter Or No valid result")
                # result = data.get("message", "Wrong Name for Getter Or No valid result")
                # rospy.loginfo(result)
                # 安全解析返回结果
                try:
                    # 提取数据
                    target_x = data.get('target_x', 0.0)
                    target_y = data.get('target_y', 0.0)
                    target_z = data.get('target_z', 0.0)
                    target_rx = data.get('rotation_x')
                    target_ry = data.get('rotation_y')
                    target_rz = data.get('rotation_z')
                    rospy.loginfo(f"cam-目标位置: {target_x}, {target_y}, {target_z}, 旋转角度: {target_rx}, {target_ry}, {target_rz}")
                    if abs(target_x) + abs(target_y) + abs(target_z) < 0.01:
                        self.tts_text = "这个位置不太好抓哦，能不能换个位置试试？"
                        req = TTSRequest()
                        req.request = self.tts_text
                        self.tts_client.wait_for_service()
                        Thread(target=self.tts_client.call, args=(req,), daemon=True).start()
                        rospy.logwarn("目标位置过小，可能是无效数据，返回空响应")
                        return VLAProcessResponse("目标位置过小，可能是无效数据")
                    else:
                        self.tts_text = "拿稳喽，不要乱动哦！"
                        req = TTSRequest()
                        req.request = self.tts_text
                        self.tts_client.wait_for_service()
                        Thread(target=self.tts_client.call, args=(req,), daemon=True).start()

                    # 旋转矩阵
                    # target_r_incamera = R.from_euler('xyz', [target_rx, target_ry, target_rz], degrees=False).as_matrix()
                    
                    # 相机在机械臂基坐标系下的旋转矩阵
                    # [[ 0.04205621  0.0202342   0.99891033]
                    #  [-0.1034163  -0.99433648  0.0244956 ]    
                    #  [ 0.99374863 -0.1043338  -0.03972548]]
                    camera_rotation = np.array([[0.04205621, 0.0202342, 0.99891033],
                                                [-0.1034163, -0.99433648, 0.0244956],
                                                [0.99374863, -0.1043338, -0.03972548]])
                    print(camera_rotation)
                    
                    # 计算目标物体在机械臂基坐标系下的旋转矩阵
                    # target_r_inbase = np.dot(camera_rotation, target_r_incamera)

                    # 将旋转矩阵转换为四元数
                    # target_quaternion = R.from_matrix(target_r_inbase).as_quat(scalar_first=True)  # 确保四元数格式为(w, x, y, z)

                    # 计算目标物体在机械臂基坐标系下的位置
                    # 提取的目标位置变为列向量
                    target_position = np.array([[target_x], [target_y], [target_z]])
                    print(f"目标位置（列向量）: {target_position}")
                    # 旋转矩阵乘以目标位置
                    target_position_inbase = np.dot(camera_rotation, target_position) + np.array([ [0.08589616], [-0.16534466], [-0.10202788]]) - np.array([[0.05656], [0], [-0.1414]])
                    print(f"目标位置在机械臂基坐标系下: {target_position_inbase}")
                    act = data.get('action')
                    rospy.loginfo(f"arm-目标位置: {target_position_inbase[0].item()}, {target_position_inbase[1].item()}, {target_position_inbase[2].item()}, 旋转角度: {target_rx}, {target_ry}, {target_rz}, 动作: {act}")

                    # [w, x, y ,z] = self.euler_to_quaternion(target_rx, target_ry, target_rz)  # 确保转换正确
                    [w, x, y ,z] = [0, 0.9238915, 0, -0.3826545]
                    arm_data = [target_position_inbase[0].item(), target_position_inbase[1].item(), target_position_inbase[2].item(), w, x, y, z]
                    rospy.loginfo(f"机械臂目标数据: {arm_data}")
                    parsed_result = {
                        'target_x': target_position_inbase[0].item(),
                        'target_y': target_position_inbase[1].item(),
                        'target_z': target_position_inbase[2].item(),
                        'quaternion_w': w,
                        'quaternion_x': x,
                        'quaternion_y': y,
                        'quaternion_z': z,
                        'action': act
                    }
                    # rospy.loginfo(f"Parsed Result: {parsed_result}")
                    arm_client = rospy.ServiceProxy("/aris_node/traj_srv", TrajectoryService)
                    traj_req = TrajectoryServiceRequest()
                    traj_req.target_pq[0] = target_position_inbase[0].item()
                    traj_req.target_pq[1] = target_position_inbase[1].item()
                    traj_req.target_pq[2] = target_position_inbase[2].item()
                    traj_req.target_pq[3] = w
                    traj_req.target_pq[4] = x 
                    traj_req.target_pq[5] = y
                    traj_req.target_pq[6] = z
                    # ik_req.action = act
                    arm_client.wait_for_service()
                    try:
                        ik_response = arm_client.call(traj_req)
                        rospy.loginfo(f"IK Response: {ik_response}")
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Service call failed: {e}")
                        return VLAProcessResponse(f"Service call failed: {e}")
                    
                except:
                    # parsed_result = str(result)
                    rospy.logerr("Failed to parse API response or extract target position and rotation.")
                    traceback.print_exc()
                    pass
                return VLAProcessResponse(str(parsed_result))
            else:
                error_msg = f"API Error {response.status_code}: {response.text}"
                rospy.logerr(error_msg)
                return VLAProcessResponse(error_msg)
        except Exception as e:
            traceback.print_exc()
            error_msg = f"Network error: {str(e)}"
            rospy.logerr(error_msg)
            return VLAProcessResponse(error_msg)

if __name__ == "__main__":
    rospy.init_node('vla_service_server')
    server = VLAServiceServer()
    rospy.spin()