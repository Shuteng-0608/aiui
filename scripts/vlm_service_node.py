#!/usr/bin/env python
# -*- coding:utf-8 -*-\
# export LD_PRELOAD=/lib/x86_64-linux-gnu/libtiff.so.5
import rospy
from aiui.srv import VLMProcess, VLMProcessResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import requests
import base64
import ast
import cv2

class VLMServiceServer:
    def __init__(self):
        # 初始化CV桥接器
        self.bridge = CvBridge()
        # API配置
        self.api_url = rospy.get_param('~api_url', 'http://172.18.35.200:8000/uploads/vlm_queries')
        # 初始化服务
        self.service = rospy.Service('vlm_service', VLMProcess, self.handle_vlm_request)
        rospy.loginfo("VLM Service Server is ready")

    def handle_vlm_request(self, req):
        """仅在服务调用时订阅一帧图像"""
        try:
            # 阻塞等待直到收到一帧图像（超时5秒）
            image_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)
        except rospy.ROSException as e:
            error_msg = f"Failed to capture image: {str(e)}"
            rospy.logerr(error_msg)
            return VLMProcessResponse(error_msg)
        
        try:
            # 转换ROS图像为OpenCV格式并编码
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            _, buffer = cv2.imencode('.jpg', cv_image)
            encoded_image = base64.b64encode(buffer).decode('utf-8')
        except Exception as e:
            error_msg = f"Image processing error: {str(e)}"
            rospy.logerr(error_msg)
            return VLMProcessResponse(error_msg)

        # 发送请求到VLM API
        # language = "用中文回答我"
        payload = {"image": encoded_image, "prompt": req.prompt }
        try:
            response = requests.post(self.api_url, json=payload, timeout=10)
            if response.ok:
                result = response.json().get("read_message", "No valid result")
                rospy.loginfo(result)
                # 安全解析返回结果
                try:
                    parsed_result = ast.literal_eval(result) if isinstance(result, str) else result
                except:
                    parsed_result = str(result)
                return VLMProcessResponse(str(parsed_result))
            else:
                error_msg = f"API Error {response.status_code}: {response.text}"
                rospy.logerr(error_msg)
                return VLMProcessResponse(error_msg)
        except Exception as e:
            error_msg = f"Network error: {str(e)}"
            rospy.logerr(error_msg)
            return VLMProcessResponse(error_msg)

if __name__ == "__main__":
    rospy.init_node('vlm_service_server')
    server = VLMServiceServer()
    rospy.spin()