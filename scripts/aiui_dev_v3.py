#! /usr/bin/env python3
import struct
import time
import json
from socket import *
import signal
import requests
from threading import Thread, Event, Lock
import threading
import subprocess
import queue
from aiui.srv._PlayArmMovement import PlayArmMovementRequest
import numpy as np
import pygame
import pygame.mixer
import os
import random
from datetime import datetime

import rosbag
import rospy
import math
import actionlib
from aiui.srv import TTS, TTSRequest
from aiui.srv import TTSV2, TTSV2Request
from aiui.srv import VLMProcess, VLMProcessRequest
from aiui.srv import StringService, StringServiceRequest
from aiui.srv import DH5SetPosition, DH5SetPositionRequest
from aiui.srv import VLAProcess, VLAProcessRequest
from aiui.srv import CheckRunStatus, CheckRunStatusRequest
from aiui.srv import PlayArmMovement, PlayArmMovementResponse
from aiui.srv import RecoverService, RecoverServiceRequest
from arm_teleop.srv import StartDualTeleOP, StartDualTeleOPRequest
from arm_teleop.msg import DualArmMovej, DualHandTele
from woosh_msgs.msg import StepControlAction, StepControlGoal, StepControl
from woosh_msgs.srv import ExecTask, ExecTaskRequest
from woosh_msgs.msg import RobotStatus
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from sentenceBuffer import SentenceBuffer
# from sr_modbus_sdk import SRModbusSdk
from processStrategy import AiuiMessageProcess, ConfirmProcess

def stop_handle(sig, frame):
    global run
    run = False


signal.signal(signal.SIGINT, stop_handle)
run = True



class SocketDemo(Thread):
    def __init__(self):
        super().__init__()
        self.client_socket = None
        self.server_ip_port = ('192.168.8.206', 19199)
        self.server_ip = self.server_ip_port[0]
        self.connected_event = Event()
        self.stop_event = Event()
        self.connect()
        self.start_ping_check()
        self.detected_intent = None
        self.tts_text = ""
        self.wakeup_state = False
        # self.start_time = 0
        # self.end_time = 0
        self.intent_list = ["SayHi", "handshake", "guolai", "LabTour", 
                            "Bow", "Nod", "vlm", "vla", "self_photo", 
                            "pangu", "take_photo", "LOVE", "handclap", 
                            "chanxian", "Forward", "Backwards", "Goleft", 
                            "Goright", "Turnleft", "Turnright",
                            "zhezhi_robot", "medical_robot", "award_intro",
                            "bianbao_hand", "chanxian", "project_intro",
                            "dabianbao_robot", "ruanti_robot", "bianbao_robot",
                            "paper_intro", "lab_intro", "back_home", "recovery", "jianxiu", "playvedio"]

        self.arm_client = rospy.ServiceProxy("aris_node/cmd_str_srv",StringService)
        self.vlm_client = rospy.ServiceProxy("vlm_service",VLMProcess)
        # self.tts_client = rospy.ServiceProxy("/tts_service/generator",TTS)
        self.tts_client = rospy.ServiceProxy("/tts_service/tts_v2",TTSV2)
        self.dh5_client = rospy.ServiceProxy("/dh5/set_all_position",DH5SetPosition)
        self.vla_client = rospy.ServiceProxy("vla_service", VLAProcess)
        self.check_client = rospy.ServiceProxy("/aris_node/check_srv", CheckRunStatus)
        self.play_moment_client = rospy.ServiceProxy("play_arm_movement", PlayArmMovement)

        self.recover_client = rospy.ServiceProxy("/aris_node/recover_srv", RecoverService)

        self.seen_status_0 = False  # 标记是否见过状态0
        self.intent_state = False  # intent状态标志
        self.openQA = False  # 是否开启开放式问答
        self.vlm_text = ""  # VLM文本
        self.vla_text = ""  # VLA文本
        self.audio_thread = None  # 用于播放音频的线程
        # 新增的音频处理系统
        pygame.mixer.init()
        self.tts_queue = queue.Queue()  # TTS任务队列
        self.audio_queue = queue.Queue()  # 音频播放队列
        self.audio_lock = Lock()  # 音频锁确保连续播放
        self.active_audio_channel = None
        self.current_audio_stream = None
        self.active_audio_file = None
        self.xgxg = False  # 控制实验室参观的标志
        self.lab_tour = False  # 控制实验室参观的标志

        self.woosh_client = actionlib.SimpleActionClient('/cmd_vel_control', StepControlAction)
        self.timeout = 5
        rospy.loginfo("等待 step_control Action 服务器...")
        # 添加超时和错误处理
        if not self.woosh_client.wait_for_server(rospy.Duration(self.timeout)):
            rospy.logerr(f"无法连接到Action服务器，超时: {self.timeout}秒")
            raise Exception("Action服务器连接失败")
        rospy.loginfo("连接到 step_control Action 服务器")

        rospy.wait_for_service('/exec_task')
        try:
            self.exec_task_client = rospy.ServiceProxy('/exec_task', ExecTask)
            rospy.loginfo("连接到 /exec_task 服务成功")
        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: %s", e)


        # 初始化状态监听
        self.robot_status = None
        self.status_subscriber = rospy.Subscriber('/robot_status', RobotStatus, self.robot_status_callback)

        # 启动TTS和音频处理线程
        
        Thread(target=self.process_tts_queue, daemon=True).start()
        Thread(target=self.play_audio_from_queue, daemon=True).start()
        # 创建句子缓冲区
        self.sentence_buffer = SentenceBuffer(min_sentence_length=2, max_sentence_length=30)
        
        # 启动句子处理线程
        self.sentence_processing_thread = Thread(target=self.process_sentences, daemon=True)
        self.sentence_processing_thread.start()

    def robot_status_callback(self, msg):
        """机器人状态回调函数"""
        self.robot_status = msg
    
    def wait_for_task_completion(self, mark_no="", timeout=60):
        """
        等待任务完成
        
        参数:
        - timeout: 超时时间（秒），默认60秒
        - mark_no: 任务标识，默认空字符串
        返回:
        - True: 任务完成
        - False: 任务失败或超时
        """
        start_time = time.time()
        last_state = 7
        task_flag = 0
        too_long = False
        very_long = False
        
        while task_flag == 0:
            if self.xgxg == True and self.lab_tour == True:
                self.cancel_task(mark_no)
            # 检查超时
            if time.time() - start_time > timeout:
                rospy.logerr(f"任务等待超时（{timeout}秒）")
                self.cancel_task(mark_no)
                return False
            if time.time() - start_time > 30.0 and too_long == False and self.detected_intent == "bianbao_hand":
                self.sentence_buffer.append_text("等我清清嗓子，摆个舒服的姿势再为您介绍！")
                too_long = True
            if time.time() - start_time > 45.0 and very_long == False and self.detected_intent == "bianbao_hand":
                self.sentence_buffer.append_text("让您久等了，我继续为您介绍吧！")
                very_long = True
            
            if self.robot_status is None:
                rospy.logwarn("未收到机器人状态消息，等待中...")
                rospy.sleep(1)
                continue
                
            current_state = self.robot_status.task_state
            rospy.loginfo(f"当前任务状态: {current_state}")
            
            # 检查任务状态
            if current_state == 0 and last_state == 3:
                rospy.loginfo("任务执行完成")
                task_flag = 1
                return True
            if current_state == 7 and last_state == 3:
                rospy.loginfo("任务执行完成")
                task_flag = 1
                return True
            if current_state == 5 and last_state == 3:
                rospy.loginfo("任务执行完成")
                task_flag = 1
                return True
            
            # 等待一段时间后再次检查
            last_state = current_state
            rospy.sleep(0.1)
        
        return True


    def flush_all(self):
        """清空所有队列和缓冲区，删除相关音频文件"""
        # 清空文本缓冲区并添加到TTS队列
        with self.sentence_buffer.lock:
            if self.sentence_buffer.buffer:
                self.tts_queue.put(self.sentence_buffer.buffer)
                self.sentence_buffer.buffer = ""
        
        # 清空TTS队列
        flushed_texts = []
        while not self.tts_queue.empty():
            try:
                text = self.tts_queue.get_nowait()
                if text:
                    flushed_texts.append(text)
            except queue.Empty:
                break
        
        if flushed_texts:
            rospy.loginfo(f"清空TTS队列中的文本: {', '.join(flushed_texts)}")
        
        # 清空音频队列并删除文件
        audio_files_to_delete = []
        while not self.audio_queue.empty():
            try:
                audio_path = self.audio_queue.get_nowait()
                if audio_path:
                    audio_files_to_delete.append(audio_path)
            except queue.Empty:
                break
        
        # 删除所有音频文件
        for audio_path in audio_files_to_delete:
            try:
                if os.path.exists(audio_path):
                    os.remove(audio_path)
                    rospy.loginfo(f"已删除临时文件: {audio_path}")
                else:
                    rospy.logwarn(f"文件不存在: {audio_path}")
            except Exception as e:
                rospy.logwarn(f"删除临时文件失败: {e}")
        
        # 停止当前播放并删除当前播放文件（如果有）
        with self.audio_lock:
            if pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()
            
            # 如果当前有正在播放的文件，删除它
            if hasattr(self, 'current_playing_file') and self.current_playing_file:
                try:
                    if os.path.exists(self.current_playing_file):
                        os.remove(self.current_playing_file)
                        rospy.loginfo(f"已删除当前播放文件: {self.current_playing_file}")
                    self.current_playing_file = None
                except Exception as e:
                    rospy.logwarn(f"删除当前播放文件失败: {e}")

    # 处理TTS队列的线程函数
    def process_tts_queue(self):
        while not self.stop_event.is_set():
            try:
                # 从队列获取待处理文本
                text = self.tts_queue.get(timeout=1)
                # 调用TTS服务
                req = TTSV2Request()
                req.tts_text = text
                resp = self.tts_client.call(req)

                # 获取保存的音频文件名
                file_name = "/home/pangu/pangu/src/aiui/tts_audio/" + resp.file_name
            
                # 将音频文件加入播放队列
                if file_name:
                    self.audio_queue.put(file_name)

            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error in TTS processing: {str(e)}")
    
    # 音频播放线程函数
    def play_audio_from_queue(self):
        # self.end_time = time.time()
        # print(f"音频播报延时 {self.end_time - self.start_time:.2f} 秒")

        while not self.stop_event.is_set():
            try:
                # 获取音频文件路径
                file_path = self.audio_queue.get(timeout=1)
                
                # 使用锁确保音频连贯播放
                with self.audio_lock:
                    # 如果有当前在播放的音频，等待其结束
                    if pygame.mixer.music.get_busy():
                        # self.end_time = time.time()
                        # print(f"音频播报延时 {self.end_time - self.start_time:.2f} 秒")
                        # 设置混合器事件监听
                        BUSY_EVENT = pygame.USEREVENT + 1
                        pygame.mixer.music.set_endevent(BUSY_EVENT)
                        
                        # 设置超时等待（最长30秒）
                        wait_start = time.time()
                        while pygame.mixer.music.get_busy():
                            if time.time() - wait_start > 30:  # 30秒超时
                                pygame.mixer.music.stop()
                                break
                            pygame.time.delay(100)  # 每100ms检查一次
                    
                    # 加载并播放新音频
                    pygame.mixer.music.load(file_path)
                    pygame.mixer.music.play()
                    rospy.loginfo(f"Playing audio: {file_path}")
                    # 播放完成后删除临时文件
                    try:
                        os.remove(file_path)
                        rospy.loginfo(f"已删除临时文件: {file_path}")
                    except Exception as e:
                        rospy.logwarn(f"删除临时文件失败: {e}")
                    
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error in audio playback: {str(e)}")
    

    # 将文本添加到TTS队列
    def add_to_tts_queue(self, text):
        if text:
            self.tts_queue.put(text)
    
    # 播放现有音频文件（如有）
    def play_existing_audio(self, file_path):
        pygame.mixer.init()
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():  # 等待播放结束
            pygame.time.Clock().tick(10)
    
    def connect(self):
        while not self.stop_event.is_set():
            try:
                if self.client_socket:
                    self.client_socket.close()
                self.client_socket = socket(AF_INET, SOCK_STREAM)
                self.client_socket.settimeout(5)  # 设置连接超时
                self.client_socket.connect(self.server_ip_port)
                rospy.loginfo(f"Connected to {self.server_ip_port}")
                self.client_socket.settimeout(None)  # 连接成功后取消超时
                self.connected_event.set()
                break
            except (ConnectionError, OSError, timeout) as e:
                rospy.logerr(f"Connection error: {e}. Retrying in 5 seconds...")
                self.connected_event.clear()
                time.sleep(5)

    def receive_full_data(self, expected_length):
        received_data = bytearray()
        while len(received_data) < expected_length:
            try:
                chunk = self.client_socket.recv(
                    expected_length - len(received_data))
                if not chunk:
                    # print("Connection closed by the server.")
                    return None
                received_data.extend(chunk)
            except timeout:
                # print(f"Socket timeout")
                return None
        return bytes(received_data)

    def start_ping_check(self):
        Thread(target=self.ping_check, daemon=True).start()

    def ping_check(self):
        while not self.stop_event.is_set():
            try:
                response = subprocess.run(
                    ["ping", "-c", "1", self.server_ip],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE
                )
                if response.returncode != 0:
                    print(
                        f"Ping to {self.server_ip} failed. Reconnecting...")
                    self.connected_event.clear()
                    self.connect()
            except Exception as e:
                print(f"Error during ping: {e}")
            time.sleep(10)

    def close(self):
        self.stop_event.set()
        if self.client_socket:
            self.client_socket.close()
        print("Socket closed")

    def get_aiui_type(self, data):
        # rospy.loginfo("获取 AIUI 类型数据中...")
        # rospy.logwarn(f"获取 AIUI 类型数据: {data}")
        if 'content' in data:
            content = data['content']
            if 'info' in content:
                info = content['info']
                if not isinstance(info, dict):
                    return
                if 'data' in info and isinstance(info['data'], list) and len(info['data']) > 0:
                    data_item = info['data'][0]
                    if 'params' in data_item:
                        params = data_item['params']
                        sub_value = params.get('sub')
                        if sub_value is not None:
                            self.aiui_type = sub_value

    def get_iat_result(self, data):
        # 提取并拼接 w 字段
        words = []
        ws_list = data.get('content', {}).get(
            'result', {}).get('text', {}).get('ws', [])
        for item in ws_list:
            cw_list = item.get('cw', [])
            for cw in cw_list:
                words.append(cw.get('w', ''))
        # 拼接成字符串
        sn_value = data.get('content', {}).get(
            'result', {}).get('text', {}).get('sn')
        ls_value = data.get('content', {}).get(
            'result', {}).get('text', {}).get('ls')
        status_value = -1
        if (sn_value == 1):
            status_value = 0
        elif (ls_value == True):
            status_value = 2
        else:
            status_value = 1
        result_string = ''.join(words)
        if status_value == 0:
            # rospy.loginfo(f"新识别开始, 状态0")
            self.flush_all()  # 清空之前的缓冲区
        if (result_string != "" or status_value == 2):
            # pass
            rospy.loginfo(f"识别结果是: {result_string} {status_value}")

    def thake_photo(self):
        bridge = CvBridge()
    
        try:
            rospy.sleep(0.8)
            self.play_existing_audio("/home/pangu/pangu/src/aiui/audio/look_at_me.mp3")
            # rospy.sleep(2)
            self.play_existing_audio("/home/pangu/pangu/src/aiui/audio/get_ready.mp3")
            # rospy.sleep(2)
            self.play_existing_audio("/home/pangu/pangu/src/aiui/audio/pose.mp3")
            # rospy.sleep(2)
            self.play_existing_audio("/home/pangu/pangu/src/aiui/audio/three.mp3")
            rospy.sleep(1.5)
            self.play_existing_audio("/home/pangu/pangu/src/aiui/audio/two.mp3")
            rospy.sleep(1.5)
            self.play_existing_audio("/home/pangu/pangu/src/aiui/audio/one.mp3")
            rospy.sleep(2.5)
            self.play_existing_audio("/home/pangu/pangu/src/aiui/audio/qie_zi.mp3")
            # rospy.sleep(2)

            
            # 等待并获取ROS图像消息
            rospy.loginfo("等待相机图像消息...")
            image_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
            rospy.loginfo("成功获取图像!")
            
            # 保存图像
            timestamp = rospy.get_time()  # 获取ROS时间
            human_time = datetime.fromtimestamp(timestamp).strftime("%Y%m%d_%H%M%S_%f")
            filename = f"captured_image_{human_time}.jpg"
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"图像已保存为 {filename}")
            # cv2.imwrite("captured_image.jpg", cv_image)
            # rospy.loginfo("图像已保存为 captured_image.jpg")
            
            # 创建全屏窗口显示图像
            screen_width, screen_height = 1920, 1080  # 根据实际屏幕分辨率调整
            window_name = "Fullscreen Image"
            cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            
            # 调整图像尺寸以适应屏幕（保持宽高比）
            h, w = cv_image.shape[:2]
            scale = min(screen_width/w, screen_height/h)
            resized_image = cv2.resize(cv_image, (int(w*scale), int(h*scale)))
            
            # 创建黑色背景并在中央显示图像
            display_image = np.zeros((screen_height, screen_width, 3), dtype=np.uint8)
            x_offset = (screen_width - resized_image.shape[1]) // 2
            y_offset = (screen_height - resized_image.shape[0]) // 2
            display_image[y_offset:y_offset+resized_image.shape[0], 
                        x_offset:x_offset+resized_image.shape[1]] = resized_image
            
            # 显示图像
            cv2.imshow(window_name, display_image)
            rospy.loginfo("按任意键退出全屏显示...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            
        except rospy.ROSException:
            rospy.logerr("等待图像消息超时，请检查相机是否已启动")
        except Exception as e:
            rospy.logerr(f"发生错误: {str(e)}")
    def labTour(self):
        self.lab_tour = True
        self.xgxg = False
        greetings = [
            "有什么我能帮到您的，尽管告诉我哦！",
            "你好呀，很高兴见到您！",
            "让我来为您介绍一下我们的实验室吧！",
        ]
        for i in [2,6,8,9,8,6,2]:
            if self.xgxg == True:
                break
            self.send_task(str(i))
            # 随机挑一句问候，避免重复感
            self.sentence_buffer.append_text(random.choice(greetings))
            rospy.sleep(0.5)
        self.xgxg = False
        self.lab_tour = False
        # self.sentence_buffer.append_text("回到初始点位！")

    def handle_detected_intent(self, intent):
        
        self.intent_state = True  # 重置意图状态
        rospy.loginfo(f"意图状态: {self.intent_state}")
        
        if intent == "SayHi":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行打招呼动作")
            self.sentence_buffer.append_text("你好呀，很高兴见到您！")
            req = StringServiceRequest()
            req.request = 3
            # self.arm_client.wait_for_service()
            resp = self.check_client.call(CheckRunStatusRequest())
            if resp.ros_run_flag is False:
                Thread(target=self.arm_client.call, args=(req,), daemon=True).start()
        elif intent == "handshake":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行握手动作")
            self.sentence_buffer.append_text("好呀，很高兴认识你，我还想再多和你交流交流呢！")
            req = StringServiceRequest()
            req.request = 4 # TODO
            # self.arm_client.wait_for_service()
            resp = self.check_client.call(CheckRunStatusRequest())
            if resp.ros_run_flag is False:
                Thread(target=self.arm_client.call, args=(req,), daemon=True).start()
        elif intent == "guolai":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行过来动作")
            self.woosh_mf(0.2, 0.1, True) 
        elif intent == "LabTour":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行实验室参观动作")
            self.sentence_buffer.append_text("好啊，我带你在实验室里逛一逛吧！")
            Thread(target=self.labTour, daemon=True).start()
            # self.sentence_buffer.append_text("有什么我能帮到您的，尽管告诉我哦！")
            # for i in range(2, 13):
            #    self.send_task(str(i))
            #    rospy.sleep(2)
            # self.sentence_buffer.append_text("回到初始点位！")
        elif intent == "Bow":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行鞠躬欢送动作")
            self.sentence_buffer.append_text("哇时间过得好快, 再见喽，期待下次再和您见面，记得要常来看我哦！")
            req = StringServiceRequest()
            req.request = 5 # TODO
            # self.arm_client.wait_for_service()
            resp = self.check_client.call(CheckRunStatusRequest())
            if resp.ros_run_flag is False:
                Thread(target=self.arm_client.call, args=(req,), daemon=True).start()
        elif intent == "Nod":

            print(f"检测到 [{intent}] 意图, 执行点头动作")
            self.sentence_buffer.append_text("我是南科盘古，")

            self.sentence_buffer.append_text("由南方科技大学机器人研究院研发的首款人形机器人，")
            self.sentence_buffer.append_text("也是深圳地区首个完全由高校独立研制的人形机器人，")
            self.sentence_buffer.append_text("我具有高度拟人化的手臂,")
            self.sentence_buffer.append_text("搭载多模态大模型，")
            self.sentence_buffer.append_text("能够进行多模式智能交互，")
            self.sentence_buffer.append_text("很高兴认识您！")
            
            # req = StringServiceRequest()
            # req.request = '2' # TODO
            # self.arm_client.wait_for_service()
            # self.arm_client.call(req)
            # Thread(target=self.arm_client.call, args=(req,), daemon=True).start()       
        elif intent == "vla":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行视觉语言动作")
            self.sentence_buffer.append_text("好的，没问题！")

            vla_req = VLAProcessRequest()
            vla_req.prompt = self.vla_text
            # self.vla_client.wait_for_service()
            resp = self.check_client.call(CheckRunStatusRequest())
            if resp.ros_run_flag is False:
                Thread(target=self.vla_client.call, args=(vla_req,), daemon=True).start()       
        elif intent == "vlm":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行描述动作")
            self.sentence_buffer.append_text("好的，让我仔细看一下！")

            vlm_req = VLMProcessRequest()
            vlm_req.prompt = self.vlm_text
            # self.vlm_client.wait_for_service()
            resp = self.vlm_client.call(vlm_req)
            vlm_result = resp.vlm_result
            rospy.loginfo(f"VLM 结果: {vlm_result}")
            self.sentence_buffer.append_text(vlm_result)
        elif intent == "self_photo":
            # self.intent_state = True
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行自拍动作")
            self.sentence_buffer.append_text("好的，摆个点赞的姿势，来和我自拍一张吧")
            arm_req = StringServiceRequest()
            arm_req.request = random.choice([6, 9, 10])
            # arm_req.request = 6
            # self.arm_client.wait_for_service()
            resp = self.check_client.call(CheckRunStatusRequest())
            if resp.ros_run_flag is False:
                Thread(target=self.arm_client.call, args=(arm_req,), daemon=True).start()
        elif intent == "pangu":
            # self.intent_state = True
            rospy.loginfo(f"检测到 [{intent}] 意图, 讲述盘古开天地的故事")
            self.sentence_buffer.append_text("好的，盘古是中国古代传说时期中开天辟地的神。")
            self.sentence_buffer.append_text("在很久很久以前，宇宙混沌一团，盘古凭借着自己的神力把天地开辟出来了。")           
        elif intent == "take_photo":
            # self.intent_state = True
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行拍照动作")
            # self.sentence_buffer.append_text("好的，没问题，大家跟我过来一下！让我来帮大家拍一张合照！")
            # self.send_task("13")
            self.sentence_buffer.append_text("大家都过来吧！准备好了么，站到我面前，让我来为大家拍一张大合照！")
            rospy.sleep(7)
            arm_req = StringServiceRequest()
            arm_req.request = 7
            # self.arm_client.wait_for_service()
            resp = self.check_client.call(CheckRunStatusRequest())
            if resp.ros_run_flag is False:
                Thread(target=self.arm_client.call, args=(arm_req,), daemon=True).start()
                self.thake_photo()
        elif intent == "LOVE":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行比心动作")
            self.sentence_buffer.append_text("南科大爱你呦！啾咪啾咪！")
            arm_req = StringServiceRequest()
            arm_req.request = random.choice([13])
            # self.arm_client.wait_for_service()
            resp = self.check_client.call(CheckRunStatusRequest())
            if resp.ros_run_flag is False:
                Thread(target=self.arm_client.call, args=(arm_req,), daemon=True).start()
        elif intent == "handclap":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行鼓掌动作")
            self.sentence_buffer.append_text("来大家一起鼓掌！精彩！精彩！！")
            arm_req = StringServiceRequest()
            arm_req.request = 8 # TODO
            # self.arm_client.wait_for_service()
            resp = self.check_client.call(CheckRunStatusRequest())
            if resp.ros_run_flag is False:
                Thread(target=self.arm_client.call, args=(arm_req,), daemon=True).start()
        elif intent == "Forward":
            self.sentence_buffer.append_text("好的，没问题！")
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行前进动作")
            self.woosh_mf(0.2, 0.1)
        elif intent == "Backwards":
            self.sentence_buffer.append_text("好的，没问题！")
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行后退动作")
            self.woosh_mf(-0.2, 0.1)
        elif intent == "Turnleft":
            self.sentence_buffer.append_text("好的，没问题！")
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行左转动作")
            self.woosh_rotate(10)
        elif intent == "Turnright":
            self.sentence_buffer.append_text("好的，没问题！")
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行右转动作")
            self.woosh_rotate(-10)
        elif intent == "Goleft":
            self.sentence_buffer.append_text("好的，没问题！")
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行左移动作")
            self.woosh_rotate(90)
            self.woosh_mf(0.2, 0.1)
            self.woosh_rotate(-90)
        elif intent == "Goright":
            self.sentence_buffer.append_text("好的，没问题！")
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行右移动作")
            self.woosh_rotate(-90)
            self.woosh_mf(0.2, 0.1)
            self.woosh_rotate(90)      
        elif intent == "lab_intro":
            self.sentence_buffer.append_text("好的，没问题！让我来为您介绍一下我们的研究院吧！")
            self.send_task("2")
            # rospy.sleep(2)
            self.flush_all()
            Thread(target=self.play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task2.mp3",)).start()
        elif intent == "paper_intro":
            self.sentence_buffer.append_text("好的，没问题！让我带您了解一下戴院士的著作！")
            self.send_task("3")
            # rospy.sleep(2)
            self.flush_all()
            Thread(target=self.play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task3.mp3",)).start()
        elif intent == "bianbao_robot":
            self.sentence_buffer.append_text("好的，没问题！让我来为您介绍一下我们的变胞机器人吧！")
            self.send_task("4")
            # rospy.sleep(2)
            self.flush_all()
            Thread(target=self.play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task4.mp3",)).start()
        elif intent == "ruanti_robot":
            self.sentence_buffer.append_text("好的，没问题！让我来为您介绍一下我们的软体机器人吧！")
            self.send_task("5")
            # rospy.sleep(2)
            self.flush_all()
            Thread(target=self.play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task5.mp3",)).start()
        elif intent == "dabianbao_robot":
            self.sentence_buffer.append_text("好的，没问题！让我来为您介绍一下我们第二代的多功能变胞机器人吧！")
            self.send_task("6")
            # rospy.sleep(2)
            self.flush_all()
            Thread(target=self.play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task6.mp3",)).start()
        elif intent == "project_intro":
            self.sentence_buffer.append_text("好的，没问题！让我来为您介绍一下我们的研究项目吧！")
            self.send_task("7")
            # rospy.sleep(2)
            self.flush_all()
            Thread(target=self.play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task7.mp3",)).start()
        elif intent == "chanxian":
            self.sentence_buffer.append_text("好的，没问题！让我来为您介绍一下我们的智能产线吧！")
            self.send_task("8")
            # rospy.sleep(2)
            self.flush_all()
            Thread(target=self.play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task8.mp3",)).start()
        elif intent == "bianbao_hand":
            self.sentence_buffer.append_text("好的，没问题！让我来为您介绍我们实验室研发的变胞灵巧手吧！")
            self.send_task("9")
            # rospy.sleep(2)
            req = PlayArmMovementRequest()
            req.rate = 18.0
            self.play_moment_client.wait_for_service()
            self.flush_all()
            self.play_moment_client.call(req)
            # self.play_arm_movement(rate=18.0)
            # Thread(target=self.play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task9.mp3",)).start()
        elif intent == "award_intro":
            self.sentence_buffer.append_text("好的，没问题！让我来为您介绍一下我们的研究成果与荣誉吧！")
            self.send_task("10")
            # rospy.sleep(2)
            self.flush_all()
            Thread(target=self.play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task10.mp3",)).start()
        elif intent == "medical_robot": 
            self.sentence_buffer.append_text("好的，没问题！让我来为您介绍一下我们的医疗机器人吧！")
            self.send_task("12")
            # rospy.sleep(2)
            self.flush_all()
            Thread(target=self.play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task11.mp3",)).start()
        elif intent == "zhezhi_robot":
            self.sentence_buffer.append_text("好的，没问题！让我来为您介绍一下我们的折纸机器人吧！")
            self.send_task("11")
            # rospy.sleep(2)
            self.flush_all()
            Thread(target=self.play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task12.mp3",)).start()
        elif intent == "back_home":
            self.sentence_buffer.append_text("那我先回去休息啦，期待下次再和您见面，记得要常来看我哦！")
            self.send_task("1")
            # rospy.sleep(2)
        
        elif intent == "recovery":
            self.sentence_buffer.append_text("好的，没问题！让我先调整一下状态！")
            cmd_req = RecoverServiceRequest()
            cmd_req.cmd = "recover"
            self.recover_client.call(cmd_req)
            # Thread(target=self.recover_client.call, args=(cmd_req,), daemon=True).start
        
        elif intent == "jianxiu":
            self.sentence_buffer.append_text("好的，没问题！让我先调整一下状态！")
            cmd_req = RecoverServiceRequest()
            cmd_req.cmd = "maintain"
            self.recover_client.call(cmd_req)
            # Thread(target=self.recover_client.call, args=(cmd_req,), daemon=True).start
            

    
    # 替换原有的play_audio函数
    def enqueue_audio(self, file_path):
        """将音频文件加入播放队列（替代直接播放）"""
        self.audio_queue.put(file_path)


    def get_nlp_result(self, data):
        # if self.intent_state == True:
        #     # rospy.loginfo("意图状态，忽略 NLP 结果")
        #     return
        # 提取 text 字段
        text_value = data.get('content', {}).get(
            'result', {}).get('nlp', {}).get('text')

        status_value = data.get('content', {}).get(
            'result', {}).get('nlp', {}).get('status')

        if text_value is not None and status_value is not None:
            # pass
            rospy.loginfo(f"大模型回答结果是: {text_value}  {status_value}")
        # 状态0: 新响应开始
        if status_value == 0:
            # rospy.loginfo(f"新响应开始, 状态0")
            self.seen_status_0 = True  # 标记已见过状态0
            if self.intent_state == True:
                self.seen_status_0 = False  # 假装没有看见
                # rospy.loginfo("意图状态, 忽略状态0")
            else:
                self.flush_all()  # 清空之前的缓冲区
                # rospy.loginfo("清空缓冲区，开始新响应")
                self.sentence_buffer.append_text(text_value)

        # 状态1: 中间段落
        elif status_value == 1:
            # rospy.loginfo(f"中间段落, 状态1")
            if self.intent_state == True:
                self.seen_status_0 = False  # 重置状态标志
            if  self.seen_status_0:
                self.sentence_buffer.append_text(text_value)

        # 状态2: 最终段落
        elif status_value == 2:
            # rospy.loginfo(f"最终段落, 状态2")
            if self.seen_status_0:
                self.sentence_buffer.append_text(text_value)
                self.seen_status_0 = False  # 重置状态标志
            # elif self.detected_intent == "WHATTIME" or self.openQA or self.intent_state == True:
            elif self.detected_intent == "WHATTIME" or self.openQA:
                # rospy.loginfo("最终段落，意图状态或开放式问答，追加文本")
                # rospy.loginfo(f"生效的意图: {self.detected_intent}" )
                self.flush_all()
                self.sentence_buffer.append_text(text_value)
                self.openQA = False
        
            
            
            if self.intent_state == True:
                # 如果是 intent-activated 状态，直接清空缓冲区
                # self.flush_all()
                self.intent_state = False
                # rospy.loginfo("意图状态已重置")
            
            self.seen_status_0 = False

        # 更新连贯性处理
        with self.audio_lock:
            if pygame.mixer.music.get_busy():
                # 标记为连贯播放下一条
                pass

    def process_sentences(self):
        """从缓冲区提取句子并进行处理"""
        while not self.stop_event.is_set():
            # 获取下一个完整句子
            sentence = self.sentence_buffer.get_next_sentence()
            if not sentence:
                time.sleep(0.1)
                continue
                
            # 处理句子 - 发送到 TTS
            # self.end_time = time.time()
            self.tts_queue.put(sentence)

    def active_callback(self):
        """Goal开始执行时的回调"""
        rospy.loginfo("🚀 移动动作开始执行...")
    
    def feedback_callback(self, feedback):
        """进度反馈回调"""
        rospy.loginfo(f"Feedback: {feedback}")
        # rospy.loginfo(f"📊 移动进度: {feedback.feedback}, 百分比: {feedback.percent}%, 模式: {feedback.executeMode}")
    
    def done_callback(self, status, result):
        """Goal完成时的回调"""
        # 状态码说明
        status_names = {
            actionlib.GoalStatus.PENDING: "等待中",
            actionlib.GoalStatus.ACTIVE: "执行中", 
            actionlib.GoalStatus.PREEMPTED: "被抢占",
            actionlib.GoalStatus.SUCCEEDED: "成功",
            actionlib.GoalStatus.ABORTED: "失败",
            actionlib.GoalStatus.REJECTED: "被拒绝",
            actionlib.GoalStatus.PREEMPTING: "抢占中",
            actionlib.GoalStatus.RECALLING: "召回中",
            actionlib.GoalStatus.RECALLED: "已召回",
            actionlib.GoalStatus.LOST: "丢失"
        }
        
        status_name = status_names.get(status, "未知状态")
        rospy.loginfo(f"🎯 移动动作完成! 状态: {status_name}({status}), 结果码: {result.result}")
        
        # 根据结果进行不同处理
        if status == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("✅ 移动任务成功完成！")
        elif status == actionlib.GoalStatus.PREEMPTED:
            rospy.logwarn("⚠️ 移动任务被取消！")
        elif status == actionlib.GoalStatus.ABORTED:
            rospy.logerr("❌ 移动任务失败！")
    
    def woosh_rotate(self, angle, mode='normal'):
        """发送移动命令"""
        try:
            goal = StepControlGoal()
            goal.mode = StepControlGoal.EXCUTE
            goal.useAvoid = False
            
            step = StepControl()
            step.executeMode = StepControlGoal.ROTATE
            if mode == 'wake_up':
                step.data = -(angle - 90) / 180.0 * math.pi  # 听声辨位特殊处理
            elif mode == 'normal':
                step.data = angle / 180.0 * math.pi  # 角度转弧度
            step.speed = 2
            # step.angle = (angle - 90) / 180.0 * math.pi  # 角度转弧度
            
            goal.stepControl = [step]
            
            # rospy.loginfo(f"发送移动指令: 距离={distance}m, 速度={speed}m/s, 避障={use_avoid}")
            
            # 发送目标并设置回调函数
            self.woosh_client.send_goal(goal, 
                                 done_cb=self.done_callback, 
                                 active_cb=self.active_callback, 
                                 feedback_cb=self.feedback_callback)
            rospy.loginfo("✅ 移动指令已发送，等待执行...")
            
            # 可选：等待结果（同步方式）
            success = self.woosh_client.wait_for_result(rospy.Duration(self.timeout))
            
            if success:
                rospy.loginfo("🎉 移动任务顺利完成！")
                return True
            else:
                rospy.logwarn("⏰ 移动任务超时！")
                self.woosh_client.cancel_goal()
                return False
                
        except Exception as e:
            rospy.logerr(f"发送指令时发生错误: {e}")
            return False
    
    def cancel_task(self, mark_no=""):
        """
        取消当前任务
        """
        try:
            # 创建服务请求
            req = ExecTaskRequest()
            
            req.task_exect = 4      # 取消任务
            req.task_id = 0         # 任务id
            req.task_type = 1       # 拣选
            req.direction = 0       # 动作方向
            req.task_type_no = 0    # 组合类型默认0
            req.mark_no = mark_no        # 储位编号
            
            rospy.loginfo("取消当前任务")
            response = self.exec_task_client.call(req)
            
            # 处理响应
            if response.success:
                rospy.loginfo(f"任务取消成功: {response.message}")
            else:
                rospy.logwarn(f"任务取消失败: {response.message}")
                rospy.logwarn(f"状态码: {response.statusCode}")
            
            return response
            
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用异常: {e}")
            return None
    
    def send_task(self, mark_no):
        """
        发送任务，只包含task_id和mark_no
        
        参数:
        - task_id: 任务ID
        - mark_no: 标记编号
        """
        try:
            # 创建服务请求
            req = ExecTaskRequest()
            
            req.task_exect = 1      # 执行任务
            req.task_id = 0         # 任务id
            req.task_type = 1       # 拣选
            req.direction = 0       # 动作方向
            req.task_type_no = 0    # 组合类型默认0
            req.mark_no = mark_no   # 储位编号
            
            rospy.loginfo(f"发送任务: mark_no={mark_no}")
            response = self.exec_task_client.call(req)
            
            
            # 处理响应
            if response.success:
                rospy.loginfo(f"任务执行成功: {response.message}")
                # 等待任务完成
                rospy.loginfo("开始监听任务执行状态...")
                task_completed = self.wait_for_task_completion(mark_no)
                
                if task_completed:
                    rospy.loginfo("任务执行成功完成")
                else:
                    rospy.logwarn("任务执行未正常完成")
                    
                return task_completed
            else:
                rospy.logwarn(f"任务执行失败: {response.message}")
                rospy.logwarn(f"状态码: {response.statusCode}")
            
            return response
            
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用异常: {e}")
            return None


    def woosh_mf(self, distance, speed, use_avoid=True):
        """发送移动命令"""
        try:
            goal = StepControlGoal()
            goal.mode = StepControlGoal.EXCUTE
            goal.useAvoid = use_avoid
            
            step = StepControl()
            step.executeMode = StepControlGoal.STRAIGHT
            step.data = distance
            step.speed = speed
            # step.angle = (angle - 90) / 180.0 * math.pi  # 角度转弧度
            
            goal.stepControl = [step]
            
            # rospy.loginfo(f"发送移动指令: 距离={distance}m, 速度={speed}m/s, 避障={use_avoid}")
            
            # 发送目标并设置回调函数
            self.woosh_client.send_goal(goal, 
                                 done_cb=self.done_callback, 
                                 active_cb=self.active_callback, 
                                 feedback_cb=self.feedback_callback)
            rospy.loginfo("✅ 移动指令已发送，等待执行...")
            
            # 可选：等待结果（同步方式）
            success = self.woosh_client.wait_for_result(rospy.Duration(self.timeout))
            
            if success:
                rospy.loginfo("🎉 移动任务顺利完成！")
                return True
            else:
                rospy.logwarn("⏰ 移动任务超时！")
                self.woosh_client.cancel_goal()
                return False
                
        except Exception as e:
            rospy.logerr(f"发送指令时发生错误: {e}")
            return False
        
            

    def get_intent_result(self, data):
        text_value = data.get('content', {}).get(
            'result', {}).get('cbm_semantic', {}).get('text')
        # rospy.loginfo(f"技能 text_value: {text_value} ")
        intent = json.loads(text_value)
        rc = intent['rc']
        if (rc == 0):
            category = intent.get('category', "")
            # rospy.loginfo(f"技能结果: {category} ")
        parsed_data = json.loads(text_value)
        if not isinstance(parsed_data, dict):
            rospy.logerr("解析后的数据不是字典格式")
            return
        try:
            if parsed_data.get("answer", {}).get("answerType") == "openQA":
                self.openQA = True
                rospy.loginfo("开启开放式问答模式")
        except (IndexError, AttributeError, TypeError, KeyError) as e:
            rospy.loginfo(f"非开放式问答: {str(e)}")
        
        try:
            self.detected_intent = parsed_data.get('semantic', {})[0].get('intent', {})

        except (IndexError, AttributeError, TypeError, KeyError) as e:
            self.detected_intent = None
            rospy.logwarn(f"无意图: {str(e)}")
        if self.detected_intent:
            # rospy.loginfo(f"成功提取意图: {self.detected_intent}")
            self.intent_state = False  # 重置意图状态
            if self.detected_intent == "vla":
                try:
                    self.vla_text = parsed_data.get('text', "")
                    # rospy.loginfo(f"技能 VLA 文本: {self.vla_text} ")
                except (IndexError, AttributeError, TypeError, KeyError) as e:
                    self.vla_text = ""
                    rospy.logwarn(f"语义 VLA 解析小异常: {str(e)}")
            
            if self.detected_intent == "vlm":
                try:
                    self.vlm_text = parsed_data.get('semantic', {})[0].get('template', "")
                    # rospy.loginfo(f"技能 VLM 文本: {self.vlm_text} ")
                
                except (IndexError, AttributeError, TypeError, KeyError) as e:
                    self.vlm_text = ""
                    rospy.logwarn(f"语义 VLM 解析小异常: {str(e)}")

            if self.detected_intent in self.intent_list:
                self.flush_all()  # 清空之前的缓冲区
                self.handle_detected_intent(self.detected_intent)
        else:
            rospy.logwarn("未检测到预设动作指令意图")

    def run(self):
        try:

            rospy.loginfo("Program started")
            while run:
                demo.process()
        finally:
            demo.close()
            rospy.loginfo("Program terminated")

    def process(self):
        try:
            self.client_socket.settimeout(3)  # 设置接收超时
            recv_data = self.receive_full_data(7)
            if not recv_data:
                # rospy.loginfo("No data received. Reconnecting...")
                self.connected_event.clear()
                self.connect()
                return

            if len(recv_data) < 7:
                rospy.logwarn(f"Incomplete data received: {recv_data}")
                return

            sync_head, user_id, msg_type, msg_length, msg_id = struct.unpack(
                '<BBBHH', recv_data)

            # 校验接收的数据长度
            msg_data = self.receive_full_data(msg_length + 1)

            if len(msg_data) < msg_length + 1:
                rospy.logwarn(f"Incomplete data received: {msg_data}")
                return

            # 解析消息数据
            msg = msg_data[: msg_length]
            # 校检码（最后一个字节）
            check_code = msg_data[-1]

            if sync_head == 0xa5 and user_id == 0x01:

                if msg_type == 0x01:
                    ConfirmProcess().process(self.client_socket, msg_id)
                elif msg_type == 0x04:
                    ConfirmProcess().process(self.client_socket, msg_id)
                    success, result = AiuiMessageProcess().process(self.client_socket, msg)
                    if success:
                        self.aiui_type = ""
                        data = json.loads(result)
                        self.get_aiui_type(data)
                        
                        # rospy.loginfo(f"AIUI message processed successfully: {result.decode('utf-8')}")

                        if data.get('content', {}).get('eventType', {}) == 5:
                            self.wakeup_state = False
                            rospy.loginfo(f"进入休眠状态：==== 我不在 ==== ")
                            self.xgxg = False
                            self.flush_all()  # 清空之前的缓冲区
                            self.sentence_buffer.append_text("我先退下啦！")
                        
                        if data.get('content', {}).get('eventType', {}) == 1:
                            key_ward = data.get('content', {}).get('result', {}).get('ivw_result', {}).get('keyword', {})
                            if key_ward == "xiao3 gu3 xiao3 gu3" or key_ward == "pan2 gu3 pan2 gu3" or key_ward == "xiao3 hu3 xiao3 hu3":
                                # rospy.loginfo(f"检测到小鼓小鼓唤醒词")
                                try:
                                    angle = data.get('content', {}).get('result', {}).get('ivw_result', {}).get('angle', {})
                                    rospy.loginfo(f"唤醒词角度: {angle}")
                                    rospy.loginfo(f"唤醒完整消息: {result.decode('utf-8')}")
                                    if angle is not None:
                                        rospy.loginfo(f"声源定位: {angle}")
                                        rospy.loginfo(f"唤醒成功：==== 我在 ==== ")
                                        self.xgxg = True
                                        self.flush_all()  # 清空之前的缓冲区
                                        self.sentence_buffer.append_text("我在。")
                                        self.woosh_rotate(float(angle), mode='wake_up')
                                        rospy.loginfo(f"检测到唤醒词")
                                        
                                        
                                except (IndexError, AttributeError, TypeError, KeyError) as e:
                                    rospy.logwarn(f"声源定位解析小异常: {str(e)}")
                            

                        # if data.get('content', {}).get('eventType', {}) == 4:
                        #     rospy.loginfo(f"检测到唤醒词")
                        #     if self.wakeup_state == False:
                        #         self.wakeup_state = True
                        #         rospy.loginfo(f"唤醒成功：==== 我在 ==== ")
                        #         self.flush_all()  # 清空之前的缓冲区
                        #         self.sentence_buffer.append_text("我在。")
                                
                            
                        if (self.aiui_type == "iat"):
                            self.get_iat_result(data)

                        elif (self.aiui_type == "nlp"):
                            self.get_nlp_result(data)

                        elif (self.aiui_type == "cbm_semantic"):
                            self.get_intent_result(data)

                        # print(f"AIUI message processed successfully: {result.decode('utf-8')}")
                    else:
                        rospy.logwarn("AIUI message processing failed")
            else:
                return
        except timeout:
            return
        except (ConnectionError, OSError) as e:
            print(
                f"Connection error during process: {e}. Reconnecting...")
            self.connected_event.clear()
            self.connect()


if __name__ == '__main__':
    rospy.init_node("AIUI_node")
    
    demo = SocketDemo()
    demo.start()

    rospy.spin()
    