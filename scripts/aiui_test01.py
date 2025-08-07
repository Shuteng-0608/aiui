#! /usr/bin/env python3
import struct
import time
import json
from socket import *
import signal
import requests
from threading import Thread, Event, Lock
import subprocess
import queue
import numpy as np
import pygame
import pygame.mixer
import os
import random

import rospy
from aiui.srv import TTS, TTSRequest
from aiui.srv import VLMProcess, VLMProcessRequest
from aiui.srv import StringService, StringServiceRequest
from aiui.srv import DH5SetPosition, DH5SetPositionRequest
from aiui.srv import VLAProcess, VLAProcessRequest
from aiui.srv import CheckRunStatus, CheckRunStatusRequest
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
        self.server_ip_port = ('192.168.8.141', 19199)
        self.server_ip = self.server_ip_port[0]
        self.connected_event = Event()
        self.stop_event = Event()
        self.connect()
        self.start_ping_check()
        self.detected_intent = None
        self.tts_text = ""
        self.wakeup_state = False

        self.arm_client = rospy.ServiceProxy("aris_node/cmd_str_srv",StringService)
        self.vlm_client = rospy.ServiceProxy("vlm_service",VLMProcess)
        self.tts_client = rospy.ServiceProxy("/tts_service/generator",TTS)
        self.dh5_client = rospy.ServiceProxy("/dh5/set_all_position",DH5SetPosition)
        self.vla_client = rospy.ServiceProxy("vla_service", VLAProcess)
        self.check_client = rospy.ServiceProxy("/aris_node/check_srv", CheckRunStatus)

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
        # 启动TTS和音频处理线程
        Thread(target=self.process_tts_queue, daemon=True).start()
        Thread(target=self.play_audio_from_queue, daemon=True).start()
        # 创建句子缓冲区
        self.sentence_buffer = SentenceBuffer(min_sentence_length=2, max_sentence_length=30)
        
        # 启动句子处理线程
        self.sentence_processing_thread = Thread(target=self.process_sentences, daemon=True)
        self.sentence_processing_thread.start()


    def flush_all(self):
        """清空所有队列和缓冲区，删除相关音频文件"""
        with self.sentence_buffer.lock:
            # 清空文本缓冲区
            if self.sentence_buffer.buffer:
                self.tts_queue.put(self.sentence_buffer.buffer)
                self.sentence_buffer.buffer = ""
            
            # 清空TTS队列
            while not self.tts_queue.empty():
                try:
                    text = self.tts_queue.get_nowait()
                    if text:
                        rospy.loginfo(f"清空TTS队列中的文本: {text}")
                except queue.Empty:
                    break
            
            while not self.audio_queue.empty():
                try:
                    # 获取队列中的音频文件路径
                    audio_path = self.audio_queue.get_nowait()
                    try:
                        os.remove(audio_path)
                        rospy.loginfo(f"已删除临时文件: {audio_path}")
                    except Exception as e:
                        rospy.logwarn(f"删除临时文件失败: {e}")
                    
                except queue.Empty:
                    break

    # 处理TTS队列的线程函数
    def process_tts_queue(self):
        while not self.stop_event.is_set():
            try:
                # 从队列获取待处理文本
                text = self.tts_queue.get(timeout=1)
                # 调用TTS服务
                req = TTSRequest()
                req.request = text
                # self.tts_client.wait_for_service()
                resp = self.tts_client.call(req)
                # if resp.tts_url == "tts_url":
                #     rospy.logwarn("TTS服务未返回有效的音频URL")
                #     resp = self.tts_client.call(req)
                #     rospy.logwarn(resp)
                #     rospy.logwarn("重试中...")

                # 获取保存的音频文件名
                file_url = resp.tts_url
                try:
                    # 下载MP3文件
                    response = requests.get(file_url)
                    response.raise_for_status()  # 检查请求是否成功
                    
                    # 保存临时文件
                    timestamp = int(time.time() * 1000)  # 毫秒级时间戳
                    temp_file = f"tts_{timestamp}.mp3"
                    # temp_file = "temp_tts.mp3"
                    with open(temp_file, 'wb') as f:
                        f.write(response.content)
                    
                    # rospy.loginfo(f"已下载TTS音频到: {temp_file}")
                except Exception as e:
                    rospy.logerr(f"下载或播放音频时出错: {e}")

                
                filename = temp_file
                
                # 将音频文件加入播放队列
                if filename:
                    self.audio_queue.put(filename)
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error in TTS processing: {str(e)}")
    
    # 音频播放线程函数
    def play_audio_from_queue(self):

        while not self.stop_event.is_set():
            try:
                # 获取音频文件路径
                file_path = self.audio_queue.get(timeout=1)
                
                # 使用锁确保音频连贯播放
                with self.audio_lock:
                    # 如果有当前在播放的音频，等待其结束
                    if pygame.mixer.music.get_busy():
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
                    # rospy.loginfo(f"Playing audio: {file_path}")
                    # 播放完成后删除临时文件
                    try:
                        os.remove(file_path)
                        # rospy.loginfo(f"已删除临时文件: {file_path}")
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
        if 'content' in data:
            content = data['content']
            if 'info' in content:
                info = content['info']
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
        if (result_string != "" or status_value == 2):
            rospy.loginfo(f"识别结果是: {result_string} {status_value}")


    # def labTour(self, start, end):
    #     mb_server = SRModbusSdk()
    #     mb_server.connect_tcp('192.168.10.141')
    #     for i in range(start, end+1):
    #         mb_server.move_to_station_no(i, 1)
    #         mb_server.wait_movement_task_finish(1) 

    def thake_photo(self):
        bridge = CvBridge()
    
        try:
            rospy.sleep(0.8)
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/look_at_me.mp3")
            # rospy.sleep(2)
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/get_ready.mp3")
            # rospy.sleep(2)
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/pose.mp3")
            # rospy.sleep(2)
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/three.mp3")
            rospy.sleep(1.5)
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/two.mp3")
            rospy.sleep(1.5)
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/one.mp3")
            rospy.sleep(2.5)
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/qie_zi.mp3")
            # rospy.sleep(2)

            
            # 等待并获取ROS图像消息
            rospy.loginfo("等待相机图像消息...")
            image_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
            rospy.loginfo("成功获取图像!")
            
            # 保存图像
            cv2.imwrite("captured_image.jpg", cv_image)
            rospy.loginfo("图像已保存为 captured_image.jpg")
            
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
            self.sentence_buffer.append_text("好呀，和我握个手吧，很高兴认识你，我还想再多和你交流交流呢！")
            req = StringServiceRequest()
            req.request = 4 # TODO
            # self.arm_client.wait_for_service()
            resp = self.check_client.call(CheckRunStatusRequest())
            if resp.ros_run_flag is False:
                Thread(target=self.arm_client.call, args=(req,), daemon=True).start()

        elif intent == "LabTour":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行实验室游览动作")
            # mb_server = SRModbusSdk()
            # mb_server.connect_tcp('192.168.10.141')
            # mb_server.move_to_station_no(2, 1)
            # mb_server.wait_movement_task_finish(1) 
            # self.labTour()
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
            self.sentence_buffer.append_text("我叫南科盘古，")
            self.sentence_buffer.append_text("我是南方科技大学机器人研究院研发的第一款人形机器人，")
            self.sentence_buffer.append_text("我可以做一些简单的交互动作，")
            self.sentence_buffer.append_text("还可以陪你闲聊散心,")
            self.sentence_buffer.append_text("另外我还是实验室的科研小助手，")
            self.sentence_buffer.append_text("想知道今天的天气也可以问我哦！")
            
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
            self.intent_state = True
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
            self.intent_state = True
            rospy.loginfo(f"检测到 [{intent}] 意图, 讲述盘古开天地的故事")
            self.sentence_buffer.append_text("好的，盘古是中国古代传说时期中开天辟地的神。")
            self.sentence_buffer.append_text("在很久很久以前，宇宙混沌一团，盘古凭借着自己的神力把天地开辟出来了。")
            
        elif intent == "take_photo":
            self.intent_state = True
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行拍照动作")
            self.sentence_buffer.append_text("好的，没问题，大家都过来吧！站到我面前，让我来为大家拍一张大合照！")
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
            arm_req.request = random.choice([12, 13])
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



            
    # 替换原有的play_audio函数
    def enqueue_audio(self, file_path):
        """将音频文件加入播放队列（替代直接播放）"""
        self.audio_queue.put(file_path)


    def get_nlp_result(self, data):
        # 提取 text 字段
        text_value = data.get('content', {}).get(
            'result', {}).get('nlp', {}).get('text')

        status_value = data.get('content', {}).get(
            'result', {}).get('nlp', {}).get('status')

        if text_value is not None and status_value is not None:
            rospy.loginfo(f"大模型回答结果是: {text_value}  {status_value}")
        # 状态0: 新响应开始
        if status_value == 0:
            self.seen_status_0 = True  # 标记已见过状态0
            if self.intent_state == True:
                self.seen_status_0 = False  # 假装没有看见
            else:
                self.flush_all()  # 清空之前的缓冲区
                self.sentence_buffer.append_text(text_value)

        # 状态1: 中间段落
        elif status_value == 1:
            if self.intent_state == True:
                self.seen_status_0 = False  # 重置状态标志
            if  self.seen_status_0:
                self.sentence_buffer.append_text(text_value)

        # 状态2: 最终段落
        elif status_value == 2:
            if self.seen_status_0:
                self.sentence_buffer.append_text(text_value)
                self.seen_status_0 = False  # 重置状态标志
            elif self.detected_intent == "WHATTIME" or self.openQA or self.intent_state == True:
                self.flush_all()
                self.sentence_buffer.append_text(text_value)
                self.openQA = False
        
            
            
            if self.intent_state == True:
                # 如果是 intent-activated 状态，直接清空缓冲区
                # self.flush_all()
                self.intent_state = False
                rospy.loginfo("意图状态已重置")
            
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
            self.tts_queue.put(sentence)
            
            # 记录日志
            rospy.loginfo(f"合成句子: {sentence}")       
            
        
            

    def get_intent_result(self, data):
        text_value = data.get('content', {}).get(
            'result', {}).get('cbm_semantic', {}).get('text')
        rospy.loginfo(f"技能 text_value: {text_value} ")
        intent = json.loads(text_value)
        rc = intent['rc']
        if (rc == 0):
            category = intent.get('category', "")
            rospy.loginfo(f"技能结果: {category} ")
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
            rospy.loginfo(f"成功提取意图: {self.detected_intent}")
            self.intent_state = True  # 重置意图状态
            if self.detected_intent == "vla":
                try:
                    self.vla_text = parsed_data.get('text', "")
                    rospy.loginfo(f"技能 VLA 文本: {self.vla_text} ")
                except (IndexError, AttributeError, TypeError, KeyError) as e:
                    self.vla_text = ""
                    rospy.logwarn(f"语义 VLA 解析小异常: {str(e)}")
            
            if self.detected_intent == "vlm":
                try:
                    self.vlm_text = parsed_data.get('semantic', {})[0].get('template', "")
                    rospy.loginfo(f"技能 VLM 文本: {self.vlm_text} ")
                
                except (IndexError, AttributeError, TypeError, KeyError) as e:
                    self.vlm_text = ""
                    rospy.logwarn(f"语义 VLM 解析小异常: {str(e)}")

            if self.detected_intent in ["SayHi", "handshake", "LabTour", "Bow", "Nod", "vla", "vlm", "self_photo", "pangu", "take_photo", "LOVE"]:
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
                rospy.loginfo("No data received. Reconnecting...")
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
                # success, result = AiuiMessageProcess().process(self.client_socket, msg)
                # print(f"msg_type: {msg_type} ")
                # print(f"收到数据: {result} ")

                if msg_type == 0x01:
                    ConfirmProcess().process(self.client_socket, msg_id)
                elif msg_type == 0x04:
                    ConfirmProcess().process(self.client_socket, msg_id)
                    success, result = AiuiMessageProcess().process(self.client_socket, msg)
                    if success:
                        self.aiui_type = ""
                        data = json.loads(result)
                        self.get_aiui_type(data)
                        # print(f"AIUI message processed successfully: {result.decode('utf-8')}")

                        if data.get('content', {}).get('eventType', {}) == 5:
                            self.wakeup_state = False
                            rospy.loginfo(f"唤醒结束：==== 我不在 ==== ")
                            self.flush_all()  # 清空之前的缓冲区
                            self.sentence_buffer.append_text("我先退下啦！")
                            

                        if data.get('content', {}).get('eventType', {}) == 4:
                            
                            if self.wakeup_state == False:
                                self.wakeup_state = True
                                rospy.loginfo(f"唤醒成功：==== 我在 ==== ")
                                self.flush_all()  # 清空之前的缓冲区
                                self.sentence_buffer.append_text("我在！")
                                
                            
                        if (self.aiui_type == "iat"):
                            self.get_iat_result(data)

                        elif (self.aiui_type == "nlp"):
                            self.get_nlp_result(data)

                        elif (self.aiui_type == "cbm_semantic"):
                            # pass
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
    