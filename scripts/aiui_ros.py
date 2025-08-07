#! /usr/bin/env python3
import struct
import logging
import time
import json
from socket import *
import signal
from processStrategy import AiuiMessageProcess, ConfirmProcess
from threading import Thread, Event
import threading
import subprocess
from sr_modbus_sdk import SRModbusSdk
import rospy
from aiui.srv import TTS, TTSRequest, TTSResponse
from aiui.srv import VLMProcess, VLMProcessRequest, VLMProcessResponse
from aiui.srv import StringService, StringServiceRequest, StringServiceResponse
from aiui.srv import DH5SetPosition, DH5SetPositionRequest, DH5SetPositionResponse
from aiui.srv import VLAProcess, VLAProcessRequest, VLAProcessResponse
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import pygame

def stop_handle(sig, frame):
    global run
    run = False


signal.signal(signal.SIGINT, stop_handle)
run = True

# 配置日志记录
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# 文件日志处理器
file_handler = logging.FileHandler('socket_demo.log')
file_handler.setLevel(logging.INFO)
file_formatter = logging.Formatter(
    '%(asctime)s %(levelname)s:%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
file_handler.setFormatter(file_formatter)

# 终端日志处理器
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
console_formatter = logging.Formatter(
    '%(asctime)s %(levelname)s:%(message)s', datefmt='%Y-%m-%d %H:%M:%S')
console_handler.setFormatter(console_formatter)

# 添加处理器到记录器
# logger.addHandler(file_handler)
logger.addHandler(console_handler)

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
        self.tts_client = rospy.ServiceProxy("/tts_service/player",TTS)
        self.dh5_client = rospy.ServiceProxy("/dh5/set_all_position",DH5SetPosition)
        self.vla_client = rospy.ServiceProxy("vla_service", VLAProcess)
        self.pending_response = []  # 暂存中间段(状态1)的文本
        self.current_response = ""  # 当前拼接中的完整回复
        self.seen_status_0 = False  # 标记是否见过状态0
        self.vlm_state = False  # VLM状态标志
        self.vlm_text = ""  # VLM文本
        self.vla_text = ""
        self.audio_thread = None  # 用于播放音频的线程


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


    def labTour(self, start, end):
        mb_server = SRModbusSdk()
        mb_server.connect_tcp('192.168.10.141')
        for i in range(start, end+1):
            mb_server.move_to_station_no(i, 1)
            mb_server.wait_movement_task_finish(1) 

    def thake_photo(self):
        bridge = CvBridge()
    
        try:
            rospy.sleep(0.8)
            self.play_audio("/home/whc/aiui_ws/src/aiui/audio/look_at_me.mp3")
            # rospy.sleep(2)
            self.play_audio("/home/whc/aiui_ws/src/aiui/audio/get_ready.mp3")
            # rospy.sleep(2)
            self.play_audio("/home/whc/aiui_ws/src/aiui/audio/pose.mp3")
            # rospy.sleep(2)
            self.play_audio("/home/whc/aiui_ws/src/aiui/audio/three.mp3")
            rospy.sleep(1.5)
            self.play_audio("/home/whc/aiui_ws/src/aiui/audio/two.mp3")
            rospy.sleep(1.5)
            self.play_audio("/home/whc/aiui_ws/src/aiui/audio/one.mp3")
            rospy.sleep(2.5)
            self.play_audio("/home/whc/aiui_ws/src/aiui/audio/qie_zi.mp3")
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
        if intent == "SayHi":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行打招呼动作")
            # client = rospy.ServiceProxy("cmd_str_srv",StringService)
            # self.arm_client = rospy.ServiceProxy("/aris_node/cmd_str_srv",StringService)
            req = StringServiceRequest()
            req.request = '3'
            self.arm_client.wait_for_service()
            Thread(target=self.arm_client.call, args=(req,), daemon=True).start()

            # client.close()
        elif intent == "handshake":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行握手动作")
            # client = rospy.ServiceProxy("cmd_str_srv",StringService)

            req = StringServiceRequest()
            req.request = '4' # TODO
            self.arm_client.wait_for_service()

            # dh5_req = DH5SetPositionRequest()
            # dh5_req.hand_type = 'right'  # 1 for right hand, 2 for left hand
            # dh5_req.position_list = [800, 1500, 1500, 1500, 1500, 800]  # TODO for handshake position
            # dh5_req.hand_mode = 'hand'
            # self.dh5_client.wait_for_service()
            # Thread(target=self.dh5_client.call, args=(dh5_req,), daemon=True).start()
            Thread(target=self.arm_client.call, args=(req,), daemon=True).start()

            # client.close()
        elif intent == "LabTour":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行实验室游览动作")
            # mb_server = SRModbusSdk()
            # mb_server.connect_tcp('192.168.10.141')
            # mb_server.move_to_station_no(2, 1)
            # mb_server.wait_movement_task_finish(1) 
            # self.labTour()
        elif intent == "Bow":
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行鞠躬欢送动作")
            req = StringServiceRequest()
            req.request = '5' # TODO
            self.arm_client.wait_for_service()
            Thread(target=self.arm_client.call, args=(req,), daemon=True).start()

 
        # elif intent == "Nod":
        #     print(f"检测到 [{intent}] 意图, 执行点头动作")
        #     # client = rospy.ServiceProxy("cmd_str_srv",StringService)
        #     req = StringServiceRequest()
        #     req.request = '2' # TODO
        #     self.arm_client.wait_for_service()
        #     # self.arm_client.call(req)
        #     Thread(target=self.arm_client.call, args=(req,), daemon=True).start()
        
        elif intent == "vla":
            self.vlm_state = True
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行视觉语言动作")
            tts_req = TTSRequest()
            tts_req.request = "好的，没问题！"
            self.tts_client.wait_for_service()
            Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()

            # client = rospy.ServiceProxy("tts_service",TTS)
            # client.wait_for_service()
            # client.call(tts_req)
            # client.close()

            vla_req = VLAProcessRequest()
            vla_req.prompt = self.vlm_text
            self.vla_client.wait_for_service()
            Thread(target=self.vla_client.call, args=(vla_req,), daemon=True).start()
        
        elif intent == "vlm":
            self.vlm_state = True
            self.tts_text = "好的，让我仔细看一下"
            req = TTSRequest()
            req.request = self.tts_text
            self.tts_client.wait_for_service()
            Thread(target=self.tts_client.call, args=(req,), daemon=True).start()

            rospy.loginfo(f"检测到 [{intent}] 意图, 执行描述动作")
            vlm_req = VLMProcessRequest()
            vlm_req.prompt = self.vlm_text
            self.vlm_client.wait_for_service()
            resp = self.vlm_client.call(vlm_req)
            vlm_result = resp.vlm_result
            rospy.loginfo(f"VLM 结果: {vlm_result}")
            
            # 语音合成 VLM 结果
            # client = rospy.ServiceProxy("tts_service",TTS)
            req = TTSRequest()
            req.request = vlm_result
            self.tts_client.wait_for_service()
            self.tts_client.call(req)

        elif intent == "self_photo":
            self.vlm_state = True
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行自拍动作")
            tts_req = TTSRequest()
            tts_req.request = "好的，摆个点赞的姿势，来和我自拍一张吧"
            self.tts_client.wait_for_service()
            self.tts_client.call(tts_req)
            arm_req = StringServiceRequest()
            arm_req.request = '6'
            self.arm_client.wait_for_service()
            Thread(target=self.arm_client.call, args=(arm_req,), daemon=True).start()

        elif intent == "pangu":
            self.vlm_state = True
            rospy.loginfo(f"检测到 [{intent}] 意图, 讲述盘古开天地的故事")
            tts_req = TTSRequest()
            tts_req.request = "好的, 盘古是中国古代传说时期中开天辟地的神。在天地还没有开辟以前，宇宙混沌一团，盘古凭借着自己的神力把天地开辟出来了。他的左眼变成了太阳，右眼变成了月亮；头发和胡须变成了夜空的星星；他的身体变成了东、西、南、北四极和雄伟的三山五岳；血液变成了江河；牙齿、骨骼和骨髓变成了地下矿藏；皮肤和汗毛变成了大地上的草木；汗水变成了雨露"
            self.tts_client.wait_for_service()
            Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()
            # self.play_audio("/home/whc/aiui_ws/src/aiui/audio/pangu.mp3")
            # audio_thread = threading.Thread(target=self.play_audio, args=("/home/whc/aiui_ws/pangu.mp3",), daemon=True)
            # self.audio_thread = Thread(target=self.play_audio, args=("/home/whc/aiui_ws/pangu.mp3",), daemon=True)
            # self.audio_thread.start()
            
        elif intent == "take_photo":
            self.vlm_state = True
            rospy.loginfo(f"检测到 [{intent}] 意图, 执行拍照动作")
            tts_req = TTSRequest()
            tts_req.request = "好的，没问题，大家都过来吧！站到我面前，让我来为大家拍一张大合照！"
            self.tts_client.wait_for_service()
            self.tts_client.call(tts_req)
            arm_req = StringServiceRequest()
            arm_req.request = '7'
            self.arm_client.wait_for_service()
            Thread(target=self.arm_client.call, args=(arm_req,), daemon=True).start()
            
            self.thake_photo()
            # Thread(target=self.play_audio, args=("/home/whc/aiui_ws/src/aiui/audio/take_photo.mp3",), daemon=True).start()

            


        
        
            
    def play_audio(self, file_path):
        pygame.mixer.init()
        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():  # 等待播放结束
            pygame.time.Clock().tick(10)


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
            # 重置之前的进度
            self.current_response = text_value
            self.pending_response = []
            self.seen_status_0 = True
            rospy.loginfo(f"新响应开始: {text_value}")

        # 状态1: 中间段落
        elif status_value == 1:
            if not self.seen_status_0:
                rospy.logwarn("收到状态1但未收到初始状态0, 忽略")
                return
            self.pending_response.append(text_value)

        # 状态2: 最终段落
        elif status_value == 2:
            # if not self.seen_status_0:
            #     rospy.logwarn("收到状态2但未收到初始状态0, 忽略")
            #     return
            # 拼接所有中间段落 + 最后段落
            full_text = self.current_response + "".join(self.pending_response) + text_value
            rospy.loginfo(f"最终完整响应: {full_text}")
            self.tts_text = full_text
            if self.tts_text and self.vlm_state == False:
                req = TTSRequest()
                req.request = self.tts_text
                self.tts_client.wait_for_service()
                Thread(target=self.tts_client.call, args=(req,), daemon=True).start()
            if self.vlm_state == True:
                # self.audio_thread.join(timeout=0.5)  # 停止播放盘古故事音频
                self.vlm_state = False

            # 重置状态
            self.current_response = ""
            self.pending_response = []
            self.seen_status_0 = False
            return full_text
        
            # # parsed_data = json.loads(text_value)
            # # ============================================================================== #
            # if type(text_value) == 'str':
            #     rospy.loginfo(f"接收到的 text_value 是字符串: {text_value}")
                
            # try:
            #     parsed_data = json.loads(text_value)  # 尝试解析JSON
            # except json.JSONDecodeError as e:  # 捕获JSON解析错误
            #     # 处理解析失败的情况（例如记录错误/返回默认值）
            #     print(f"JSON解析失败: {e}")
            #     parsed_data = None  # 或 {} / [] 根据预期结构
            # except TypeError as e:  # 捕获非字符串输入
            #     print(f"输入类型错误: {e}")
            #     parsed_data = None
            # except Exception as e:  # 可选：处理其他未知异常
            #     print(f"未知错误: {e}")
            #     parsed_data = None
            # if parsed_data is None:
            #     rospy.logerr("解析后的数据为 None")
            #     rospy.logwarn(f"解析前 text_value 的数据: {text_value}")
            #     rospy.logwarn(f"解析前 text_value 的数据类型为: {type(text_value)}")
            #     return
            # # ============================================================================== #
           

            # if not isinstance(parsed_data, dict):
            #     rospy.logerr("解析后的数据不是字典格式")
            #     return

            # try:
            #     self.tts_text = parsed_data.get('intent', {}).get('answer',{}).get('text')
            # except (IndexError, AttributeError, TypeError, KeyError) as e:
            #     # self.detected_intent = None
            #     self.tts_text = ""
            #     rospy.logerr(f"语义解析小异常: {str(e)}")

            # try:
            #     self.detected_intent = parsed_data.get('intent', {}).get('semantic', {})[0].get('intent', {})
            # except (IndexError, AttributeError, TypeError, KeyError) as e:
            #     self.detected_intent = None
            #     rospy.logwarn(f"无意图: {str(e)}")
            
            # if self.tts_text:
            #     rospy.loginfo(f"成功提取回答: {self.tts_text}")
            #     # client = rospy.ServiceProxy("tts_service",TTS)
            #     req = TTSRequest()
            #     req.request = self.tts_text
            #     self.tts_client.wait_for_service()

            #     # self.arm_client.call(req)
            #     Thread(target=self.tts_client.call, args=(req,), daemon=True).start()
            #     # client.close()

            # else:
            #     rospy.logerr(f"未成功提取回答")

            # if self.detected_intent:
            #     rospy.loginfo(f"成功提取意图: {self.detected_intent}")
            #     self.handle_detected_intent(self.detected_intent)
            # else:
            #     rospy.logwarn("未检测到预设动作指令意图")

            

    def get_intent_result(self, data):
        # intent_data = data.get('content', {}).get('result', {}).get()
        text_value = data.get('content', {}).get(
            'result', {}).get('cbm_semantic', {}).get('text')
        rospy.loginfo(f"技能 text_value: {text_value} ")
        # self.vlm_text = data.get('content', {}).get('result', {}).get('cbm_semantic', {}).get('text').get('semantic', {}).get('template')
        # print(f"技能 VLM 文本: {self.vlm_text} ")
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
            self.vlm_text = parsed_data.get('semantic', {})[0].get('template', "")
            rospy.loginfo(f"技能 VLM 文本: {self.vlm_text} ")
        except (IndexError, AttributeError, TypeError, KeyError) as e:
            self.vlm_text = ""
            rospy.logerr(f"语义vlm解析小异常: {str(e)}")
        try:
            self.detected_intent = parsed_data.get('semantic', {})[0].get('intent', {})

        except (IndexError, AttributeError, TypeError, KeyError) as e:
            self.detected_intent = None
            rospy.logwarn(f"无意图: {str(e)}")
        if self.detected_intent:
            rospy.loginfo(f"成功提取意图: {self.detected_intent}")
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
                            client = rospy.ServiceProxy("tts_service",TTS)
                            req = TTSRequest()
                            req.request = "我先退下啦！"
                            client.wait_for_service()
                            client.call(req)
                            client.close()

                        if data.get('content', {}).get('eventType', {}) == 4:
                            
                            if self.wakeup_state == False:
                                self.wakeup_state = True
                                rospy.loginfo(f"唤醒成功：==== 我在 ==== ")
                                client = rospy.ServiceProxy("tts_service",TTS)
                                req = TTSRequest()
                                req.request = "我在！"
                                client.wait_for_service()
                                client.call(req)
                                client.close()
                            
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
    