#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import os
import json
import base64
import hashlib
import hmac
import threading
from urllib.parse import urlparse, urlencode
from time import mktime
from datetime import datetime
from wsgiref.handlers import format_date_time
import websocket
import ssl
import queue
import time
from std_msgs.msg import String
from aiui.srv import TTSV2, TTSV2Response
import _thread

class Ws_Param(object):
    def __init__(self, APPID, APIKey, APISecret, gpt_url):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.host = urlparse(gpt_url).netloc
        self.path = urlparse(gpt_url).path
        self.gpt_url = gpt_url

    def create_url(self):
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        signature_origin = "host: " + self.host + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + self.path + " HTTP/1.1"

        signature_sha = hmac.new(
            self.APISecret.encode('utf-8'),
            signature_origin.encode('utf-8'),
            digestmod=hashlib.sha256
        ).digest()

        signature_sha_base64 = base64.b64encode(signature_sha).decode(encoding='utf-8')

        authorization_origin = f'api_key="{self.APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha_base64}"'
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')

        v = {
            "authorization": authorization,
            "date": date,
            "host": self.host
        }
        url = self.gpt_url + '?' + urlencode(v)
        return url


class TTS_ROS_Server:
    def __init__(self):
        rospy.init_node('tts_service_node_v2', anonymous=True)
        
        # 从参数服务器获取配置
        self.appid = rospy.get_param('~appid', 'e1ac1785')
        self.api_key = rospy.get_param('~api_key', 'ee8edf3ac8a559787df7a819182e81aa')
        self.api_secret = rospy.get_param('~api_secret', 'NWZiZTg5Y2U5ZDZiNGU0M2YyZTVhMDI5')
        self.gpt_url = rospy.get_param('~gpt_url', 'wss://aiui.xf-yun.com/v2/aiint/ws')
        self.scene = rospy.get_param('~scene', 'IFLYTEK.hts')
        self.vcn = rospy.get_param('~vcn', 'x4_lingfeizhe_oral')
        self.speed = rospy.get_param('~speed', 50)
        self.volume = rospy.get_param('~volume', 50)
        self.pitch = rospy.get_param('~pitch', 50)
        self.audio_encode = rospy.get_param('~audio_encode', 'lame')
        
        # 音频文件保存路径
        self.save_dir = rospy.get_param('~save_dir', os.path.join(os.path.expanduser('~'), '/home/pangu/pangu/src/aiui', 'tts_audio'))
        
        # 确保保存目录存在
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir, exist_ok=True)
            rospy.loginfo(f"Created save directory: {self.save_dir}")
        
        # 创建TTS服务
        self.tts_service = rospy.Service('/tts_service/tts_v2', TTSV2, self.handle_tts_request)
        
        rospy.loginfo("TTS Server started, waiting for requests...")
        rospy.loginfo(f"Audio files will be saved to: {self.save_dir}")
    
    def on_message(self, ws, message):
        """收到websocket消息的处理"""
        try:
            data = json.loads(message)
            code = data['header']['code']
            status = data.get('header').get('status', -1)
            
            if code != 0:
                rospy.logerr(f"TTS request error: {code}, {message}")
                ws.result_queue.put(("error", f"Request error: {code}"))
                ws.close()
            else:
                payload_json = data.get('payload')
                if payload_json and payload_json.get('tts'):
                    audio_bs64 = payload_json.get('tts').get('audio')
                    if audio_bs64:
                        audio_raw = base64.b64decode(audio_bs64)
                        ws.save_fp.write(audio_raw)
                
                if status == 2:
                    ws.save_fp.flush()
                    ws.save_fp.close()
                    ws.result_queue.put(("success", ws.file_name))
                    ws.close()
        except Exception as e:
            rospy.logerr(f"Error processing message: {str(e)}")
            ws.result_queue.put(("error", str(e)))
            ws.close()
    
    def on_error(self, ws, error):
        """收到websocket错误的处理"""
        rospy.logerr(f"WebSocket error: {error}")
        ws.result_queue.put(("error", str(error)))
    
    def on_close(self, ws, one, two):
        """收到websocket关闭的处理"""
        rospy.loginfo("WebSocket connection closed")
    
    def on_open(self, ws):
        """收到websocket连接建立的处理"""
        rospy.loginfo("WebSocket connection opened, sending request...")
        _thread.start_new_thread(self.send_request, (ws,))
    
    def send_request(self, ws):
        """发送TTS请求"""
        data = json.dumps(self.gen_params(
            appid=ws.appid,
            question=ws.question,
            scene=ws.scene,
            vcn=ws.vcn,
            speed=ws.speed,
            volume=ws.volume,
            pitch=ws.pitch,
            audio_encode=ws.audio_encode
        ))
        rospy.logdebug(f"Sending TTS request: {data}")
        ws.send(data)
    
    def gen_params(self, appid, question, scene, vcn, speed, volume, pitch, audio_encode):
        """生成请求参数"""
        data = {
            "header": {
                "app_id": appid,
                "uid": "7f40c8462073c0911f696ae5577b2971",
                "stmid": "text-1",
                "status": 3,
                "scene": scene,
                "sn": "7f40c8462073c0911f696ae5577b2971"
            },
            "parameter": {
                "tts": {
                    "vcn": vcn,
                    "speed": speed,
                    "volume": volume,
                    "pitch": pitch,
                    "tts": {
                        "channels": 1,
                        "sample_rate": 16000,
                        "bit_depth": 16,
                        "encoding": audio_encode
                    }
                }
            },
            "payload": {
                "text": {
                    "compress": "raw",
                    "format": "plain",
                    "text": base64.b64encode(question.encode('utf-8')).decode('utf-8'),
                    "encoding": "utf8",
                    "status": 3
                }
            }
        }
        return data
    
    def synthesize_speech(self, text):
        """合成语音的主要函数"""
        try:
            # 生成唯一的文件名
            timestamp = int(time.time() * 1000)  # 毫秒级时间戳
            import hashlib
            text_hash = hashlib.md5(text.encode()).hexdigest()[:8]
            file_name = f"tts_{timestamp}_{text_hash}.mp3"
            file_path = os.path.join(self.save_dir, file_name)
            
            rospy.loginfo(f"Generating audio file: {file_path}")
            
            # 创建WebSocket参数
            ws_param = Ws_Param(self.appid, self.api_key, self.api_secret, self.gpt_url)
            ws_url = ws_param.create_url()
            
            # 创建结果队列
            result_queue = queue.Queue()
            
            # 创建WebSocket连接
            websocket.enableTrace(False)
            ws = websocket.WebSocketApp(
                ws_url,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close,
                on_open=self.on_open
            )
            
            # 设置WebSocket属性
            ws.appid = self.appid
            ws.scene = self.scene
            ws.vcn = self.vcn
            ws.speed = self.speed
            ws.volume = self.volume
            ws.pitch = self.pitch
            ws.question = text
            ws.audio_encode = self.audio_encode
            ws.file_name = file_name
            ws.result_queue = result_queue
            
            # 打开音频文件
            ws.save_fp = open(file_path, 'wb')
            
            # 创建线程运行WebSocket
            def run_ws():
                try:
                    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
                except Exception as e:
                    rospy.logerr(f"WebSocket run error: {str(e)}")
                    result_queue.put(("error", str(e)))
            
            ws_thread = threading.Thread(target=run_ws)
            ws_thread.daemon = True
            ws_thread.start()
            
            # 等待结果
            rospy.loginfo("Waiting for TTS synthesis to complete...")
            status, result = result_queue.get(timeout=30.0)  # 30秒超时
            
            if status == "success":
                rospy.loginfo(f"TTS synthesis completed successfully: {result}")
                return True, result, f"Audio saved to: {file_path}"
            else:
                rospy.logerr(f"TTS synthesis failed: {result}")
                # 清理失败的文件
                if os.path.exists(file_path):
                    os.remove(file_path)
                return False, "", f"TTS synthesis failed: {result}"
                
        except queue.Empty:
            rospy.logerr("TTS synthesis timeout")
            return False, "", "TTS synthesis timeout"
        except Exception as e:
            rospy.logerr(f"Error in TTS synthesis: {str(e)}")
            return False, "", f"Error: {str(e)}"
    
    def handle_tts_request(self, req):
        """处理TTS服务请求"""
        rospy.loginfo(f"Received TTS request: {req.tts_text[:50]}...")
        
        if not req.tts_text or req.tts_text.strip() == "":
            rospy.logwarn("Received empty text for TTS")
            return TTSV2Response("", False, "Text is empty")
        
        try:
            success, file_name, message = self.synthesize_speech(f"  {req.tts_text}")
            return TTSV2Response(file_name, success, message)
        except Exception as e:
            rospy.logerr(f"Exception in TTS service: {str(e)}")
            return TTSV2Response("", False, f"Exception: {str(e)}")
    
    def run(self):
        """运行ROS节点"""
        rospy.spin()


if __name__ == "__main__":
    try:
        tts_server = TTS_ROS_Server()
        tts_server.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("TTS Server shutdown")