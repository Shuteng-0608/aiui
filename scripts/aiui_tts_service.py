#!/usr/bin/env python3

import _thread as thread
import base64
import datetime
import hashlib
import hmac
import json
from urllib.parse import urlparse
import time
from datetime import datetime, timezone
from urllib.parse import urlencode
import pygame
import websocket
import wave
import io
import queue  # 添加queue导入
from threading import Thread, Lock  # 添加Thread和Lock导入

import rospy
from aiui.srv import TTS, TTSResponse
from aiui.srv import TTSV2, TTSV2Response

class AIUIV2WsClient(object):
    def __init__(self):
        # 配置信息 - 使用您原始能工作的配置
        self.appid = 'e1ac1785'
        self.api_key = 'ee8edf3ac8a559787df7a819182e81aa'
        self.api_secret = 'NWZiZTg5Y2U5ZDZiNGU0M2YyZTVhMDI5'
        # self.appid = 'c79a8aff'
        # self.api_key = 'f477e270878c410e7f01f61ddeac83ac'
        # self.api_secret = 'NzRkMDY1MjEzOGU2ZTI4MmQxOTA5ZGY4'
        # self.scene = 'main_box'  # 使用原始的场景参数
        self.scene = "IFLYTEK.hts" # 超拟人合成
        # self.scene = "IFLYTEK.tts"
        self.url = "wss://aiui.xf-yun.com/v2/aiint/ws"
        
        # 状态变量
        self.audio_chunks = []
        self.is_complete = False
        self.total_audio_data = b''
        self.service_active = False
        self.current_text = ""
        self.conversion_success = False
        self.ws = None
        
        # 初始化pygame mixer
        try:
            pygame.mixer.init(frequency=16000, size=-16, channels=1)
            rospy.loginfo("Pygame mixer初始化完成")
        except Exception as e:
            rospy.logerr(f"Pygame mixer初始化失败: {e}")
        
        # 播放队列管理
        self.playback_queue = queue.Queue()
        self.is_playing = False
        self.playback_lock = Lock()
        
        # 启动播放管理线程
        Thread(target=self._playback_manager, daemon=True).start()
        rospy.loginfo("播放队列管理器已启动")
        
        # ROS service
        self.service = rospy.Service('text_to_speech', TTSV2, self.handle_tts_request)
        rospy.loginfo("TTS服务已注册")

    def _playback_manager(self):
        """播放管理线程 - 确保音频顺序播放"""
        rospy.loginfo("播放管理线程启动")
        while True:
            try:
                # 等待音频数据
                audio_data = self.playback_queue.get(timeout=1)
                rospy.loginfo("从播放队列获取音频数据，准备播放")
                
                with self.playback_lock:
                    self.is_playing = True
                    
                    # 播放音频
                    success = self._play_audio_data(audio_data)
                    
                    self.is_playing = False
                    self.playback_queue.task_done()
                    rospy.loginfo("音频播放完成")
                    
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"播放管理线程错误: {e}")
                self.is_playing = False

    def _play_audio_data(self, audio_data):
        """播放音频数据 - 替换原来的播放逻辑"""
        try:
            rospy.loginfo(f"开始播放音频，数据长度: {len(audio_data)} 字节")
            
            # 创建WAV格式的音频缓冲区
            wav_buffer = io.BytesIO()
            with wave.open(wav_buffer, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(16000)
                wav_file.writeframes(audio_data)
            
            wav_buffer.seek(0)
            
            # 等待前一个音频播放完成
            rospy.loginfo("等待前一个音频播放完成...")
            while pygame.mixer.music.get_busy():
                if rospy.is_shutdown():
                    return False
                pygame.time.wait(100)
            
            # 加载并播放新音频
            pygame.mixer.music.load(wav_buffer)
            pygame.mixer.music.play()
            rospy.loginfo("音频开始播放...")
            
            # 等待播放完成
            while pygame.mixer.music.get_busy():
                if rospy.is_shutdown():
                    pygame.mixer.music.stop()
                    return False
                pygame.time.wait(100)
            
            rospy.loginfo("音频播放完成")
            return True
            
        except Exception as e:
            rospy.logerr(f"播放音频数据失败: {e}")
            return False

    def assemble_auth_url(self, base_url):
        """生成认证URL - 使用原始能工作的逻辑"""
        try:
            host = urlparse(base_url).netloc
            path = urlparse(base_url).path
            
            # 使用UTC时间
            now = datetime.now(timezone.utc)
            date = now.strftime("%a, %d %b %Y %H:%M:%S GMT")
            
            # 构建签名原文
            signature_origin = "host: " + host + "\n"
            signature_origin += "date: " + date + "\n"
            signature_origin += "GET " + path + " HTTP/1.1"

            # 计算签名
            signature_sha = hmac.new(
                self.api_secret.encode('utf-8'), 
                signature_origin.encode('utf-8'),
                digestmod=hashlib.sha256
            ).digest()
            
            signature_sha_base64 = base64.b64encode(signature_sha).decode(encoding='utf-8')

            # 构建授权信息
            authorization_origin = f'api_key="{self.api_key}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha_base64}"'
            authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')

            v = {
                "authorization": authorization,
                "date": date,
                "host": host
            }
            
            auth_url = base_url + '?' + urlencode(v)
            rospy.loginfo("认证URL生成成功")
            return auth_url
            
        except Exception as e:
            rospy.logerr(f"生成认证URL失败: {e}")
            raise

    def handle_tts_request(self, req):
        """处理TTS service请求"""
        rospy.loginfo(f"收到TTS请求: {req.text}")
        
        if self.service_active:
            rospy.logwarn("TTS服务正忙，请稍后再试")
            return TTSResponse(False, "TTS服务正忙","")
        
        self.service_active = True
        success = False
        message = ""
        audio_file = ""  # 新增音频文件路径字段

        try:
            # 每次请求都重新生成认证URL
            self.handshake = self.assemble_auth_url(self.url)
            success = self.start_tts_conversion(req.text)
            message = "TTS转换完成" if success else "TTS转换失败"
            audio_file = "direct_playback"  # 由于直接播放，可以设置一个虚拟路径
        except Exception as e:
            rospy.logerr(f"TTS服务处理异常: {str(e)}")
            message = f"处理异常: {str(e)}"
        finally:
            self.service_active = False
            
        return TTSV2Response(success, message)

    def start_tts_conversion(self, text):
        """启动TTS转换过程"""
        self.audio_chunks = []
        self.is_complete = False
        self.total_audio_data = b''
        self.conversion_success = False
        self.current_text = text
        
        self.ws = websocket.WebSocketApp(
            self.handshake,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
        )
        
        def run_websocket():
            try:
                rospy.loginfo("启动WebSocket连接...")
                self.ws.run_forever()
            except Exception as e:
                rospy.logerr(f"WebSocket运行错误: {e}")
        
        thread.start_new_thread(run_websocket, ())
        
        timeout = 30
        start_time = time.time()
        
        while not self.is_complete:
            if time.time() - start_time > timeout:
                rospy.logwarn("TTS转换超时")
                if self.ws:
                    self.ws.close()
                break
            time.sleep(0.1)
            
            if rospy.is_shutdown():
                if self.ws:
                    self.ws.close()
                break
        
        return self.conversion_success

    def on_open(self, ws):
        rospy.loginfo("WebSocket连接已建立")
        self.send_tts_request()

    def send_tts_request(self):
        """发送TTS请求 - 使用原始能工作的格式"""
        if not self.current_text:
            rospy.logerr("没有要转换的文本")
            return
            
        try:
            # 使用您原始能工作的请求格式
            aiui_data = {
                "header": {
                    "sn":"c4a5b45ef6da245ebdd111e4d69d17b3",
                    "app_id": self.appid,
                    "stmid": "text-1",
                    "status": 3,
                    "scene": self.scene,  # 使用原始的场景参数
                },
                "parameter": {
                    "tts": {
                        "vcn": "x4_lingfeizhe_oral",  # 使用原始的音色
                        #"vcn": "x2_xiaojuan",
                        "tts": {
                            "channels": 1,
                            "sample_rate": 16000,
                            "bit_depth": 16,
                            "encoding": "raw"
                        }
                    }
                },
                "payload": {
                    "text": {
                        "compress": "raw",
                        "format": "plain",
                        "text": base64.b64encode(self.current_text.encode('utf-8')).decode('utf-8'),
                        "encoding": "utf8",
                        "status": 3
                    }
                }
            }
            
            data = json.dumps(aiui_data)
            rospy.loginfo(f'发送TTS请求: {self.current_text}')
            rospy.loginfo(f'请求数据: {json.dumps(aiui_data, indent=2, ensure_ascii=False)}')
            # rospy.loginfo(data)
            self.ws.send(data)
            
        except Exception as e:
            rospy.logerr(f"发送TTS请求失败: {e}")

    def on_message(self, ws, message):
        try:
            data = json.loads(message)
            rospy.loginfo(f'收到WebSocket消息')
            # rospy.loginfo(f'消息内容: {json.dumps(data, indent=2, ensure_ascii=False)}')
            
            header = data.get('header', {})
            code = header.get('code', -1)
            
            if code != 0:
                error_msg = data.get('message', '未知错误')
                rospy.logerr(f'API错误: {code}, 消息: {error_msg}')
                
                # 详细调试信息
                rospy.logerr(f"完整响应: {json.dumps(data, indent=2, ensure_ascii=False)}")
                
                self.conversion_success = False
                self.is_complete = True
                return
            
            # 处理音频数据
            if 'payload' in data and 'tts' in data['payload']:
                tts_audio = data['payload']['tts']['audio']
                tts_content = base64.b64decode(tts_audio)
                self.audio_chunks.append(tts_content)
                rospy.loginfo(f"收到音频片段，累计: {len(self.audio_chunks)}个片段")

            # 检查是否完成
            if header.get('status') == 2:
                rospy.loginfo("所有音频片段接收完成")
                self.is_complete = True
                self.conversion_success = self.play_complete_audio()
                if self.ws:
                    self.ws.close()
                
        except Exception as e:
            rospy.logerr(f"处理WebSocket消息错误: {e}")
            self.conversion_success = False
            self.is_complete = True

    def play_complete_audio(self):
        """将音频数据加入播放队列（替换原来的直接播放）"""
        if not self.audio_chunks:
            rospy.logwarn("没有音频数据可播放")
            return False
            
        try:
            self.total_audio_data = b''.join(self.audio_chunks)

            # rospy.loginfo(f"生成的完整音频：{self.total_audio_data}")
            rospy.loginfo(f"音频数据生成完成，长度: {len(self.total_audio_data)} 字节，加入播放队列")
            
            # 将音频数据加入播放队列（而不是直接播放）
            self.playback_queue.put(self.total_audio_data)
            rospy.loginfo("音频数据已加入播放队列，等待顺序播放")
            
            # 立即返回成功，播放由播放管理线程处理
            return True
            
        except Exception as e:
            rospy.logerr(f"处理音频数据失败: {e}")
            return False

    def on_error(self, ws, error):
        rospy.logerr(f"WebSocket错误: {error}")
        self.conversion_success = False
        self.is_complete = True

    def on_close(self, ws, close_status_code, close_msg):
        rospy.loginfo("WebSocket连接关闭")
        self.is_complete = True

    def cleanup(self):
        """清理资源"""
        if self.ws:
            try:
                self.ws.close()
            except:
                pass
        if pygame.mixer.get_init():
            pygame.mixer.music.stop()
            pygame.mixer.quit()

def main():
    rospy.init_node('aiui_tts_service')
    rospy.loginfo("启动AIUI TTS服务节点...")
    
    try:
        tts_client = AIUIV2WsClient()
        rospy.on_shutdown(tts_client.cleanup)
        rospy.loginfo("TTS服务节点准备就绪")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS中断")
    except Exception as e:
        rospy.logerr(f"TTS服务节点异常: {e}")
    finally:
        rospy.loginfo("TTS服务节点关闭")

if __name__ == "__main__":
    main()