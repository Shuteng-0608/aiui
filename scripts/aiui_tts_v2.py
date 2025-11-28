import _thread as thread
import base64
import datetime
import hashlib
import hmac
import json
from urllib.parse import urlparse
import time
from datetime import datetime
from time import mktime
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time
import pygame
import websocket
import wave
import io

# 配置信息
url = "wss://aiui.xf-yun.com/v2/aiint/ws"
appid = "e1ac1785"
api_key = "ee8edf3ac8a559787df7a819182e81aa"
api_secret = "NWZiZTg5Y2U5ZDZiNGU0M2YyZTVhMDI5"
# scene = "IFLYTEK.tts"
# scene = "IFLYTEK.hts" # 超拟人合成
scene = "main_box"
data_type = 'text'
question = "你好，这是一个测试语音合成的句子"

class AIUIV2WsClient(object):
    def __init__(self):
        # 初始化音频收集相关变量
        self.audio_chunks = []  # 存储所有音频片段
        self.is_complete = False  # 标记是否接收完成
        self.total_audio_data = b''  # 合并后的完整音频数据
        self.count = 0
        
        # 初始化pygame mixer
        pygame.mixer.init(frequency=16000, size=-16, channels=1)
        print("Pygame mixer初始化完成")
        self.handshake = self.assemble_auth_url(url)

    def assemble_auth_url(self, base_url):
        # 保持你原来的认证逻辑不变
        host = urlparse(base_url).netloc
        path = urlparse(base_url).path
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))

        signature_origin = "host: " + host + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + path + " HTTP/1.1"

        signature_sha = hmac.new(api_secret.encode('utf-8'), signature_origin.encode('utf-8'),
                                 digestmod=hashlib.sha256).digest()
        signature_sha_base64 = base64.b64encode(signature_sha).decode(encoding='utf-8')

        authorization_origin = f'api_key="{api_key}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha_base64}"'
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')

        v = {
            "host": host,
            "date": date,
            "authorization": authorization,
        }
        url = base_url + '?' + urlencode(v)
        return url

    def on_open(self, ws):
        print("### ws connect open")
        thread.start_new_thread(self.run, ())

    def run(self):
        if data_type == "text":
            self.text_req()

    def text_req(self):
        aiui_data = {
            "header": {
                "sn": "c4a5b45ef6da245ebdd111e4d69d17b3",
                "app_id": appid,
                "stmid": "text-1",
                "status": 3,
                "scene": scene,
            },
            "parameter": {
                "tts": {
                    # "vcn": "x2_xiaojuan",
                    "vcn": "x4_lingfeizhe_oral",
                    # "volume": "99",
                    "tts": {
                        "channels": 1,
                        "sample_rate": 16000,
                        "bit_depth": 16,
                        "encoding": "raw",
                        
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
        data = json.dumps(aiui_data)
        print('发送TTS请求:', question)
        self.ws.send(data)

    def collect_and_play_audio(self, pcm_data):
        """收集音频片段，等整句话完成后再播放"""
        # 添加到音频片段列表
        self.audio_chunks.append(pcm_data)
        print(f"收到音频片段，长度: {len(pcm_data)} 字节，累计片段数: {len(self.audio_chunks)}")
        
        # 如果已经标记完成，则合并所有片段并播放
        if self.is_complete:
            self.play_complete_audio()

    def play_complete_audio(self):
        """播放完整的音频"""
        if not self.audio_chunks:
            print("没有音频数据可播放")
            return
            
        # 合并所有音频片段
        self.total_audio_data = b''.join(self.audio_chunks)
        print(f"开始播放完整音频，总长度: {len(self.total_audio_data)} 字节，由 {len(self.audio_chunks)} 个片段组成")
        
        try:
            # 创建WAV格式的音频数据
            wav_buffer = io.BytesIO()
            with wave.open(wav_buffer, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)  # 16bit
                wav_file.setframerate(16000)
                wav_file.writeframes(self.total_audio_data)
            
            wav_buffer.seek(0)
            
            # 等待当前播放完成
            while pygame.mixer.music.get_busy():
                pygame.time.wait(50)
            
            # 播放完整音频
            pygame.mixer.music.load(wav_buffer)
            pygame.mixer.music.play()
            print("完整音频开始播放...")
            
            # 保存到文件（可选，用于调试）
            with open('complete_audio.wav', 'wb') as f:
                f.write(wav_buffer.getvalue())
            print("完整音频已保存到 complete_audio.wav")
            
        except Exception as e:
            print(f"播放完整音频时出错: {e}")

    def on_message(self, ws, message):

        data = json.loads(message)
        header = data['header']
        code = header['code']
        
        # print('收到消息，状态码:', code)
        
        if code != 0:
            print('请求错误:', code, json.dumps(data, ensure_ascii=False))
            ws.close()
            return

        if 'status' in header and header['status'] == 0:
            # self.count += 1
            print(f"开始接收TTS, {header['status']}")
        elif 'status' in header and header['status'] == 1:
            # self.count += 1
            print(f"接收TTS中, {header['status']}")
        elif 'status' in header and header['status'] == 2:
            # self.count += 1
            print(f"完成TTS接收, {self.count} - {header['status']}")
            # self.count = 0


        # 处理TTS音频数据
        if 'tts' in message:
            # print("收到TTS")
            self.count += 1
            tts_audio = data['payload']['tts']['audio']
            tts_content = base64.b64decode(tts_audio)
            
            # 收集音频片段
            self.collect_and_play_audio(tts_content)

        # 检查是否接收完成
        if 'status' in header and header['status'] == 2:
            print(f"所有音频片段接收完成，开始合并播放...{self.count}")
            self.count = 0
            self.is_complete = True
            self.play_complete_audio()
            
            # 等待播放完成后再关闭连接
            def wait_and_close():
                while pygame.mixer.music.get_busy():
                    pygame.time.wait(100)
                print("音频播放完成，关闭连接")
                ws.close()
            
            thread.start_new_thread(wait_and_close, ())

    def on_error(self, ws, error):
        print("### connection error: " + str(error))
        ws.close()

    def on_close(self, ws, close_status_code, close_msg):
        print("### connection is closed ###")
        # 清理资源
        if pygame.mixer.get_init():
            pygame.mixer.music.stop()
            pygame.mixer.quit()

    def start(self):
        self.ws = websocket.WebSocketApp(
            self.handshake,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close,
        )
        self.ws.run_forever()

if __name__ == "__main__":
    client = AIUIV2WsClient()
    client.start()