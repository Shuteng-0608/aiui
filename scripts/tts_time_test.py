#!/usr/bin/env python

import rospy
from aiui.srv import TTS, TTSRequest
import time
import requests
import pygame
start_time = 0
def play_audio(file_path):
    pygame.mixer.init()
    pygame.mixer.music.load(file_path)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():  # 等待播放结束
        pygame.time.Clock().tick(10)

def download_and_play_tts(url):
    global start_time
    try:
        # 下载MP3文件
        response = requests.get(url)
        response.raise_for_status()  # 检查请求是否成功
        
        # 保存临时文件
        temp_file = "temp_tts.mp3"
        
        with open(temp_file, 'wb') as f:
            f.write(response.content)
        
        rospy.loginfo(f"已下载TTS音频到: {temp_file}")
        rospy.loginfo(f"TTS请求耗时: {time.time() - start_time:.2f} 秒")
        
        # 播放音频
        # playsound(temp_file)
        play_audio(temp_file)
        # 删除临时文件（可选）
        # os.remove(temp_file)
        
    except Exception as e:
        rospy.logerr(f"下载或播放音频时出错: {e}")
def tts_client(text_to_speak):
    global start_time
    # 等待服务可用
    rospy.wait_for_service('/tts_service/generator')
    try:
        # 创建服务代理
        tts_proxy = rospy.ServiceProxy('/tts_service/generator', TTS)
        
        # 创建请求对象并设置文本
        start_time = time.time()
        request = TTSRequest()
        request.request = text_to_speak
        
        # 调用服务
        rospy.loginfo(f"发送TTS请求: '{text_to_speak}'")

        response = tts_proxy(request)
        
        # 处理响应
        rospy.loginfo(f"TTS服务响应: {response.response}")
        rospy.loginfo(f"TTS音频URL: {response.tts_url}")
        download_and_play_tts(response.tts_url)

        
        return response
        
    except rospy.ServiceException as e:
        rospy.logerr(f"服务调用失败: {str(e)}")
        return None

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("tts_client_node")
    
    # 示例文本
    texts_to_speak = [
        "你好，我是ROS机器人",
        "当前时间是2025年10月16日",
        "天气晴朗，温度25摄氏度",
        "系统运行正常，所有传感器在线"
    ]
    
    # 循环发送TTS请求
    for text in texts_to_speak:
        rospy.loginfo(f"\n=== 准备播报: '{text}' ===")
        response = tts_client(text)
        
        # 等待当前播报完成
        # rospy.sleep(1)  # 根据实际音频长度调整
        
        # 检查是否需要退出
        if rospy.is_shutdown():
            break
    
    rospy.loginfo("所有TTS请求完成")