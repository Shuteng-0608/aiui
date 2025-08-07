#!/usr/bin/env python3

import rospy
import os
import sys
import time
import random
import threading
import cv2
import numpy as np
import signal
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from playsound import playsound
from threading import Thread

from aiui.srv import TTS, TTSRequest
from aiui.srv import VLMProcess, VLMProcessRequest
from aiui.srv import StringService, StringServiceRequest
from aiui.srv import DH5SetPosition, DH5SetPositionRequest
from aiui.srv import VLAProcess, VLAProcessRequest
from sentenceBuffer import SentenceBuffer

import tty
import termios
import select
import fcntl
import struct

# 全局变量用于控制程序运行
running = True

class TerminalManager:
    def __init__(self):
        # 保存原始终端设置
        self.original_settings = termios.tcgetattr(sys.stdin)
        # 终端激活状态
        self.is_terminal_active = True
        
    def configure_terminal(self):
        """配置终端为原始模式"""
        if sys.stdout.isatty():
            tty.setraw(sys.stdin.fileno())
            
    def restore_terminal(self):
        """恢复终端原始设置"""
        if sys.stdout.isatty():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_settings)
            
    def read_key(self):
        """非阻塞读取按键"""
        try:
            # 设置非阻塞模式
            old_flags = fcntl.fcntl(sys.stdin, fcntl.F_GETFL)
            fcntl.fcntl(sys.stdin, fcntl.F_SETFL, old_flags | os.O_NONBLOCK)
            
            try:
                # 尝试读取按键
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    return key
            finally:
                # 恢复阻塞模式
                fcntl.fcntl(sys.stdin, fcntl.F_SETFL, old_flags)
        except Exception as e:
            rospy.logerr(f"读取按键错误: {e}")
        return None
    
    def update_terminal_state(self):
        """更新终端状态"""
        if self.read_key() is not None:
            self.is_terminal_active = True
        else:
            self.is_terminal_active = False
            
    def display_status(self):
        """显示终端状态"""
        if not sys.stdout.isatty():
            return
            
        # 保存当前光标位置
        print("\033[s", end="")
        
        # 移动到状态行
        print("\033[30;1H", end="")
        
        # 清除状态行
        print("\033[2K", end="")
        
        # 打印新状态
        if self.is_terminal_active:
            print("\033[1;32m>> 终端已激活 (按键有效)\033[0m")
        else:
            print("\033[1;31m>> 终端未激活 (按键无效)\033[0m")
        
        # 恢复光标位置
        print("\033[u", end="")
        sys.stdout.flush()

class ExtendedKeyboardActionNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('extended_keyboard_action_printer', anonymous=True)
        
        # 初始化终端管理器
        self.terminal = TerminalManager()
        self.terminal.configure_terminal()
        
        # 初始化服务客户端
        self.arm_client = rospy.ServiceProxy("aris_node/cmd_str_srv", StringService)
        self.vlm_client = rospy.ServiceProxy("vlm_service", VLMProcess)
        self.tts_client = rospy.ServiceProxy("/tts_service/player", TTS)
        self.dh5_client = rospy.ServiceProxy("/dh5/set_all_position", DH5SetPosition)
        self.vla_client = rospy.ServiceProxy("vla_service", VLAProcess)
        
        # 设置退出信号处理
        rospy.on_shutdown(self.shutdown_hook)
        
        # 显示初始菜单
        self.display_menu()
        
        # 启动终端状态更新线程
        self.terminal_thread = threading.Thread(target=self.update_terminal_status)
        self.terminal_thread.daemon = True
        self.terminal_thread.start()
        
        rospy.loginfo(f"扩展键盘动作监听器已启动 (节点: {rospy.get_name()})")

    def update_terminal_status(self):
        """持续更新终端状态"""
        while running and not rospy.is_shutdown():
            self.terminal.update_terminal_state()
            self.terminal.display_status()
            time.sleep(0.2)  # 每0.2秒检查一次

    def display_menu(self):
        """显示选项菜单"""
        if not sys.stdout.isatty():
            return
            
        # 清屏
        os.system('clear')
        
        # 打印标题
        print("\033[1;36m")  # 青色
        print("     人形机器人交互控制台")
        print("==============================")
        print("\033[0m")  # 重置颜色
        
        # 系统信息
        print(f"节点名称: {rospy.get_name()}")
        print("功能: 通过键盘触发机器人动作意图")
        print("键盘输入仅在终端处于焦点时生效\n")
        print("按以下键执行相应动作：\n")
        
        # 设置列宽（每列30字符）
        col_width = 30
        
        # 动作列表（三列布局）
        actions = [
            # 第一列
            ("2", "👍 点头介绍(Nod)"),
            ("3", "👋 打招呼(SayHi)"),
            ("4", "🤝 握手(handshake)"),
            ("5", "🙇 鞠躬(Bow)"),
            
            # 第二列
            ("6", "🤳 自拍(pose)"),
            ("7", "📷 拍照(take_photo)"),
            ("8", "👏 鼓掌(handclap)"),
            ("9", "💖 比心(LOVE)"),
            
            # 第三列
            ("p", "📖 盘古故事(pangu)"),
            ("a", "👁️ 视觉语言动作(vla)"),
            ("m", "🖼️ 视觉描述(vlm)"),
            ("0", "🚪 退出程序")
        ]
        
        # 分三列打印动作
        header = [
            "\033[1;33m[按键] 动作描述       \033[0m".ljust(col_width),
            "\033[1;33m[按键] 动作描述       \033[0m".ljust(col_width),
            "\033[1;33m[按键] 动作描述       \033[0m".ljust(col_width)
        ]
        
        # 打印表头
        print(" | ".join(header))
        
        # 打印分隔线
        print("-" * (col_width * 3 + 6))
        
        # 打印动作行
        for i in range(0, len(actions), 3):
            row_items = []
            for j in range(3):
                idx = i + j
                if idx < len(actions):
                    key, desc = actions[idx]
                    if key == "0":
                        color = "\033[1;31m"  # 退出项用红色
                    else:
                        color = "\033[1;36m" if j == 0 else "\033[1;32m" if j == 1 else "\033[1;35m"
                    
                    item = f"{color}[{key}]\033[0m {desc}".ljust(col_width)
                    row_items.append(item)
                else:
                    row_items.append("".ljust(col_width))
            
            # 打印一行三列
            print(" | ".join(row_items))
        
        # 添加操作说明
        print("\n\033[1;33m操作指南:\033[0m")
        print("1. 按菜单中的按键执行相应动作")
        print("2. 部分动作有随机效果（握手、鞠躬等会有不同表现）")
        print("3. 拍照功能会同时触发动作和相机操作")
        print("\n\033[1;31m退出方法: 按 0 或 Ctrl+C\033[0m")
        print("\033[1;32m提示: 键盘输入仅在终端窗口激活时有效\033[0m\n")
        
        # 状态行位置预留
        print("\033[30;1H", end="")  # 移动到第30行第一列
        print("\033[2K", end="")  # 清除该行
        print("\033[1;33m>> 当前状态: 正在初始化...\033[0m")

    def handle_detected_intent(self, key):
        """处理检测到的按键"""
        if not self.terminal.is_terminal_active:
            rospy.logdebug("终端未激活，忽略按键输入")
            return
            
        if key == '2':        
            rospy.loginfo(f"检测到 点头介绍(Nod) 意图")
            self.execute_nod()
        elif key == '3':      
            rospy.loginfo(f"检测到 打招呼(SayHi) 意图")
            self.execute_sayhi()
        elif key == '4':      
            rospy.loginfo(f"检测到 握手(handshake) 意图")
            self.execute_handshake()
        elif key == '5':      
            rospy.loginfo(f"检测到 鞠躬(Bow) 意图")
            self.execute_bow()
        elif key == '6':      
            rospy.loginfo(f"检测到 自拍(pose) 意图")
            self.execute_pose()
        elif key == '7':      
            rospy.loginfo(f"检测到 拍照(take_photo) 意图")
            self.execute_take_photo()
        elif key == '8':      
            rospy.loginfo(f"检测到 鼓掌(handclap) 意图")
            self.execute_handclap()
        elif key == '9':      
            rospy.loginfo(f"检测到 比心(LOVE) 意图")
            self.execute_love()
        elif key == 'p':      
            rospy.loginfo(f"检测到 盘古故事(pangu) 意图")
            self.execute_pangu()
        elif key == 'a':      
            rospy.loginfo(f"检测到 视觉语言动作(vla) 意图")
            self.execute_vla()
        elif key == 'm':      
            rospy.loginfo(f"检测到 视觉描述(vlm) 意图")
            self.execute_vlm()
        elif key == '0':  # 0键用于退出
            print("\n\033[1;31m>>> 接收到退出命令，关闭节点...\033[0m")
            global running
            running = False
            rospy.signal_shutdown("用户退出")

    def execute_sayhi(self):
        """执行打招呼动作"""
        tts_req = TTSRequest()
        tts_req.request = "你好呀，欢迎您来到南方科技大学机器人研究院，很高兴见到您！"
        Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()
        req = StringServiceRequest()
        req.request = 3
        Thread(target=self.arm_client.call, args=(req,), daemon=True).start()

    def execute_handshake(self):
        """执行握手动作"""
        tts_req = TTSRequest()
        tts_req.request = "好呀，和我握个手吧，很高兴认识你，我还想再多和你交流交流呢！"
        Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()
        req = StringServiceRequest()
        req.request = random.choice([4, 14, 15])  # 随机选择握手动作
        Thread(target=self.arm_client.call, args=(req,), daemon=True).start()

    def execute_bow(self):
        """执行鞠躬动作"""
        tts_req = TTSRequest()
        tts_req.request = "哇时间过得好快, 再见喽，期待下次再和您见面，记得要常来看我哦！"
        Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()
        req = StringServiceRequest()
        req.request = random.choice([5, 16, 17])  # 随机选择鞠躬动作
        Thread(target=self.arm_client.call, args=(req,), daemon=True).start()

    def execute_nod(self):
        """执行点头介绍动作"""
        tts_req = TTSRequest()
        tts_req.request = "我叫南科盘古，我是南方科技大学机器人研究院研发的第一款人形机器人，" \
                         "我可以做一些简单的交互动作，还可以陪你闲聊散心, " \
                         "另外我还是实验室的科研小助手，想知道今天的天气也可以问我哦！"
        Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()

    def execute_vla(self):
        """执行视觉语言动作"""
        tts_req = TTSRequest()
        tts_req.request = "好的，请稍等片刻"
        Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()

        vla_req = VLAProcessRequest()
        vla_req.prompt = "帮我拿一下这个瓶子"
        Thread(target=self.vla_client.call, args=(vla_req,), daemon=True).start()

    def execute_vlm(self):
        """执行视觉描述动作"""
        tts_req = TTSRequest()
        tts_req.request = "好的，让我仔细看一下！"
        Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()

        vlm_req = VLMProcessRequest()
        vlm_req.prompt = "请描述一下面前的场景"
        resp = self.vlm_client.call(vlm_req)
        vlm_result = resp.vlm_result
        rospy.loginfo(f"VLM 结果: {vlm_result}")
        
        # 如果结果过长，拆分后发送
        if len(vlm_result) > 100:
            parts = [vlm_result[i:i+100] for i in range(0, len(vlm_result), 100)]
            for part in parts:
                tts_req = TTSRequest()
                tts_req.request = part
                self.tts_client.call(tts_req)
        else:
            tts_req = TTSRequest()
            tts_req.request = vlm_result
            Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()

    def execute_pose(self):
        """执行自拍动作"""
        tts_req = TTSRequest()
        tts_req.request = "好的，摆个点赞的姿势，来和我自拍一张吧"
        Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()
        arm_req = StringServiceRequest()
        arm_req.request = random.choice([6, 9, 10])
        Thread(target=self.arm_client.call, args=(arm_req,), daemon=True).start()

    def execute_pangu(self):
        """执行盘古故事"""
        tts_req = TTSRequest()
        tts_req.request = "好的，盘古是中国古代传说时期中开天辟地的神。" \
                         "在很久很久以前，宇宙混沌一团，" \
                         "盘古凭借着自己的神力把天地开辟出来了。"
        Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()

    def execute_take_photo(self):
        """执行拍照动作"""
        tts_req = TTSRequest()
        tts_req.request = "好的，没问题，大家都过来吧！站到我面前，让我来为大家拍一张大合照！"
        Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()
        arm_req = StringServiceRequest()
        arm_req.request = 7
        Thread(target=self.arm_client.call, args=(arm_req,), daemon=True).start()
        Thread(target=self.take_photo_process, daemon=True).start()

    def execute_love(self):
        """执行比心动作"""
        tts_req = TTSRequest()
        tts_req.request = "南科大爱你呦！啾咪啾咪！"
        Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()
        arm_req = StringServiceRequest()
        arm_req.request = random.choice([12, 13])
        Thread(target=self.arm_client.call, args=(arm_req,), daemon=True).start()

    def execute_handclap(self):
        """执行鼓掌动作"""
        tts_req = TTSRequest()
        tts_req.request = "太棒了，大家都很棒！"
        Thread(target=self.tts_client.call, args=(tts_req,), daemon=True).start()
        arm_req = StringServiceRequest()
        arm_req.request = 8
        Thread(target=self.arm_client.call, args=(arm_req,), daemon=True).start()

    def take_photo_process(self):
        """拍照处理线程"""
        bridge = CvBridge()
    
        try:
            rospy.sleep(0.8)
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/look_at_me.mp3")
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/get_ready.mp3")
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/pose.mp3")
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/three.mp3")
            rospy.sleep(1.5)
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/two.mp3")
            rospy.sleep(1.5)
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/one.mp3")
            rospy.sleep(2.5)
            self.play_existing_audio("/home/whc/aiui_ws/src/aiui/audio/qie_zi.mp3")

            # 等待并获取ROS图像消息
            rospy.loginfo("等待相机图像消息...")
            image_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5.0)
            
            # 将ROS图像消息转换为OpenCV格式
            cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
            rospy.loginfo("成功获取图像!")
            
            # 保存图像
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"captured_image_{timestamp}.jpg"
            cv2.imwrite(filename, cv_image)
            rospy.loginfo(f"图像已保存为 {filename}")
            
            # 创建全屏窗口显示图像
            screen_width, screen_height = 1920, 1080
            window_name = "照片预览"
            cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
            cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            
            # 调整图像尺寸以适应屏幕
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
            cv2.waitKey(2000)  # 显示2秒
            cv2.destroyAllWindows()
            
        except rospy.ROSException:
            rospy.logerr("等待图像消息超时，请检查相机是否已启动")
        except Exception as e:
            rospy.logerr(f"拍照过程中发生错误: {str(e)}")

    def play_existing_audio(self, path):
        """播放预录音频文件"""
        try:
            if os.path.exists(path):
                playsound(path)
            else:
                rospy.logwarn(f"音频文件不存在: {path}")
        except Exception as e:
            rospy.logerr(f"播放音频时出错: {str(e)}")

    def key_loop(self):
        """主键盘循环"""
        rospy.loginfo("键盘监听器启动，等待输入...")
        print("\033[1;32m>> 键盘监听器已启动，请按菜单项进行操作\033[0m")
        
        while running and not rospy.is_shutdown():
            key = self.terminal.read_key()
            if key:
                self.handle_detected_intent(key)

    def shutdown_hook(self):
        """节点关闭时的清理工作"""
        global running
        running = False
        
        rospy.loginfo("正在关闭键盘动作监听器...")
        self.terminal.restore_terminal()
        
        if sys.stdout.isatty():
            print("\n\033[1;34m扩展键盘动作监听器已安全关闭!\033[0m")
            print("\033[1;35m感谢使用! 下次再见! 👋\033[0m\n")
        else:
            rospy.loginfo("扩展键盘动作监听器已安全关闭")

    def run(self):
        """主运行循环"""
        try:
            self.key_loop()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS中断，关闭节点")
        except Exception as e:
            rospy.logerr(f"运行时发生错误: {e}")
        finally:
            self.shutdown_hook()

def signal_handler(sig, frame):
    """信号处理函数"""
    global running
    print("\n\033[1;31m>>> 接收到中断信号，关闭节点...\033[0m")
    running = False
    rospy.signal_shutdown("用户中断")

if __name__ == "__main__":
    # 设置信号处理
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        node = ExtendedKeyboardActionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS中断异常")
    except Exception as e:
        rospy.logerr(f"程序启动发生错误: {e}")
    finally:
        rospy.loginfo("程序结束")