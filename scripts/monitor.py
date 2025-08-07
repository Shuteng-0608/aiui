#!/usr/bin/env python
import os
import time
import socket
import subprocess
import threading
import logging
import signal
from datetime import datetime

# 配置日志系统
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('roscore_monitor.log')
    ]
)
logger = logging.getLogger('ROScoreMonitor')

# 配置参数
ROSCORE_PORT = 8888
ROSCORE_URI = "http://192.168.8.52:8888"
CHECK_INTERVAL = 2  # 检查间隔(秒)
STANDBY_DURATION = 5  # 启动后等待时间(秒)
KILL_TIMEOUT = 5  # 杀死roscore的超时时间(秒)
NETWORK_TIMEOUT = 1.0  # 网络检查超时(秒)

class ROScoreMonitor:
    def __init__(self):
        self.roscore_process = None
        self.active_roscore_pid = None
        self.is_connected = False
        self.last_connection_time = datetime.now()
        self.last_down_time = None
        self.running = True
        self.lock = threading.Lock()
        
        # 设置优雅退出
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        logger.info(f"初始化ROScore监控器，目标端口: {ROSCORE_PORT}")
        
    def signal_handler(self, signum, frame):
        """处理退出信号"""
        logger.info(f"接收到信号 {signum}，准备退出...")
        self.running = False
        self.stop_roscore()
        
    def check_port_available(self, port):
        """检查端口是否可用"""
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(NETWORK_TIMEOUT)
            return s.connect_ex(('localhost', port)) != 0
    
    def check_roscore_connection(self):
        """检查与ROScore的连接状态"""
        try:
            # 创建一个TCP socket并尝试连接
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(NETWORK_TIMEOUT)
                s.connect(('localhost', ROSCORE_PORT))
                return True
        except (socket.timeout, ConnectionRefusedError):
            return False
    
    def start_roscore(self):
        """启动roscore在指定端口"""
        # 确保端口可用
        if not self.check_port_available(ROSCORE_PORT):
            logger.warning(f"端口 {ROSCORE_PORT} 被占用，尝试关闭已有进程...")
            self.stop_roscore()
            time.sleep(1)
            
            # 重试检查端口
            if not self.check_port_available(ROSCORE_PORT):
                logger.error(f"无法释放端口 {ROSCORE_PORT}，无法启动 roscore")
                return False
        
        logger.info(f"启动 roscore -p {ROSCORE_PORT}")
        
        # 配置环境变量以确保使用正确的主端口
        env = os.environ.copy()
        env["ROS_MASTER_URI"] = ROSCORE_URI
        env["ROS_HOSTNAME"] = "192.168.8.52"
        
        try:
            # 启动roscore
            self.roscore_process = subprocess.Popen(
                ["roscore", "-p", "8888"],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setpgrp  # 创建新的进程组
            )
            
            # 记录PID
            self.active_roscore_pid = self.roscore_process.pid
            logger.info(f"ROScore已启动，PID: {self.active_roscore_pid}")
            
            # 启动线程捕获输出
            threading.Thread(
                target=self.capture_output, 
                args=(self.roscore_process,),
                daemon=True
            ).start()
            
            # 等待roscore初始化完成
            time.sleep(STANDBY_DURATION)
            return True
        except Exception as e:
            logger.error(f"启动roscore失败: {e}")
            return False
    
    def capture_output(self, process):
        """捕获并记录roscore的输出"""
        while True:
            line = process.stdout.readline()
            if not line and process.poll() is not None:
                break
            if line:
                logger.debug(f"[roscore] {line.strip()}")
    
    def stop_roscore(self):
        """停止roscore进程"""
        if self.roscore_process is not None:
            logger.info(f"尝试停止roscore进程 (PID: {self.roscore_process.pid})")
            try:
                # 发送SIGINT信号给整个进程组
                os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGINT)
                self.roscore_process.wait(timeout=KILL_TIMEOUT)
            except subprocess.TimeoutExpired:
                logger.warning("正常关闭超时，强制终止")
                try:
                    os.killpg(os.getpgid(self.roscore_process.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            except Exception as e:
                logger.error(f"停止roscore时出错: {e}")
            
            self.roscore_process = None
            self.active_roscore_pid = None
    
    def monitor_loop(self):
        """主监控循环"""
        logger.info("开始监控roscore连接状态...")
        
        # 初始检查并尝试启动
        if not self.check_roscore_connection():
            logger.warning("初始状态: ROScore未运行")
            self.start_roscore()
        
        while self.running:
            try:
                current_status = self.check_roscore_connection()
                
                # 加锁确保状态处理不会冲突
                with self.lock:
                    if current_status != self.is_connected:
                        if current_status:
                            # ROScore恢复连接
                            downtime = (datetime.now() - self.last_down_time).total_seconds() if self.last_down_time else 0
                            logger.info(f"✅ ROScore已恢复连接! (中断时长: {downtime:.1f}秒)")
                            self.is_connected = True
                            self.last_connection_time = datetime.now()
                            self.last_down_time = None
                        else:
                            # ROScore断开连接
                            logger.error("⚠️ ROScore连接断开!")
                            self.is_connected = False
                            self.last_down_time = datetime.now()
                            
                            # 尝试重新启动
                            logger.info("尝试重新启动roscore...")
                            self.stop_roscore()
                            time.sleep(1)
                            self.start_roscore()
                
                # 等待下一次检查
                time.sleep(CHECK_INTERVAL)
                
            except Exception as e:
                logger.error(f"监控循环中出错: {e}")
                time.sleep(CHECK_INTERVAL * 2)  # 出错后稍长时间再试
        
        logger.info("监控器已停止运行")

if __name__ == '__main__':
    # 创建监控器实例
    monitor = ROScoreMonitor()
    
    # 启动监控循环
    monitor.monitor_loop()