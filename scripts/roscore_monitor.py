#!/usr/bin/env python
import rospy
import rosgraph
import time
import threading
import subprocess

class RoscoreMonitor:
    def __init__(self):
        self.master_uri = rospy.get_param('~master_uri', None)
        self.connection_status = False
        self.last_connection_time = 0
        self.disconnection_time = 0
        self.connection_lock = threading.Lock()
        self.reconnection_handler = None
        self.disconnection_handler = None
        
        rospy.loginfo(f"启动ROS Master监控器，目标: {self.master_uri or '默认ROS Master'}")
        
        # 设置默认处理函数
        self.set_reconnection_handler(self.default_reconnection_handler)
        self.set_disconnection_handler(self.default_disconnection_handler)
        
        # 初始检查
        self.check_connection(initial=True)

    def is_connected(self):
        """检查当前是否连接到ROS Master"""
        try:
            return rosgraph.is_master_online()
        except:
            return False

    def monitor_loop(self):
        """主监控循环"""
        check_interval = rospy.get_param('~check_interval', 1.0)  # 秒
        
        while not rospy.is_shutdown():
            self.check_connection()
            time.sleep(check_interval)

    def check_connection(self, initial=False):
        """检查连接状态并处理状态变化"""
        current_status = self.is_connected()
        current_time = time.time()
        
        with self.connection_lock:
            # 初始状态
            if initial:
                self.connection_status = current_status
                self.last_connection_time = current_time if current_status else 0
                rospy.loginfo(f"初始状态: {'已连接' if current_status else '未连接'}")
                return
            
            # 状态未变化
            if current_status == self.connection_status:
                if current_status:
                    self.last_connection_time = current_time
                return
            
            # 状态变化：断开连接
            if not current_status and self.connection_status:
                self.connection_status = False
                self.disconnection_time = current_time
                rospy.logerr(f"⚠️ ROS Master连接断开! (断开时间: {time.ctime(current_time)})")
                
                # 执行断开处理函数
                if self.disconnection_handler:
                    try:
                        self.disconnection_handler()
                    except Exception as e:
                        rospy.logerr(f"断开处理函数执行失败: {e}")
            
            # 状态变化：重新连接
            elif current_status and not self.connection_status:
                self.connection_status = True
                downtime = current_time - self.disconnection_time
                self.last_connection_time = current_time
                rospy.loginfo(f"✅ ROS Master重新连接成功! (断开时长: {downtime:.1f}秒)")
                
                # 执行重连处理函数
                if self.reconnection_handler:
                    try:
                        self.reconnection_handler(downtime)
                    except Exception as e:
                        rospy.logerr(f"重连处理函数执行失败: {e}")

    def set_reconnection_handler(self, handler):
        """设置重连时的回调函数"""
        self.reconnection_handler = handler
        rospy.loginfo("重连处理函数已设置")

    def set_disconnection_handler(self, handler):
        """设置断开连接时的回调函数"""
        self.disconnection_handler = handler
        rospy.loginfo("断开连接处理函数已设置")

    @staticmethod
    def default_reconnection_handler(downtime):
        """默认的重连处理函数"""
        rospy.logwarn(f"执行默认重连操作: 重启ROS节点 (服务中断{downtime:.1f}秒)")
        # 在实际应用中，这里应该重启您的节点
        # ros.signal_shutdown("Master reconnected, restarting node")
        # time.sleep(1)
        subprocess.call(["roslaunch", "aiui", "service.launch"])  # 示例: 重启一个测试脚本
        subprocess.call(["rosrun", "aiui", "ros_test01.py"])  # 示例: 重启一个测试脚本
        # subprocess.call(["rosrun", "aiui", "tts_test.py"])  # 示例: 重启一个测试脚本

    @staticmethod
    def default_disconnection_handler():
        """默认的断开连接处理函数"""
        rospy.logwarn("执行默认断开操作: 清理资源并等待重连")
        # 在实际应用中，这里应该清理资源
        # for pub in your_publishers: pub.unregister()
        # for sub in your_subscribers: sub.unregister()

def custom_reconnection_handler(downtime):
    """自定义重连处理函数示例"""
    rospy.loginfo(f"💡 自定义重连处理: Master恢复服务，中断{downtime:.1f}秒")
    rospy.loginfo("重新初始化节点资源...")
    # 实际应用中这里可以:
    # 1. 重新创建发布者/订阅者
    # 2. 重新注册服务
    # 3. 重置节点状态
    
def custom_disconnection_handler():
    """自定义断开连接处理函数示例"""
    rospy.loginfo("⚠️ 自定义断开处理: Master不可用，进入安全模式")
    rospy.loginfo("清理敏感资源，保存状态...")
    # 实际应用中这里可以:
    # 1. 关闭文件/网络连接
    # 2. 保存当前状态
    # 3. 通知其他系统组件

if __name__ == '__main__':
    try:
        rospy.init_node('roscore_monitor', anonymous=True)
        
        # 创建监控器
        monitor = RoscoreMonitor()
        
        # 设置自定义处理函数（取消注释使用）
        # monitor.set_reconnection_handler(custom_reconnection_handler)
        # monitor.set_disconnection_handler(custom_disconnection_handler)
        
        # 启动监控循环
        monitor.monitor_loop()
        
    except rospy.ROSInterruptException:
        pass