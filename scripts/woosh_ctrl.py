#!/usr/bin/env python3

import rospy
import actionlib
import sys
from woosh_msgs.msg import StepControlAction, StepControlGoal, StepControl

class MoveCommandClient:
    def __init__(self, timeout=30.0):
        self.client = actionlib.SimpleActionClient('/cmd_vel_control', StepControlAction)
        self.timeout = timeout
        rospy.loginfo("等待 step_control Action 服务器...")
        
        # 添加超时和错误处理
        if not self.client.wait_for_server(rospy.Duration(timeout)):
            rospy.logerr(f"无法连接到Action服务器，超时: {timeout}秒")
            raise Exception("Action服务器连接失败")
        rospy.loginfo("连接到 step_control Action 服务器")
    
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
    
    def send_command(self, distance, speed, use_avoid=True):
        """发送移动命令"""
        try:
            goal = StepControlGoal()
            goal.mode = StepControlGoal.EXCUTE
            goal.useAvoid = use_avoid
            
            step = StepControl()
            step.executeMode = StepControlGoal.STRAIGHT
            step.data = distance
            step.speed = speed
            # step.angle = 0.0
            
            goal.stepControl = [step]
            
            rospy.loginfo(f"发送移动指令: 距离={distance}m, 速度={speed}m/s, 避障={use_avoid}")
            
            # 发送目标并设置回调函数
            self.client.send_goal(goal, 
                                 done_cb=self.done_callback, 
                                 active_cb=self.active_callback, 
                                 feedback_cb=self.feedback_callback)
            rospy.loginfo("✅ 移动指令已发送，等待执行...")
            
            # 可选：等待结果（同步方式）
            success = self.client.wait_for_result(rospy.Duration(self.timeout))
            
            if success:
                rospy.loginfo("🎉 移动任务顺利完成！")
                return True
            else:
                rospy.logwarn("⏰ 移动任务超时！")
                self.client.cancel_goal()
                return False
                
        except Exception as e:
            rospy.logerr(f"发送指令时发生错误: {e}")
            return False
    
    def cancel_command(self):
        """取消当前移动命令"""
        try:
            self.client.cancel_goal()
            rospy.loginfo("🛑 移动任务取消指令已发送")
            return True
        except Exception as e:
            rospy.logerr(f"取消移动任务时发生错误: {e}")
            return False
    
    def get_status(self):
        """获取当前移动状态"""
        try:
            state = self.client.get_state()
            state_names = {
                actionlib.GoalStatus.PENDING: "等待中",
                actionlib.GoalStatus.ACTIVE: "执行中",
                actionlib.GoalStatus.PREEMPTED: "被抢占", 
                actionlib.GoalStatus.SUCCEEDED: "成功",
                actionlib.GoalStatus.ABORTED: "失败"
            }
            return state_names.get(state, "未知状态")
        except Exception as e:
            rospy.logerr(f"获取状态时发生错误: {e}")
            return "错误"

def main():
    rospy.init_node('move_command_client', anonymous=True)
    
    # 解析命令行参数
    if len(sys.argv) < 3:
        print("用法: rosrun aiui woosh_ctrl.py <距离> <速度> [use_avoid]")
        print("示例: rosrun aiui woosh_ctrl.py 0.1 0.1 true")
        print("示例: rosrun aiui woosh_ctrl.py 0.1 0.1 false")
        return 1
    
    try:
        distance = float(sys.argv[1])
        speed = float(sys.argv[2])
        use_avoid = True if len(sys.argv) < 4 or sys.argv[3].lower() == 'true' else False
        
        rospy.loginfo(f"启动移动客户端: 距离={distance}m, 速度={speed}m/s, 避障={use_avoid}")
        
        # 创建客户端实例
        client = MoveCommandClient(timeout=30.0)
        
        # 发送命令
        success = client.send_command(distance, speed, use_avoid)
        
        if success:
            rospy.loginfo("程序执行成功完成")
            return 0
        else:
            rospy.logwarn("程序执行失败")
            return 1
            
    except ValueError as e:
        rospy.logerr(f"参数格式错误！距离和速度必须是数字: {e}")
        return 1
    except Exception as e:
        rospy.logerr(f"发生错误: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
        return 1

if __name__ == '__main__':
    sys.exit(main())