#!/usr/bin/env python3

import rospy
import sys
from woosh_msgs.srv import ExecTask, ExecTaskRequest

class SimpleExecTaskClient:
    def __init__(self):
        # 等待服务可用
        rospy.wait_for_service('/exec_task')
        try:
            self.exec_task_client = rospy.ServiceProxy('/exec_task', ExecTask)
            rospy.loginfo("连接到 /exec_task 服务成功")
        except rospy.ServiceException as e:
            rospy.logerr("服务调用失败: %s", e)
            sys.exit(1)
    
    def send_task(self, mark_no):
        """
        发送任务，只包含task_id和mark_no
        
        参数:
        - task_id: 任务ID
        - mark_no: 标记编号
        """
        try:
            # 创建服务请求
            req = ExecTaskRequest()
            
            req.task_exect = 1      # 执行任务
            req.task_id = 0         # 任务id
            req.task_type = 1       # 拣选
            req.direction = 0       # 动作方向
            req.task_type_no = 0    # 组合类型默认0
            req.mark_no = mark_no   # 储位编号
            
            rospy.loginfo(f"发送任务: mark_no={mark_no}")
            response = self.exec_task_client.call(req)
            
            # 处理响应
            if response.success:
                rospy.loginfo(f"任务执行成功: {response.message}")
            else:
                rospy.logwarn(f"任务执行失败: {response.message}")
                rospy.logwarn(f"状态码: {response.statusCode}")
            
            return response
            
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用异常: {e}")
            return None

def main():
    rospy.init_node('woosh_exec_task_client', anonymous=True)
    

    if len(sys.argv) != 2:
        print("用法: python3 script.py <mark_no>")
        print("示例: python3 script.py A001")
        sys.exit(1)
    mark_no = sys.argv[1]
    
    client = SimpleExecTaskClient()
    client.send_task(mark_no)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass