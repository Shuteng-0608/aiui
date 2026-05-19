#!/usr/bin/env python
import rospy
import rosbag
from arm_teleop.msg import DualArmMovej, DualHandTele
from arm_teleop.srv import MovejService, MovejServiceRequest
from arm_teleop.srv import StartDualTeleOP, StartDualTeleOPRequest
import time
import threading
import queue
import pygame

# 导入自定义服务类型
from aiui.srv import PlayArmMovement, PlayArmMovementResponse

# 播放现有音频文件（如有）
def play_existing_audio(file_path):
    pygame.mixer.init()
    pygame.mixer.music.load(file_path)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():  # 等待播放结束
        pygame.time.Clock().tick(10)

# rosbag filter input.bag output.bag "t.to_sec() >= t1 and t.to_sec() <= t2"
def act_play(bag_path, rate=1.0):
    # 创建发布者
    arm_publisher = rospy.Publisher('/arm_teleop/dual_arm_movej', DualArmMovej, queue_size=100)
    hand_publisher = rospy.Publisher('/arm_teleop/dual_hand_tele', DualHandTele, queue_size=100)
    rospy.loginfo("开始播放: %s", bag_path)
    rospy.loginfo("播放速率: %.1fx", rate)

    arm_msg_count = 0
    hand_msg_count = 0

    arm_queue = queue.Queue()
    hand_queue = queue.Queue()
    rate_obj = rospy.Rate(rate * 10)

    def publisher_worker(msg_queue, publisher, is_arm=True):
        nonlocal arm_msg_count, hand_msg_count
        while not rospy.is_shutdown():
            msg = msg_queue.get()
            if msg is None:
                msg_queue.task_done()
                break
            publisher.publish(msg)
            if is_arm:
                arm_msg_count += 1
                rospy.loginfo("播放双臂消息 %d", arm_msg_count)
            else:
                hand_msg_count += 1
                rospy.loginfo("播放双手消息 %d", hand_msg_count)
            rate_obj.sleep()
            msg_queue.task_done()

    arm_thread = threading.Thread(target=publisher_worker, args=(arm_queue, arm_publisher, True), daemon=True)
    hand_thread = threading.Thread(target=publisher_worker, args=(hand_queue, hand_publisher, False), daemon=True)
    arm_thread.start()
    hand_thread.start()

    start_time = rospy.Time.from_sec(10.0)
    for topic, msg, timestamp in bag_path.read_messages(start_time=start_time):
        if topic == '/arm_teleop/dual_arm_movej':
            arm_queue.put(msg)
        elif topic == '/arm_teleop/dual_hand_tele':
            hand_queue.put(msg)
        else:
            rospy.logdebug("忽略无关话题: %s", topic)

    # 发送结束标记并等待线程完成
    arm_queue.put(None)
    hand_queue.put(None)
    arm_queue.join()
    hand_queue.join()

    bag_path.close()


def play_arm_movement(rate=1.0):
    """
    播放双臂运动数据和双手遥操作数据
    """
    bag0 = rosbag.Bag("/home/pangu/pangu/src/arm_teleop/demo/act00_mod.bag")
    bag1 = rosbag.Bag("/home/pangu/pangu/src/arm_teleop/demo/act01_mod.bag")
    bag2 = rosbag.Bag("/home/pangu/pangu/src/arm_teleop/demo/act02_mod.bag")
    bag3 = rosbag.Bag("/home/pangu/pangu/src/arm_teleop/demo/act03_mod.bag")
    bag4 = rosbag.Bag("/home/pangu/pangu/src/arm_teleop/demo/act04_mod.bag")
    bag5 = rosbag.Bag("/home/pangu/pangu/src/arm_teleop/demo/act05_mod.bag")
    bag6 = rosbag.Bag("/home/pangu/pangu/src/arm_teleop/demo/act06_mod.bag")
    bag7 = rosbag.Bag("/home/pangu/pangu/src/arm_teleop/demo/act07_mod.bag")

    # ============== Start Dual TeleOP Service ==============
    rospy.wait_for_service('/aris_node/start_teleop_srv')
    start_teleop_service = rospy.ServiceProxy('/aris_node/start_teleop_srv', StartDualTeleOP)
    tele_req = StartDualTeleOPRequest()
    tele_req.running_flag = True
    tele_response = start_teleop_service.call(tele_req)

    threading.Thread(target=play_existing_audio, args=("/home/pangu/pangu/src/aiui/audio/task9_v2.mp3",), daemon=True).start()
    scale = 1.1  # 1.1倍速
    act_play(bag0, rate * scale)
    act_play(bag1, rate * scale)
    # rospy.sleep(1)
    act_play(bag2, rate * scale) # tieban
    # rospy.sleep(1)
    act_play(bag3, 12.0 * scale)
    rospy.sleep(2)
    act_play(bag4, 15.0 * scale)
    # rospy.sleep(1)
    act_play(bag5, 19.0 * scale)
    # rospy.sleep(1)
    act_play(bag6, rate * scale) # 
    # rospy.sleep(1)
    act_play(bag7, 17.0 * scale) # guo
    
    # rospy.loginfo("播放完成，双臂 %d 条，双手 %d 条", arm_msg_count, hand_msg_count)
    rospy.sleep(2)  # 确保所有消息发布完成 before stopping teleop
    tele_req = StartDualTeleOPRequest()
    tele_req.running_flag = False
    tele_response = start_teleop_service.call(tele_req)
    rospy.sleep(5)


def handle_play_arm_movement(req):
    """
    处理播放运动数据的服务回调函数
    """
    rospy.loginfo("收到播放请求，速率: %.1f", req.rate)
    try:
        # 调用原有的播放函数
        play_arm_movement(rate=req.rate)
        return PlayArmMovementResponse(success=True, message="播放完成")
    except Exception as e:
        rospy.logerr("播放过程中发生错误: %s", str(e))
        return PlayArmMovementResponse(success=False, message=f"错误: {str(e)}")


if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node("arm_player_server")
    
    # 创建服务
    s = rospy.Service('play_arm_movement', PlayArmMovement, handle_play_arm_movement)
    rospy.loginfo("播放运动数据服务已启动，等待请求...")
    
    # 保持节点运行，等待服务调用
    rospy.spin()