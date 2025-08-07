#!/usr/bin/env python
# -*- coding:utf-8 -*-
# @File: sr_modbus_model.py
# @Author: shiyixuan
# @Date: 2021/1/20
# @Describe:

from dataclasses import dataclass

from enum import Enum


class SystemState(Enum):
    """枚举系统状态"""
    SYS_STATE_INITIALING = 0x01  # 系统正在初始化
    SYS_STATE_IDLE = 0x02  # 系统空闲
    SYS_STATE_ERROR = 0x03  # 系统出错
    SYS_STATE_START_LOCATING = 0x04  # 正在启动定位
    SYS_STATE_TASK_NAV_INITIALING = 0x05  # 导航正在初始化
    SYS_STATE_TASK_NAV_FINDING_PATH = 0x06  # 导航正在寻路
    SYS_STATE_TASK_NAV_WAITING_FINISH = 0x07  # 正在等待到达目标位置
    SYS_STATE_TASK_NAV_WAITING_FINISH_SLOW = 0x08  # 检测到障碍,减速
    SYS_STATE_TASK_NAV_REFINDING_PATH = 0x09  # 导航正在重新寻路
    SYS_STATE_TASK_NAV_PAUSED = 0x0A  # 遇到障碍暂停运动
    SYS_STATE_TASK_NAV_NO_WAY = 0x0B  # 无法抵达目标位置
    SYS_STATE_TASK_PATH_NAV_INITIALING = 0x0E  # 正在初始化执行固定路径
    SYS_STATE_TASK_PATH_WAITING_FINISH = 0x0F  # 正在等待固定路径执行结束
    SYS_STATE_TASK_PATH_WAITING_FINISH_SLOW = 0x10  # 检测到障碍,减速前进
    SYS_STATE_TASK_PATH_PAUSED = 0x11  # 遇到障碍暂停运动
    SYS_STATE_TASK_NAV_NO_STATION = 0x12  # 无法检测到目标站点


class LocationState(Enum):
    """枚举定位状态"""
    LOCATION_STATE_NONE = 0x01  # 定位未启动
    LOCATION_STATE_INITIALING = 0x02  # 初始化中
    LOCATION_STATE_RUNNING = 0x03  # 定位正常
    LOCATION_STATE_RELOCATING = 0x04  # 正在重定位
    LOCATION_STATE_ERROR = 0x05  # 定位错误,需要重新启动定位


class OperationState(Enum):
    """枚举操作状态"""
    OPERATION_NONE = 0x00  # 无效操作
    OPERATION_AUTO = 0x01  # 自动控制模式
    OPERATION_MANUAL = 0x02  # 手动控制模式


class BatteryState(Enum):
    """枚举电池状态"""
    BATTERY_NA = 0x00  # 状态不可用
    BATTERY_CHARGING = 0x02  # 正在充电
    BATTERY_NO_CHARGING = 0x03  # 未充电


class MissionStatus(Enum):
    """枚举Mission运行状态"""
    MISSION_STATUS_NA = 0x00  # 无效状态
    MISSION_STATUS_PENDING = 0x02  # mission任务在队列中,但是又还没有启动的状态
    MISSION_STATUS_RUNNING = 0x03  # mission任务正在执行
    MISSION_STATUS_PAUSED = 0x04  # mission任务暂停
    MISSION_STATUS_FINISHED = 0x05  # mission任务完成
    MISSION_STATUS_CANCEL = 0x06  # mission任务取消


class MissionResult(Enum):
    """枚举Mission执行结果"""
    MISSION_STATUS_NA = 0x00  # 无效状态
    MISSION_STATUS_FINISHED = 0x01  # mission任务完成
    MISSION_STATUS_CANCEL = 0x02  # mission任务取消
    MISSION_STATUS_ERROR = 0x03  # mission任务错误


class MovementState(Enum):
    """枚举移动任务状态"""
    MT_NA = 0x00  # 无效状态
    MT_WAIT_FOR_START = 0x02  # 等待开始执行
    MT_RUNNING = 0x03  # 任务正在执行
    MT_PAUSED = 0x04  # 移动暂停
    MT_FINISHED = 0x05  # 移动完成
    MT_IN_CANCEL = 0x06  # 正在取消中
    MT_WAIT_FOR_CHECKPOINT = 8  # 交通管制


class MovementResult(Enum):
    """枚举移动任务结果"""
    MT_TASK_NA = 0x0  # 无效状态
    MT_TASK_FINISHED = 0x1  # 移动任务完成
    MT_TASK_CANCEL = 0x02  # 移动任务取消
    MT_TASK_ERROR = 0x03  # 移动任务错误


class ActionState(Enum):
    """枚举动作任务状态"""
    AT_NA = 0x00  # 无效状态
    AT_WAIT_FOR_START = 0x02  # 等待执行动作任务
    AT_RUNNING = 0x03  # 动作任务正在执行
    AT_PAUSED = 0x04  # 暂停动作任务
    AT_FINISHED = 0x05  # 任务完成
    AT_IN_CANCEL = 0x06  # 正在取消中


class ActionResult(Enum):
    """枚举动作任务结果"""
    AT_TASK_NA = 0x0  # 无效状态
    AT_TASK_FINISHED = 0x1  # 动作任务完成
    AT_TASK_CANCEL = 0x02  # 动作任务取消
    AT_TASK_ERROR = 0x03  # 动作任务错误


class ObstacleAvoid(Enum):
    """避障策略"""
    OBSTACLE_AVOID_WAIT = 0x01  # 暂停运动直至障碍消失
    OBSTACLE_AVOID_REPLAN = 0x02  # 重新规划路径绕过障碍
    OBSTACLE_AVOID_NONE = 0x10  # 不处理


@dataclass()
class BatteryInfo:
    """电池信息"""
    voltage: int = 0  # 电池电压，单位mV
    current: int = 0  # 电池电流，单位mA (负数为充电电流)
    temperature: int = 0  # 电池温度，单位°C
    remain_time: int = 0  # 电池预计剩余工作时间，单位min
    percentage_electricity: int = 0  # 剩余电量，单位%
    state: int = 0  # 当前电池的状态
    use_cycles: int = 0  # 电池充放电循环使用了的次数
    nominal_capacity: int = 0  # 电池标称容量，单位mAh


@dataclass()
class Pose:
    """位姿"""
    x: int = 0  # x: 单位(毫米)
    y: int = 0  # y: 单位(毫米)
    yaw: int = 0  # yaw: 单位(弧度 * 1000)


@dataclass()
class TotalService:
    """服务周期"""
    total_mileage: int = 0  # 运动总里程： 单位m
    total_startup_time: int = 0  # 开机总时间： 单位s
    total_startup_times: int = 0  # 开机总次数： 单位次


@dataclass()
class PGVScanDmcode:
    """PGV扫描二维码"""
    dmcode_id: int = 0  # 二维码id
    x: int = 0  # x: 单位(毫米)
    y: int = 0  # y: 单位(毫米)
    yaw: int = 0  # yaw: 单位(弧度 * 1000)


@dataclass()
class HardwareErrorCode:
    """硬件错误码"""
    error_code1: int = 0
    error_code2: int = 0
    error_code3: int = 0
    error_code4: int = 0
    error_code5: int = 0


@dataclass()
class MissionTask:
    """mission运行状态"""
    mission_id: int = 0  # 当前正在运行的mission id
    mission_state: int = 0  # Mission运行状态
    mission_result: int = 0  # Mission执行结果
    mission_error_code: int = 0  # Mission错误码


@dataclass()
class MovementTask:
    """移动任务"""
    state: int = 0  # 移动任务状态
    no: int = 0  # 当前移动任务no
    target_station: int = 0  # 当前移动任务目标站点
    path_no: int = 0  # 当前路径编号,移动任务运行过程中有效
    result: int = 0  # 移动任务结果
    result_value: int = 0  # 移动任务结果值


@dataclass()
class ActionTask:
    """移动任务"""
    state: int = 0  # 动作任务状态
    no: int = 0  # 当前动作任务no
    id: int = 0  # 当前动作任务ID
    param0: int = 0  # 当前动作任务参数0
    param1: int = 0  # 当前动作任务参数1
    result: int = 0  # 动作任务结果
    result_value: int = 0  # 动作任务结果值


@dataclass()
class Speed:
    """速度"""
    x_dir_linear_velocity: int = 0  # x方向线速度：单位mm/s
    y_dir_linear_velocity: int = 0  # y方向线速度：单位mm/s
    rotate_velocity: int = 0  # 角速度：单位(1/1000)rad/s


@dataclass()
class MissionRegisters:
    """mission中的通用寄存器"""
    register0: int = 0  # uint
    register1: int = 0  # uint
    register2: int = 0  # uint
    register3: int = 0  # uint
    register4: int = 0  # uint
    register5: int = 0  # uint
    register6: int = 0  # uint
    register7: int = 0  # uint
