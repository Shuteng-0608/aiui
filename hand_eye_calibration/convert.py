import numpy as np
import matplotlib.pyplot as plt

def quaternion_to_euler(w, x, y, z):
    """
    将单位四元数转换为欧拉角（Roll, Pitch, Yaw）
    旋转顺序：Z-Y-X（先Yaw，再Pitch，最后Roll）
    
    参数:
        w, x, y, z: 单位四元数的四个分量
    
    返回:
        roll, pitch, yaw: 欧拉角（弧度）
    """
    # 确保四元数已归一化
    norm = np.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    # 计算俯仰角Pitch（y轴旋转）
    sin_p = 2.0 * (w*y - z*x)
    # 避免数值误差导致的超出[-1,1]范围
    sin_p = np.clip(sin_p, -1.0, 1.0)
    pitch = np.arcsin(sin_p)
    
    # 计算偏航角Yaw（z轴旋转）和滚转角Roll（x轴旋转）
    sin_r_cosp = 2.0 * (w*x + y*z)
    cos_r_cosp = 1.0 - 2.0 * (x*x + y*y)
    roll = np.arctan2(sin_r_cosp, cos_r_cosp)
    
    sin_y_cosp = 2.0 * (w*z + x*y)
    cos_y_cosp = 1.0 - 2.0 * (y*y + z*z)
    yaw = np.arctan2(sin_y_cosp, cos_y_cosp)
    
    return roll, pitch, yaw

# 读取pose.txt文件
data = []
with open('/home/whc/aiui_ws/calib_data/20250617/poses_save.txt', 'r') as f:
    for line in f:
        # 去除首尾空白并分割字符串
        values = line.strip().split(',')
        # 转换为浮点数
        float_values = list(map(float, values))
        data.append(float_values)

# 处理每一行数据
euler_data = []
for row in data:
    # 提取四元数分量（假设顺序为w,x,y,z）
    w, x, y, z = row[3], row[4], row[5], row[6]
    
    # 转换为欧拉角（弧度）
    roll, pitch, yaw = quaternion_to_euler(w, x, y, z)
    
    # 转换为角度（可选）
    # roll_deg = np.degrees(roll)
    # pitch_deg = np.degrees(pitch)
    # yaw_deg = np.degrees(yaw)
    
    # 保存转换后的数据（包含原始位置和欧拉角）
    # 原始数据：[tx, ty, tz] + [w, x, y, z]
    # 转换后：[tx, ty, tz, roll_deg, pitch_deg, yaw_deg]
    result_row = row[0:3] + [roll, pitch, yaw]
    euler_data.append(result_row)

# 将转换后的数据保存到新文件
with open('/home/whc/aiui_ws/calib_data/20250617/pose_euler.txt', 'w') as f:
    for row in euler_data:
        # 格式化输出，保留6位小数
        line = ','.join([f'{val:.6f}' for val in row])
        f.write(line + '\n')

print(f"转换完成！结果已保存到 pose_euler.txt，共处理 {len(data)} 行数据")