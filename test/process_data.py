import numpy as np
import math
def calculate_column_stats(file_path):
    # 使用 NumPy 读取文本文件
    datas = np.loadtxt(file_path)
    filter_data = []
    count = 0
    for data in datas:
        if abs(data[0]) < 1.0 and abs(data[1]) < 1.0 and abs(data[5]) < 0.1 and data[5] != 0:
            count += 1
            filter_data.append(data)
    data1 =  np.array(filter_data)
    print("count = ", count)
    # 计算每一列的均值和标准差
    column_means = np.mean(data1, axis=0)
    column_stds = np.std(data1, axis=0)

    return column_means, column_stds

def euler2rotation(roll, pitch, yaw):
    cos_roll = math.cos(roll)
    cos_pitch = math.cos(pitch)
    cos_yaw = math.cos(yaw)
    sin_roll = math.sin(roll)
    sin_pitch = math.sin(pitch)
    sin_yaw = math.sin(yaw)
    rotation = np.identity(3)
    rotation[0][0] = cos_yaw * cos_roll - sin_yaw * sin_pitch * sin_roll
    rotation[1][0] = sin_yaw * cos_roll + cos_yaw * sin_pitch * sin_roll
    rotation[2][0] = -cos_pitch * sin_roll
    rotation[0][1] = -sin_yaw * cos_pitch
    rotation[1][1] = cos_yaw * cos_pitch
    rotation[2][1] = sin_pitch
    rotation[0][2] = cos_yaw * sin_roll + sin_yaw * sin_pitch * cos_roll
    rotation[1][2] = sin_yaw * sin_roll - cos_yaw * sin_pitch * cos_roll
    rotation[2][2] = cos_pitch * cos_roll
    return rotation

file_path = "/media/data/L2I_calibration/build/devel/lib/L2I_calibration/all.txt"
mean, std = calculate_column_stats(file_path)

T_l2i =  np.array([[0.9996539041998037,0.02467177360400634,-0.009131013378722592,-0.02599951],
                [-0.02466335352205607,0.9996952788887938, 0.001033615258626519, 0.21332493],
                [0.009153732087833868,-0.0008080561177524306, 0.9999577772257061, 0.37190165],
                [0.         , 0.        ,  0.        ,  1.        ]])
R_delta = euler2rotation(mean[0] / (180 / math.pi), mean[1] / (180 / math.pi), mean[2] / (180 / math.pi))
T_delta = np.identity(4)
T_delta[0:3, 0:3] = R_delta
T_delta[0:3, 3] = np.array([mean[3], mean[4], mean[5]])
T_new = np.dot(T_l2i, T_delta)
print("Column mean : ", mean)
print("Column std : ", std)
print("T_new: ", T_new)