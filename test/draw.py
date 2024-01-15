import open3d as o3d
import numpy as np
import copy
import os
import time
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from calib_utils.transformation_utils import Transmat2RT,RT2Transmat, TransformPoints
from calib_utils.py_lidar import save_pcd_with_data, trans_pointcloud2_to_array
from pyquaternion import Quaternion

def read_txt(txt_path):
        pose = []
        with open(txt_path, 'r') as file:
            # 逐行读取
            for line in file:
                cur_pose = line.split(",")
                pose.append(cur_pose)
        return pose
def get_T_pose(txt_q, txt_trans):
        R_cam_2_world = txt_q.rotation_matrix
        T_cam_2_world = np.identity(4)
        T_cam_2_world[:3,:3] = R_cam_2_world
        T_cam_2_world[:3, 3] = txt_trans
        return T_cam_2_world   
if __name__ == "__main__":   
        T_l2i_220 = np.array([[ 9.99995686e-01, -2.37039579e-03, -1.73459020e-03,  2.42367372e-02],
                            [ 2.37061629e-03 , 9.99997182e-01,  1.25075287e-04,  3.50166206e-01],
                            [ 1.73428884e-03, -1.29186796e-04,  9.99998488e-01,  1.24973272e-01],
                            [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        T_l2i_qj = np.array([[ 0.9999805,  -0.00568603, -0.00258086,  0.03977593],
                            [ 0.00568253,  0.99998293, -0.00136291, -0.16320373],
                            [ 0.00258857,  0.00134822,  0.99999574,  0.10224169],
                            [ 0.        ,  0.        ,  0.        ,  1.        ]])
        T_l2i_220_ylb = np.array([[0.9995853933007058, -0.028601724486214397, -0.003314039270702873, -0.03220797],
                            [0.02865765443782627, 0.9994221499236386, 0.018278541630401008, 0.33317482],
                            [0.0027893264411347025, -0.018365935816800905, 0.9998274411415091, 0.16190165],
                            [0.0, 0.0, 0.0, 1.0]])
        qj_pose_path = "/media/jyb/ONLY/0808-1/GDHD-POS-NAV96830_0808_qx_imu.txt"
        pro_pose_path = "/media/jyb/ONLY/0808-1/meragePose.txt"
        pose_qj = read_txt(qj_pose_path)
        pose_220 = read_txt(pro_pose_path)
        frame=0
        roll_list = []
        pitch_list = []
        yaw_qj_list = []
        yaw_220_list = []
        time_list = []
        x_qj_list = []
        y_qj_list = []
        z_qj_list = []
        x_220_list = []
        y_220_list = []
        z_220_list = []
        
        i = 0
        for tmp in pose_qj:
            if i < 500:
                time = tmp[0]
                q = Quaternion(tmp[-1], tmp[4], tmp[5], tmp[6])
                t = np.array([tmp[1], tmp[2], tmp[3]])
                T = get_T_pose(q,t)
                T_l2w = np.dot(T, T_l2i_qj)
                r1,t1 = Transmat2RT(T_l2w)
                deg = np.rad2deg(r1)
                yaw_qj_list.append(deg[2] + 90)
                x_qj_list.append(t1[0] - 373873)
                y_qj_list.append(t1[1] - 3447987)
                z_qj_list.append(t1[2] - 17)
                for tmp_220 in pose_220:
                    time_220 = tmp_220[0]
                    if time != time_220:
                        continue
                    else :
                        q2 = Quaternion(tmp_220[-1], tmp_220[4], tmp_220[5], tmp_220[6])
                        t2 = np.array([tmp_220[1], tmp_220[2], tmp_220[3]])
                        T2 = get_T_pose(q2,t2)
                        T_l2w_220 = np.dot(T2, T_l2i_220_ylb)
                        r2,t2 = Transmat2RT(T_l2w_220)
                        deg2 = np.rad2deg(r2)
                        yaw_220_list.append(deg2[2] + 90)
                        x_220_list.append(t2[0] - 373873)
                        y_220_list.append(t2[1] - 3447987)
                        z_220_list.append(t2[2] - 17)
                time_list.append(time)
                i = i + 1
            else:
                break
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(time_list, yaw_qj_list,label='qj_yaw')
        ax.plot(time_list, yaw_220_list,label='220_yaw')
        # ax.plot(time_list, roll_list,label='220_yaw')
        fig2 = plt.figure()
        ax1 = fig2.add_subplot(211)
        ax1.plot(time_list, x_220_list,label='220_x')
        ax1.plot(time_list, y_220_list,label='220_y')
        ax1.plot(time_list, z_220_list,label='220_z')
        ax1.plot(time_list, x_qj_list,label='qj_x')
        ax1.plot(time_list, y_qj_list,label='qj_y')
        ax1.plot(time_list, z_qj_list,label='qj_z')
        plt.legend()
        plt.show()
        