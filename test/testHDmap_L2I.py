import HDmap_L2I_py
import open3d as o3d
import numpy as np
import copy
import os
import time
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
from calib_utils.transformation_utils import Transmat2RT,RT2Transmat, TransformPoints
from calib_utils.py_lidar import save_pcd_with_data, trans_pointcloud2_to_array


class Calibration(object):
    def __init__(self, cfg_path):
        self.cfg_path = cfg_path
        with open(cfg_path, 'r') as f:
            for line in f:
                if 'output_path' in line:
                    self.output_path = line.split("=")[1].split("\n")[0]
                if 'align_virtualize' in line:
                    self.align_virtualize = line.split("=")[1].split("\n")[0]
                if 'pose_path' in line:
                    self.pose_path = line.split("=")[1].split("\n")[0]
        self.time_pose = HDmap_L2I_py.GetTimeAndGpsPose(cfg_path)
        self.one_time_pose = HDmap_L2I_py.GetOneTimeAndGpsPose(cfg_path, 1691508273.400084)
        self.T_l2i_list = []
        
        self.T_l2i = np.eye(4)
            

    def get_pcd_list(self, dir_path, ext=None):
        newDir = dir_path
        if os.path.isfile(dir_path):
            if ext is None:
                self.pcd_list.append(dir_path)
            else:
                if ext in dir_path[-3:]:
                    self.pcd_list.append(dir_path)
        elif os.path.isdir(dir_path):
                for s in os.listdir(dir_path):
                        newDir=os.path.join(dir_path,s)
                        self.get_pcd_list(newDir, ext)
                    
    def calEvenOfT(self):
        eular = np.array([0.0,0.0,0.0])
        t = np.array([0.0,0.0,0.0])
        roll = []
        pitch = []
        yaw = []
        x =[]
        y = []
        z = []
        frame = []
        i =0
        for cur_T_l2i in self.T_l2i_list:
            r1, t1 = Transmat2RT(cur_T_l2i)
            roll.append(r1[0])
            pitch.append(r1[1])
            yaw.append(r1[2])
            x.append(t1[0])
            y.append(t1[1])
            z.append(t1[2])
            frame.append(i)
            i = i+ 1
            eular += r1
            t += t1
        eular /= len(self.T_l2i_list)
        t /= len(self.T_l2i_list)
        self.T_l2i = RT2Transmat(eular, t)
        
        fig = plt.figure()
        ax = fig.add_subplot(111)
        ax.plot(frame, roll ,label='roll')
        ax.plot(frame, pitch,label='pitch')
        ax.plot(frame, yaw,label = 'yaw')

        plt.show()
        return eular, t
    
    def trans_pointcloud2_to_array(self, source):

        info = np.zeros(source.shape + (3,), dtype=float)
        info[...,0] = 0
        info[...,1] = 0
        # info[...,1] = 0
        info[...,2] = 0
        # print(points)
        
        return source, info
    
    def Tran_lidar_to_enu(self):
        T_l2i = np.array([[0.9995853933007058, -0.028601724486214397, -0.003314039270702873, -0.03220797],
                            [0.02865765443782627, 0.9994221499236386, 0.018278541630401008, 0.33317482],
                            [0.0027893264411347025, -0.018365935816800905, 0.9998274411415091, 0.16190165],
                            [0.0, 0.0, 0.0, 1.0]])
        T_l2i_ylb = np.array([[ 0.99958441, -0.02859022, -0.00368883, -0.04407468],
                                [ 0.02865763,  0.99939437,  0.01973901,  0.15454698],
                                [ 0.00312225, -0.01983652,  0.99979836,  0.1280971],
                                [ 0.        ,  0.        ,  0.        ,  1.        ]])
        frame=0
        for tmp in calibration.time_pose:
            print("---------frame: {}--------".format(frame))
            frame = frame + 1
            time = format(tmp.time,'.6f')
            cur_lidar_pointcloud_path = os.path.join(calibration.output_path, time + '_lidar.pcd')
            init_pose = tmp.pose
            print(time)
            # print(init_pose)
            source = o3d.io.read_point_cloud(cur_lidar_pointcloud_path)
            T_lidar2enu = np.dot(init_pose, self.T_l2i)
            print(T_lidar2enu)
            lidar_point = copy.deepcopy(source)   
            lidar_point.transform(T_lidar2enu) 
            points = np.asarray(lidar_point.points)
            points[...,0] -= 376000
            points[...,1] -= 3447000
            lidar_point.points = o3d.utility.Vector3dVector(points)
            save_pointCloud_file = os.path.join("/media/data/output/enu_map_220", time + '_rearlidar_enu.pcd')
            o3d.io.write_point_cloud(save_pointCloud_file, lidar_point)
    def read_txt(self, txt_path):
        pose = []
        with open(txt_path, 'r') as file:
            # 逐行读取
            for line in file:
                cur_pose = line.split(",")
                pose.append(cur_pose)
        return pose

        
    def Run(self):
        frame = 0
        T = np.array([[0.9995853933007058, -0.028601724486214397, -0.003314039270702873, -0.03220797],
                            [0.02865765443782627, 0.9994221499236386, 0.018278541630401008, 0.33317482],
                            [0.0027893264411347025, -0.018365935816800905, 0.9998274411415091, 0.16190165],
                            [0.0, 0.0, 0.0, 1.0]])
        # T = np.array([[ 9.99988881e-01, 4.30283255e-03, 1.92970477e-03, 5.72691500e-02],
        #               [-4.30626451e-03, 9.99989148e-01, 1.77787566e-03, 7.39580000e-04],
        #               [-1.92203393e-03, -1.78616571e-03,9.99996558e-01, 1.48617730e-01],
        #               [ 0.00000000e+00,  0.00000000e+00, 0.00000000e+00,1.00000000e+00]])
        for tmp in calibration.time_pose:
            print("---------frame: {}--------".format(frame))
            frame = frame + 1
            time = format(tmp.time,'.6f')
            cur_lidar_pointcloud_path = os.path.join(calibration.output_path, time + '_lidar.pcd')
            cur_map_pointcloud_path = os.path.join(calibration.output_path, time + '_submap_imu_cor.pcd')
            init_pose = tmp.pose
            print(init_pose)
            pose_init = np.eye(4)
            pose_init[0:3, 0:3] = init_pose[0:3, 0:3]
            # pose[0:3, 0:3] = init_pose[0:3, 0:3]
            pose = np.dot(pose_init, T)
            print(pose)
            # pose[1][3] = 0.3
            # pose[2][3] = 0.15
            source = o3d.io.read_point_cloud(cur_lidar_pointcloud_path)
            target = o3d.io.read_point_cloud(cur_map_pointcloud_path)
            down_target = target.voxel_down_sample(voxel_size=0.5)
            if calibration.align_virtualize == 'true':
                source.paint_uniform_color([1, 0.706, 0])     #yellow
                down_target.paint_uniform_color([0, 0.651, 0.929]) #target 
                o3d.visualization.draw_geometries([source, down_target],width=1500,height=1500)
            threshold = 0.5
            # source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            # target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            reg_p2p = o3d.pipelines.registration.registration_icp(
                source, down_target, threshold, pose,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20))
    
            print(reg_p2p)
            print('transformation is :')
            print(np.dot(np.linalg.inv(pose_init), reg_p2p.transformation))
            self.T_l2i_list.append(np.dot(np.linalg.inv(pose_init), reg_p2p.transformation))
            source_temp = copy.deepcopy(source)       
            target_temp = copy.deepcopy(down_target)
            
            source_temp.transform(reg_p2p.transformation)
            cur_o3d_path =  os.path.join(calibration.output_path, str(time) + '_o3d.pcd')
            o3d.io.write_point_cloud(cur_o3d_path, source_temp)
            if calibration.align_virtualize == 'true':
                source_temp.paint_uniform_color([1, 0.706, 0]) 
                target_temp.paint_uniform_color([0, 0.651, 0.929])
                o3d.visualization.draw_geometries([source_temp, target_temp],width=1500,height=1500)


if __name__ == "__main__":    
    pcd_path = "/media/data/L2I_calibration/config/config.cfg"
    start_init = time.time() 
    calibration = Calibration(pcd_path)
    end_init = time.time()
    print("init time: ", end_init - start_init)
    start_icp = time.time()
    calibration.Run()
    end_icp = time.time()
    
    print("icp time: ", end_icp - start_icp)
    eular, t = calibration.calEvenOfT()
    calibration.Tran_lidar_to_enu()
    T = RT2Transmat(eular, t)
    print(eular)
    print(t)
    print(T)