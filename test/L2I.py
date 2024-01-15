import laspy
import numpy as np
import open3d as o3d
import math
import os
import rosbag
from rospy.rostime import Time

class HDmap_L2I(object):
    def __init__(self, las_path, output_path):
        self.las_path = las_path
        self.output_path = output_path
        self.grid_size = 50
        
    def ToFlatIndex(self, index):
        result = 0
        for i in range(0, 8):
            result |= ((index[0] & (1 << i)) << i) | ((index[1] & 1 << i) << (i + 1))
        return result
    
    def read_local_map(self, xyz):
        
        idx = math.floor((xyz[0] / self.grid_size))
        idy = math.floor((xyz[1] / self.grid_size))
        map_points = np.zeros((0,3))
        for i in range(-1, 2):
            for j in range(-1, 2):
                index = np.array([idx + i, idy + j])
                id = self.ToFlatIndex(index)
                temppath = os.path.join(self.las_path,  str(id) + "_" + str(index[0]) +"_"+ str(index[1]))
                cur_las_path = temppath + ".las"
                infile = laspy.read(cur_las_path)
                tmp_points = np.vstack((infile.x - xyz[0], infile.y - xyz[1], infile.z - xyz[2])).transpose()
                map_points = np.concatenate((map_points, tmp_points), axis=0)
        local_map = o3d.geometry.PointCloud()
        local_map.points = o3d.utility.Vector3dVector(map_points)
        save_path = os.path.join(self.output_path, "output.pcd")
        o3d.io.write_point_cloud(save_path, local_map)
        return local_map

    def match(self, imu_pose, lidar_pointcloud, T_l2i_init):
        pose_init = np.eye(4)
        pose_init[0:3, 0:3] = imu_pose[0:3, 0:3]
        pose = np.dot(pose_init, T_l2i_init)
        xyz = np.array([imu_pose[0,3], imu_pose[1,3], imu_pose[2,3]])
        local_map = self.read_local_map(xyz)
        down_target = local_map.voxel_down_sample(voxel_size=0.5)
        threshold = 0.5
        reg_p2p = o3d.pipelines.registration.registration_icp(
            lidar_pointcloud, down_target, threshold, pose,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20))
        print(reg_p2p)
        print('transformation is :')
        T_l2i = np.dot(np.linalg.inv(pose_init), reg_p2p.transformation)
        print(T_l2i)
        T_l2w = np.dot(imu_pose, T_l2i)
        return T_l2w
    
    def las_to_pcd(self, lidar_point_path):
        infile = laspy.read(lidar_point_path)
        map_points = np.zeros((0,3))
        tmp_points = np.vstack((infile.x - 374900, infile.y - 3447500, infile.z)).transpose()
        map_points = np.concatenate((map_points, tmp_points), axis=0)
        index = np.where(map_points[: , 2] < 1)
        map_points = map_points[index]
        local_map = o3d.geometry.PointCloud()
        local_map.points = o3d.utility.Vector3dVector(map_points)
        return local_map
    
    def match_pcd(self, lidar_point_path, map_point_path):
        lidar_pcd = self.las_to_pcd(lidar_point_path)
        o3d.visualization.draw_geometries([lidar_pcd])
        map_pcd = self.las_to_pcd(map_point_path)
        o3d.visualization.draw_geometries([map_pcd])
        
        down_source = lidar_pcd.voxel_down_sample(voxel_size=0.5)
        down_target = map_pcd.voxel_down_sample(voxel_size=0.5)
        threshold = 0.02
        pose = np.eye(4)
        reg_p2p = o3d.pipelines.registration.registration_icp(
            down_source, down_target, threshold, pose,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20))
        print(reg_p2p)
        print('transformation is :',reg_p2p.transformation)
if __name__ == "__main__" :
    las_path = "/mnt/share/jinyanbin/Hdmap_grid"
    output_path = "/mnt/share/jinyanbin/output"
    lidar_point_path = "/mnt/Done_Date/22_GND_vslam/03_TEST/20231220/20231220003042.337/Pwc_raw_las/las/20231220003045.699/top/1703003465.899_top.las"
    map_point_path = "/mnt/share/jinyanbin/geojson/las/2023-08-08-23-20-35/1691508141.200_tail.las"
    lidar_pointcloud = o3d.io.read_point_cloud(lidar_point_path)
    l2i_calibration = HDmap_L2I(las_path, output_path)
    imu_pose = np.array([[-0.99794617089772997,   0.056714843604630381,  -0.029780303985382894 ,     375035.11676285311], 
                           [-0.05726781973114483,   -0.99819557145712978,  0.018055413221960168, 3446382.5833634534],
                           [-0.028702557617755864,   0.019723783569004987,   0.99939338378234344,      19.125983715995517],
                           [0         ,  0          , 0         , 1]])
    T_l2i_init = np.array([[0.9995853933007058, -0.028601724486214397, -0.003314039270702873, -0.03220797],
                    [0.02865765443782627, 0.9994221499236386, 0.018278541630401008, 0.33317482],
                    [0.0027893264411347025, -0.018365935816800905, 0.9998274411415091, 0.16190165],
                    [0.0, 0.0, 0.0, 1.0]])
    # T_l2w = l2i_calibration.match(imu_pose, lidar_pointcloud, T_l2i_init)
    T_l2w = l2i_calibration.match_pcd(lidar_point_path, map_point_path)
    print(T_l2w)
    # l2i_calibration.read_local_map(xyz)