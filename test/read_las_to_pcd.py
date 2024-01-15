import laspy
import open3d as o3d
import sys
import os
import numpy as np

from calib_utils.py_lidar import  save_pcd_with_data


def trans_pointcloud2_to_array(cloud_array, remove_nans=True, dtype=float):

    # if remove_nans:
    #     mask = np.isfinite(cloud_array['x']) & np.isfinite(cloud_array['y']) & np.isfinite(cloud_array['z'])
    #     cloud_array = cloud_array[mask]

    points = np.zeros(cloud_array.shape + (3,), dtype=dtype)
    points[...,0] = cloud_array[: , 0]
    points[...,1] = cloud_array[: , 1]
    points[...,2] = cloud_array[: , 2]

    info = np.zeros(cloud_array.shape + (3,), dtype=dtype)
    info[...,0] = cloud_array['intensity']
    info[...,1] = cloud_array['ring']
    # info[...,1] = 0
    info[...,2] = cloud_array['timestamp']*1e9
    # print(points)
    
    return points, info

def read_las( las_path):
        total_pointcloud = []
        total_intensity = []
        las_index = 0
        las_dir_list=os.listdir(las_path)
        for las_dir in las_dir_list:
            print("\r", end="")
            print('read las file:',"▓"*(las_index//50),'%.3f'%float(100*las_index/len(las_dir_list)),'%', end="")
            inFile = laspy.read(las_path+"/"+las_dir) # read a las file
            points = inFile.points
            xyz = np.vstack((inFile.x, inFile.y, inFile.z)).transpose() # extract x, y, z and put into a list
            color = np.vstack((inFile.intensity,inFile.intensity, inFile.intensity)).transpose()
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(xyz)
            pcd.colors = o3d.utility.Vector3dVector(color)
            pcd=pcd.voxel_down_sample(voxel_size=0.1)
            # print(type(pcd.points[0]))
            leng = len(pcd.points)
            for i in range(0, leng):
                if (pcd.points[i][1] > 3445543.2732 and pcd.points[i][1] < 3445756.6297 and pcd.points[i][2] < 38):
                    list_point=[pcd.points[i][0] - 375000,pcd.points[i][1] - 3447000,pcd.points[i][2]]
                    list_intensity = [pcd.colors[i][0],pcd.colors[i][0], pcd.colors[i][0]]
                    total_pointcloud.append(list_point)
                    total_intensity.append(list_intensity)
            las_index+=1
            sys.stdout.flush()
        return np.array(total_pointcloud), np.array(total_intensity)
    
if __name__ == "__main__":    
    cfg_path = "/mnt/share/jinyanbin/geojson/las_150m/test1"
    points, intensity = read_las(cfg_path)
    path = "/mnt/share/jinyanbin/geojson/las_150m/test1/1.pcd"
    # start_icp = time.time()
    # calibration.Run()
    # end_icp = time.time()
    save_pcd_with_data(points, intensity,path)
    
    print("icp time: ",points.shape)