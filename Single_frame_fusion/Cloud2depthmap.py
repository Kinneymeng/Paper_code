# -*-coding:utf-8 -*-
import os
import open3d as o3d
import numpy as np
import math
import matplotlib.pyplot as plt
import cv2 as cv
import calib_utils
import depth_to_pc

def pass_through(pcd, limit_min, limit_max, filter_value_name="y"):
    """
    直通滤波
    :param cloud:输入点云
    :param limit_min: 滤波条件的最小值
    :param limit_max: 滤波条件的最大值
    :param filter_value_name: 滤波字段(x or y or z)
    :return: 位于[limit_min,limit_max]范围的点云
    """
    points = np.asarray(pcd.points)

    if filter_value_name == "x":

        ind = np.where((points[:, 0] >= limit_min) & (points[:, 0] <= limit_max))[0]
        x_cloud = pcd.select_by_index(ind)
        return x_cloud

    elif filter_value_name == "y":

        ind = np.where((points[:, 1] >= limit_min) & (points[:, 1] <= limit_max))[0]
        y_cloud = pcd.select_by_index(ind)
        return y_cloud

    elif filter_value_name == "z":

        ind = np.where((points[:, 2] >= limit_min) & (points[:, 2] <= limit_max))[0]
        z_cloud = pcd.select_by_index(ind)
        return z_cloud

def PointToDense(Point):
    CAM_WID,CAM_HGT=1280,366
    CAM_FX,CAM_FY=350.715,350.715
    CAM_CX,CAM_CY=320.020,182.2878



    point_arrary=np.asarray(Point.points).astype(np.float32)

    x=point_arrary[:,0]

    # u=np.ones(point_arrary.shape[0])
    # v=np.ones(point_arrary.shape[0])
    #
    # for i in range(point_arrary.shape[0]):
    #     if x[i]>0.2:
    #         u[i] = np.round(point_arrary[i][1] * CAM_FX / x[i] + CAM_CX).astype(int)
    #         v[i] = np.round(point_arrary[i][2] * CAM_FY / x[i] + CAM_CY).astype(int)
    #
    #
    # valid = np.bitwise_and(np.bitwise_and((u > 0), (u < CAM_WID)),
    #                        np.bitwise_and((v > 0), (v < CAM_HGT)))
    # u, v, z = u[valid], v[valid], x[valid]
    #
    # x_scaler=255*(z-min(z))/(max(z)-min(z))
    # x_scaler=np.round(x_scaler).astype(int)
    # print(u[0].shape)
    # print(v)
    # print(x_scaler[0].shape)
    #
    # #np.savetxt("u.txt",u)
    #
    # img_z = np.full((CAM_HGT, CAM_WID), np.inf)
    #
    # img_z[u[0],v[0]]=3
    # for i in range(u.size):
    #     img_z[v[i],u[i]]=min(img_z[v[i],u[i]],x_scaler[i])
    u=np.round(point_arrary[:,1] * CAM_FX / x + CAM_CX).astype(int)
    v=np.round(point_arrary[:,2] * CAM_FY / x + CAM_CY).astype(int)
    print(u.size)
    valid = np.bitwise_and(np.bitwise_and((u >= 0), (u < CAM_WID)),
                           np.bitwise_and((v >= 0), (v < CAM_HGT)))
    u, v, z = u[valid], v[valid], x[valid]
    print(max(z))
    print(min(z))
    x_scalar=80*(z-min(z))/(max(z)-min(z))

    img_z=np.full((CAM_HGT,CAM_WID),np.inf)
    for ui, vi, zi in zip(u,v,x_scalar):
        img_z[vi,ui]=min(img_z[vi,ui],zi)
        k=0

        # for i in range(1):
        #     k += 6
        #     img_z[vi+k,ui]=img_z[vi,ui]

    #img_z[vi,ui]=max(img_z[vi,ui],zi)时，保证为空值的点为inf
    # for i in range(CAM_WID):
    #     for j in range(CAM_HGT):
    #         if img_z[j,i]==0.0:
    #             img_z[j,i]=np.inf


    np.savetxt('dep_rot.csv', img_z, fmt='%.12f', delimiter=',', newline='\n')

    img = np.genfromtxt('dep_rot.csv', delimiter=',').astype(np.float32)
    print(img.shape)
    k=0
    img_n = np.zeros((img.shape[0],img.shape[1]))
    MaxValue=0
    MinValue=0;
    for i in range(img.shape[0]):
        for j in range(img.shape[1]):
            if not math.isinf(img[i][j]):
                img_n[i][j] = img[i][j]
                MaxValue=max(MaxValue,img_n[i][j])
                MinValue=min(MinValue, img_n[i][j])

    print(MaxValue)
    print(MinValue)
    cv.imshow("img_n",img_n)
    cv.imwrite("img_n.png",img_n)
    cv.waitKey(0)

    # for i in range(img_n.shape[0]):
    #     for j in range(img_n.shape[1]):
    #         if img_n[i][j]!=0:
    #             print(img_n[i][j])
    # cv.imshow("img",img)
    # cv.waitKey(0)
    # img_n = np.zeros((img.shape[0],img.shape[1]))
    # print(img_n.shape)
    # print(img.shape)
    # for i in range(img.shape[0]):
    #     for j in range(img.shape[1]):
    #         if not math.isinf(img[i][j]):
    #             img_n[i][j] = img[i][j]
    # print(img_n)

    # plt.imshow(img_n, cmap='jet')
    # plt.imshow(img_n, cmap=plt.cm.gray)
    # plt.show()



def project_depths(point_cloud, cam_p, image_shape, max_depth=100.0):
    """Projects a point cloud into image space and saves depths per pixel.
    Args:
        point_cloud: (3, N) Point cloud in cam0
        cam_p: camera projection matrix
        image_shape: image shape [h, w]
        max_depth: optional, max depth for inversion
    Returns:
        projected_depths: projected depth map
    """

    # Only keep points in front of the camera
    all_points = point_cloud.T

    # Save the depth corresponding to each point
    points_in_img = calib_utils.project_pc_to_image(all_points.T, cam_p)
    points_in_img_int = np.int32(np.round(points_in_img))

    # Remove points outside image
    valid_indices = \
        (points_in_img_int[0] >= 0) & (points_in_img_int[0] < image_shape[1]) & \
        (points_in_img_int[1] >= 0) & (points_in_img_int[1] < image_shape[0])

    all_points = all_points[valid_indices]
    points_in_img_int = points_in_img_int[:, valid_indices]

    # Invert depths
    all_points[:, 2] = max_depth - all_points[:, 2]

    # Only save valid pixels, keep closer points when overlapping
    projected_depths = np.zeros(image_shape)
    valid_indices = [points_in_img_int[1], points_in_img_int[0]]
    projected_depths[valid_indices] = [
        max(projected_depths[
            points_in_img_int[1, idx], points_in_img_int[0, idx]],
            all_points[idx, 2])
        for idx in range(points_in_img_int.shape[1])]

    projected_depths[valid_indices] = \
        max_depth - projected_depths[valid_indices]

    return projected_depths.astype(np.float32)

def depthToPc(img):
    #Input:csv
    CAM_WID,CAM_HGT=1280,366
    CAM_FX,CAM_FY=350.715,350.715
    CAM_CX,CAM_CY=320.020,182.2878
    #
    # x,y=np.meshgrid(range(CAM_WID), range(CAM_HGT))
    # x=x.astype(np.float32)-CAM_CX
    # y = y.astype(np.float32) - CAM_CY
    # print(x.shape)
    # print(x)
    #
    # img_z=img.copy()
    # MaxValue=0
    # MinValue=0
    # for i in range(img_z.shape[0]):
    #     for j in range(img_z.shape[1]):
    #         MaxValue=max(MaxValue,img_z[i,j])
    #         MinValue=min(MinValue,img_z[i,j])
    #         if  img_z[i,j]!=0:
    #            img_z[i,j]=(img_z[i,j]*2.87/80)+0.86
    # print(MaxValue,MinValue)
    # pc_x=img_z * x/CAM_FX
    # pc_y=img_z * y/CAM_FY
    #
    # pc=np.array([img_z.ravel(),pc_x.ravel(),pc_y.ravel()]).T
    # print(pc.shape)
    # for i in range(pc.shape[0]):
    #     if pc[i][0]!=0:
    #         print(pc[i][0],pc[i][1],pc[i][2])
    # #保存点云
    # pc_points=o3d.geometry.PointCloud()
    # pc_points.points=o3d.utility.Vector3dVector(pc)
    # pc_3d_points=pc_points.remove_non_finite_points(True,True)
    # print(pc_3d_points)
    #
    # o3d.io.write_point_cloud("complete_points.pcd", pc_3d_points)
    # return pc_3d_points
    print(img[300,1000])
    print(img[300][1000])
    ans=[]
    print(img.shape)
    img_z = np.zeros((img.shape[0],img.shape[1]))
    #反归一化到实际坐标系
    for i in range(img_z.shape[0]):
        for j in range(img_z.shape[1]):
            if  img[i][j]!=0:
                img_z[i][j]=(img[i][j]*3.25758283/80)+0.24174337

    #图像和数组的横纵坐标相反
    for i in range(img_z.shape[0]):
        for j in range(img_z.shape[1]):
            if img_z[i][j]>0.5:
                c=[]
                # x_i = i - CAM_CX
                # y_i = j - CAM_CY
                # pc_x = img_z[i][j] * x_i / CAM_FX
                # pc_y = img_z[i][j] * y_i / CAM_FY
                # c.append(img_z[i][j])
                # c.append(pc_y)
                # c.append(pc_x)
                x_i = j - CAM_CX
                y_i = i - CAM_CY
                pc_x = img_z[i][j] * x_i / CAM_FX
                pc_y = img_z[i][j] * y_i / CAM_FY
                c.append(img_z[i][j])
                c.append(pc_x)
                c.append(pc_y)
                ans.append(c)

    pc=np.array(ans)


    print(pc.shape)
    pc_points=o3d.geometry.PointCloud()
    pc_points.points=o3d.utility.Vector3dVector(pc)
    #
    # points_colors = np.full((pc.shape[0], 3), 0.0)
    # points_colors[:, 0] = 1.0;
    # pc_points.colors = o3d.utility.Vector3dVector(points_colors)
    #
    pc_3d_points=pc_points.remove_non_finite_points(True,True)
    print(pc_3d_points)

    o3d.io.write_point_cloud("complete_points.pcd", pc_3d_points)
    return pc_3d_points
    #np.savetxt('pc.csv', pc, delimiter=',',fmt='%s', newline='\n')

#读取雷达点云
rslidsr_test_data_dir = 'E:\pointdata\Test1101'
rslidar_point_cloud_file_name = '221123.pcd'
rslidar_point_cloud_file_path = os.path.join(rslidsr_test_data_dir, rslidar_point_cloud_file_name)

rslidar_pcd = o3d.io.read_point_cloud(rslidar_point_cloud_file_path)
#rslidar_pcd.paint_uniform_color([1,0,1])
rslidar_pcd = rslidar_pcd.remove_non_finite_points(True, True)  # 剔除无效值

print(rslidar_pcd)
rslidar_points=np.asarray(rslidar_pcd.points)
print(rslidar_points)

cam_p=[ [350.715, 0, 320.020],
        [0, 350.715, 182.2878],
        [0, 0, 1]]

rslidar_pcd_points=pass_through(rslidar_pcd,0,math.inf,"x")

print(rslidar_pcd_points.points)
#o3d.io.write_point_cloud("zhitongFilter.pcd",rslidar_pcd_points)

#img=np.genfromtxt('dep_rot.csv',delimiter=',').astype(np.float32)
img=cv.imread("img_n_1123_Ans4.png",-1)


print(img)

pc_points=depthToPc(img)
#PointToDense(rslidar_pcd_points)
#
o3d.visualization.draw_geometries([pc_points])
#o3d.visualization.draw_geometries([rslidar_pcd_points])