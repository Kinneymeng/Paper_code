import numpy as np
import imageio

CAM = 2

def load_velodyne_points(filename):
    points = np.fromfile(filename, dtype=np.float32).reshape(-1, 4)
    #points = points[:, :3]  # exclude luminance
    return points

def load_calib(calib_dir):
    # P2 * R0_rect * Tr_velo_to_cam * y
    lines = open(calib_dir).readlines()
    lines = [ line.split()[1:] for line in lines ][:-1]
    kernal=np.array([-0.0585408,-0.00313091,0.99828,-0.997846,-0.0294629,-0.0586078,0.0295957,-0.999561,0.00139939]).reshape((3,3))
    print(kernal)
    kernal_inv=np.linalg.inv(kernal)
    print(kernal_inv)
    print(np.dot(kernal,kernal_inv))
    #
    P = np.array(lines[CAM]).reshape(3,4)
    #
    Tr_velo_to_cam = np.array(lines[5]).reshape(3,4)
    Tr_velo_to_cam = np.concatenate(  [ Tr_velo_to_cam, np.array([0,0,0,1]).reshape(1,4)  ]  , 0     )
    #
    R_cam_to_rect = np.eye(4)
    R_cam_to_rect[:3,:3] = np.array(lines[4][:9]).reshape(3,3)
    #
    P = P.astype('float32')
    Tr_velo_to_cam = Tr_velo_to_cam.astype('float32')
    R_cam_to_rect = R_cam_to_rect.astype('float32')
    print(P)
    print(Tr_velo_to_cam)
    print(R_cam_to_rect)
    return P, Tr_velo_to_cam, R_cam_to_rect

def prepare_velo_points(pts3d_raw):
    '''Replaces the reflectance value by 1, and tranposes the array, so
        points can be directly multiplied by the camera projection matrix'''
    pts3d = pts3d_raw
    print(pts3d.shape)
    # Reflectance > 0
    indices = pts3d[:, 3] > 0
    pts3d = pts3d[indices ,:]
    pts3d[:,3] = 1
    print(pts3d.shape)
    print(pts3d)
    return pts3d.transpose(), indices

def project_velo_points_in_img(pts3d, T_cam_velo, Rrect, Prect):
    pts3d_cam = Rrect.dot(T_cam_velo.dot(pts3d))
    print(pts3d_cam.shape)
    print(pts3d_cam)
    # Before projecting, keep only points with z>0
    # (points that are in fronto of the camera).
    idx = (pts3d_cam[2,:]>0)
    pts2d_cam = Prect.dot(pts3d_cam[:,idx])
    print(pts2d_cam.shape)
    print(pts2d_cam)
    return pts3d[:, idx], pts2d_cam/pts2d_cam[2,:], idx


def align_img_and_pc(img_dir, pc_dir, calib_dir):
    #img=imread(img_dir)
    img = imageio.imread(img_dir)
    pts = load_velodyne_points( pc_dir )
    P, Tr_velo_to_cam, R_cam_to_rect = load_calib(calib_dir)

    pts3d, indices = prepare_velo_points(pts)
    pts3d_ori = pts3d.copy()
    reflectances = pts[indices, 3]
    pts3d, pts2d_normed, idx = project_velo_points_in_img( pts3d, Tr_velo_to_cam, R_cam_to_rect, P  )
    print(pts2d_normed.shape)
    print(pts2d_normed)
    #print reflectances.shape, idx.shape
    reflectances = reflectances[idx]
    #print reflectances.shape, pts3d.shape, pts2d_normed.shape
    assert reflectances.shape[0] == pts3d.shape[1] == pts2d_normed.shape[1]

    rows, cols = img.shape[:2]
    print(rows)
    print(cols)
    points = []
    for i in range(pts2d_normed.shape[1]):
        c = int(np.round(pts2d_normed[0,i]))
        r = int(np.round(pts2d_normed[1,i]))
        #
        if c < cols and r < rows and r > 0 and c > 0:
            color = img[r, c, :]
            point = [ pts3d[0,i], pts3d[1,i], pts3d[2,i], color[0], color[1], color[2] ]
            points.append(point)


    points = np.array(points)
    print(points.shape)
    return points

# update the following directories
IMG_ROOT = '/home/nvidia/1-minikitti/My_test/image_2/'
PC_ROOT = '/home/nvidia/1-minikitti/My_test/velodyne/'
CALIB_ROOT = '/home/nvidia/1-minikitti/My_test/calib/'
PC_CROP_ROOT = '/home/nvidia/1-minikitti/My_test/crop/'


for frame in range(0, 1):
    img_dir = IMG_ROOT + '%06d.png' % frame
    pc_dir = PC_ROOT + '%06d.bin' % frame
    calib_dir = CALIB_ROOT + '%06d.txt' % frame

    points = align_img_and_pc(img_dir, pc_dir, calib_dir)
    
    output_name = PC_CROP_ROOT + '%06d.bin' % frame
    print('Save to %s' % output_name)
    points[:,:6].astype('float32').tofile(output_name)






