import argparse
import multiprocessing
from pathlib import Path
import open3d as o3d
import numpy as np
import cv2


version = "0.1.2"
def save_ply(output_path, points, rgb):

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(rgb)

    #pcd.estimate_normals()
    o3d.io.write_point_cloud(output_path, pcd)

def get_points_in_cam_space(img, intrinsic):
    points = np.empty([img.shape[0]*img.shape[1],3],dtype = float)

    for h in range(img.shape[0]):
        for w in range(img.shape[1]): 
            z = img[h,w]
            x = z * (w-intrinsic[0,2])/intrinsic[0,0]
            y = z * (h-intrinsic[1,2])/intrinsic[1,1]
            points[h*img.shape[1]+w] = np.array([x,y,z])
            
    return points

def save_pclouds(record_path,depth_path,intrinsic_path,rgb_path,ply_path):

    print("Saving point clouds")

    depth_paths = sorted(depth_path.glob('*[0-9].pfm'))
    rgb_paths = sorted(rgb_path.glob('*[0-9].png'))
    intrinsic_paths = sorted(intrinsic_path.glob('*[0-9].txt'))

    assert len(list(depth_paths)) == len(list(intrinsic_paths))
    assert len(list(depth_paths)) == len(list(rgb_paths))

    for i, path in enumerate(depth_paths):
        
        depth = cv2.imread(str(path), -1)
        img =  cv2.imread(str(rgb_paths[i]), -1)
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        intrinsic =  np.loadtxt(str(intrinsic_paths[i]),delimiter=',')
        cv2.imshow('RGB', img)
        cv2.imshow('Depth', depth)
        points=get_points_in_cam_space(depth,intrinsic)
        rgb_points = np.zeros_like(points)
        colors = rgb.flatten().reshape(rgb.shape[0]*rgb.shape[1],3)

        rgb_points = colors / 255.

        save_ply(str(ply_path)+'/'+path.stem+'.ply', points,rgb_points)
        key = cv2.waitKey(1)
        if key == 27 :
            break

# ,np.tile(rgb.flatten().reshape((-1, 1)), (1, 3))


# save_pclouds.py --path=E:/record3d/data/2020-10-27-19-37-24
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Save pcloud.')
    parser.add_argument("--path",
                    required=True,
                    help="Path to recording folder")

    args = parser.parse_args()
    record_path = Path(args.path)
    depth_path = Path(str(record_path)+'/depth')
    ply_path = Path(str(record_path)+'/ply')
    intrinsic_path = Path(str(record_path)+'/intrinsic')
    rgb_path = Path(str(record_path)+'/rgb')

    print("version : " + version)
    print("record path : " +str(record_path))

    if record_path.exists() and depth_path.exists() and intrinsic_path.exists() and rgb_path.exists() :
        ply_path.mkdir(exist_ok=True)
        save_pclouds(record_path,depth_path,intrinsic_path,rgb_path,ply_path)




