import open3d as o3d
import argparse
import multiprocessing
from pathlib import Path



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='show ply')
    parser.add_argument("--file",
                    required=True,
                    help="ply file")
    args = parser.parse_args()
    ply_file = Path(args.file)
    pcd = o3d.io.read_point_cloud(str(ply_file))
    o3d.visualization.draw_geometries([pcd],width=640,height=480,
            front = [ -0.094329843857898679, 0.35340736985917526, -0.93070140834048631 ],
			lookat = [ 0.60399267991822003, -1.3006209037466356, 3.38671875 ],
			up = [ 0.0061660195939751382, -0.93464466190708462, -0.35552965582488116 ],
            zoom= 0.49999999999999978)