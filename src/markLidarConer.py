import numpy as np
import open3d as o3d
import os

def scale_point_cloud(pcd, scale):
    points = np.asarray(pcd.points)
    scaled_points = points * scale
    # 创建新的点云对象
    scaled_pcd = o3d.geometry.PointCloud()
    # 将缩放后的坐标设置给新的点云对象
    scaled_pcd.points = o3d.utility.Vector3dVector(scaled_points)
    return scaled_pcd

def pick_points(pcd):
    print("Please pick at least four points on the calibration board using [shift + left click]")
    print("Press [shift + right click] to remove the selection")
    print("Press [ESC] to quit")
    vis = o3d.visualization.VisualizerWithVertexSelection()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    print("Selected points:")
    points = vis.get_picked_points()
    vis.destroy_window()
    return points

def markPcd(pcd_path, out_path):
    pcd = o3d.io.read_point_cloud(pcd_path)
    selected_points = pick_points(pcd)

    lines = [pcd_path + "\n"]
    idx = 1
    pts = [(point.coord[0], point.coord[1], point.coord[2]) for point in selected_points]

    def sort_pts(elem):
        return elem[0], elem[2]
    
    pts = sorted(pts, key=sort_pts)
    
    if pts[0][-1] < pts[1][-1]:
        pts[0], pts[1] = pts[1], pts[0]
    if pts[2][-1] < pts[3][-1]:
        pts[2], pts[3] = pts[3], pts[2]
    
    for point in pts:
        lines.append("Point {}\n".format(idx))
        idx += 1
        lines.append("{} {} {}\n".format(point[0], point[1], point[2]))
    
    with open(out_path, 'a') as f:
        f.writelines(lines)

    print("{} corner >> {}".format(pcd_path, out_path))

def main():
    # Load point cloud
    pcd = o3d.io.read_point_cloud("/home/fyh/data/GPcar/calib/2024-04-22-17-02-02/pcd/1713776539.800076962.pcd")

    # Select points on the calibration board
    # pcd = scale_point_cloud(pcd, 3)
    selected_points = pick_points(pcd)
    print(selected_points)
    print(selected_points[0].coord)

if __name__ == "__main__":
    folders = [
        "/home/fyh/data/GPcar/calib/2024-04-22-17-02-17",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-02-31",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-02-43",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-04-59",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-05-18",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-05-48",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-06-01",
        "/home/fyh/data/GPcar/calib/2024-04-22-17-06-19",
    ]
    out_path = "/home/fyh/data/GPcar/calib/lidar_corners.txt"

    for folder in folders:
        pcd_folder = os.path.join(folder, "pcd")
        pcd_path = sorted(os.listdir(pcd_folder))[0]
        pcd_path = os.path.join(pcd_folder, pcd_path)

        markPcd(pcd_path, out_path)

    # markPcd("/home/fyh/data/GPcar/calib/2024-04-22-17-02-02/pcd/1713776539.800076962.pcd", "test.txt")