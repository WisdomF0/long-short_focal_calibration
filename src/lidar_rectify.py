import cv2
from PIL import Image
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib import cm

cbar = cm.get_cmap("hsv")

def get_color(x):
    if isinstance(x, float) or isinstance(x, int):
        color = cbar(x)
        return np.array([color[2] * 255, color[1] * 255, color[0] * 255], dtype=np.uint8)
    elif isinstance(x, np.ndarray):
        return np.array([get_color(a) for a in x])

R = np.array([
    [-0.00169462614292543, -0.999997589845237, -0.00139590327483229],
    [-0.0122662134897505, 0.00141658691663275, -0.999923763738082],
    [0.999923331185376, -0.00167737450338281, -0.0122685845115003],
])

rvec = cv2.Rodrigues(R)[0]
print(rvec)

tvec = np.array([
    [0.120433501941947], 
    [-0.200273610481785],
    [-0.429270454118851]
])

cameraMatrix = np.array([
    [4612.90240537928, 0, 1304.11165822944],
    [0, 4612.91604394792, 697.721329128718],
    [0, 0, 1]
])

distCoeffs = np.float64([-0.209154573862668, -0.0103389956810070, 0, 0])

img_file = "/home/fyh/data/GPcar/calib/front_30_int/000001.jpg"
lidar_file = "/home/fyh/data/GPcar/calib_lidar_left/pcd/000001.pcd"

cloud = o3d.io.read_point_cloud(lidar_file)
cloud = np.asarray(cloud.points)
indices = np.where(cloud[:, 0] > 0)
cloud = cloud[indices]
print(cloud.shape)
distance = np.sqrt(cloud[:, 0] ** 2 + cloud[:, 1] ** 2 + cloud[:, 2] ** 2)
distance = (distance - distance.min()) / (distance.max() - distance.min()) * 0.8

pt2d, _ = cv2.projectPoints(cloud, rvec, tvec, cameraMatrix, distCoeffs)
# pt2d = np.reshape(pt2d, (-1, 2))
# pt2d_int = np.round(pt2d).astype(int)
# print(pt2d_int.shape)

size = (2560, 1440)

im = Image.open(img_file)
im = im.resize(size)
print(im.size)
x = []
y = []

m = -1
for point in pt2d:
    m = m+1
    x_2d = point[0][0]
    y_2d = point[0][1]

    if 0 <= x_2d <= size[0] and 0 <= y_2d <= size[1]:
        x.append(x_2d)
        y.append(y_2d)

x = np.array(x)
y = np.array(y)
plt.scatter(x, y, s=1)
plt.imshow(im)
plt.show()

# indices = np.where((pt2d_int[:, 0] >= 0) & (pt2d_int[:, 0] < size[0]) & 
#                    (pt2d_int[:, 1] >= 0) & (pt2d_int[:, 1] < size[1]))[0]
# print(indices.shape)

# image = cv2.imread(img_file)
# image[pt2d_int[indices][:, 1], pt2d_int[indices][:, 0]] = get_color(distance[indices])

# cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
# cv2.imshow("Image", image)
# cv2.waitKey()

# cv2.destroyAllWindows()
# color = [cbar((dist - distance.min()) / (distance.max() - distance.min()) * 0.8) for dist in distance]
# print(color[0])
