import numpy as np
import pylas
from scipy.spatial import cKDTree, ConvexHull
import rasterio
from rasterio.transform import from_origin
from pykrige.ok import OrdinaryKriging
import matplotlib.path as mpath
import matplotlib.pyplot as plt

def load_las_data(file_path):
    with pylas.open(file_path) as f:
        las = f.read()
        points = np.vstack((las.x, las.y, las.z)).transpose()
    return points

def idw_interpolation_to_tiff(points, resolution=1.0, output_file='idw_output.tif'):
    x, y, z = points[:, 0], points[:, 1], points[:, 2]

    # 使用仅 x 和 y 坐标创建凸包
    hull = ConvexHull(points[:, :2])
    # 使用凸包顶点创建 Path，但仅使用 x, y
    hull_path = mpath.Path(points[hull.vertices, :2])  # 切片以仅取 x 和 y 坐标
    # 计算网格范围
    x_min, x_max, y_min, y_max = np.min(x), np.max(x), np.min(y), np.max(y)
    grid_x, grid_y = np.meshgrid(np.arange(x_min, x_max, resolution),
                                 np.arange(y_min, y_max, resolution))

    # 创建KD树并查询
    tree = cKDTree(points[:, :2])
    flat_grid = np.c_[grid_x.ravel(), grid_y.ravel()]
    distances, indices = tree.query(flat_grid, k=4)
    epsilon = 1e-10
    weights = 1 / (distances + epsilon)

    # 应用掩膜
    mask = hull_path.contains_points(flat_grid)  # 检查每个网格点是否在凸包内
    masked_weights = weights * mask[:, np.newaxis]  # 掩膜应用于权重
    interpolated_values = np.sum(masked_weights * z[indices], axis=1) / np.sum(masked_weights, axis=1)
    grid_z = interpolated_values.reshape(grid_x.shape)

    # 处理非凸包区域
    grid_z[~mask.reshape(grid_x.shape)] = np.nan  # 设置凸包外的值为NaN

    # 保存为TIFF
    transform = from_origin(x_min, y_min, resolution, -resolution)
    with rasterio.open(output_file, 'w', driver='GTiff', height=grid_z.shape[0], width=grid_z.shape[1], count=1, dtype=grid_z.dtype, transform=transform) as dst:
        dst.write(grid_z, 1)


# Path to your LAS file
#las_file_path = 'D:/ASP_files/XSY_code/test_lavatube/MHP/PC/13/13_allpart_newtest5_0.5_1.las'
las_file_path = 'D:/ASP_files/XSY_code/test_lavatube/MHP/PC/11/11_allpart_newtest4_0.5_1.las'

# Load the data
point_cloud = load_las_data(las_file_path)

# Perform IDW interpolation and save as TIFF
idw_interpolation_to_tiff(point_cloud, resolution=2.0, output_file='D:/ASP_files/XSY_code/test_lavatube/MHP/PC/11/11_allpart_newtest4_0.5_1.tif')


