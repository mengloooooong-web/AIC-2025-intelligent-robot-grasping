# 一些工具函数和一些变量

import numpy as np
import cv2

# sudo apt install ros-humble-tf-transformations
# pip3 install --upgrade transforms3d -i https://pypi.tuna.tsinghua.edu.cn/simple
try:
    import tf_transformations as tf
except ImportError:
    import transforms3d.euler as tfe
    import transforms3d.quaternions as tfq
    
    class TFWrapper:
        @staticmethod
        def quaternion_from_euler(*args, axes='sxyz'):
            if len(args) == 3:
                return tfe.euler2quat(args[0], args[1], args[2], axes=axes)
            return tfe.euler2quat(*args)
        
        @staticmethod
        def translation_matrix(pos):
            mat = np.eye(4)
            mat[:3, 3] = pos
            return mat
        
        @staticmethod
        def quaternion_matrix(quat):
            mat = np.eye(4)
            mat[:3, :3] = tfq.quat2mat(quat)
            return mat
        
        @staticmethod
        def concatenate_matrices(*matrices):
            result = matrices[0]
            for mat in matrices[1:]:
                result = np.dot(result, mat)
            return result
    
    tf = TFWrapper()


# 参数类
class Params:
    # # 相机内参矩阵
    # from:opt/Inuitive/InuDev/config/InuSensors/ABA02220235/temperature_low/Binning/NU4K/SystemParameters.yml
    # line:90
    camera_matrix = np.array(
        [
            [390.1836894541749, 0.0, 400.02356020942409],
            [0.0, 390.1836894541749, 307.87696335078533],
            [0.0, 0.0, 1.0],
        ]
    )
    # 图像尺寸
    color_image_size = (800, 600)
    depth_image_size = (544, 360)
    # 手眼标定数据
    handeye_translation = np.array([-0.0558021, 0.0327328, 0.0382299])  # 单位：米
    # rx ry rz
    handeye_rotation_euler = np.array([2.02147, -1.4809, -88.707])  # 单位：度

    # 定义蓝色和黄色的HSV范围（OpenCV中H范围是0-180）
    # 蓝色范围 (100-130)
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([130, 255, 255])

    # 黄色范围 (20-40)
    lower_yellow = np.array([20, 50, 50])
    upper_yellow = np.array([40, 255, 255])


def get_color(cv_image, roi) -> str:
    # 获取ROI的主要颜色
    """
    检测指定ROI区域的主要颜色(蓝色、黄色或其他)

    参数:
        cv_image: 输入图像 (BGR格式)
        roi: 矩形区域 [x_offset, y_offset, width, height]

    返回:
        "blue", "yellow" 或 "other"
    """
    # 提取ROI区域
    x, y, w, h = roi
    roi_img = cv_image[y : y + h, x : x + w]

    # 转换为HSV颜色空间（对光照变化更鲁棒）
    hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)

    # 创建颜色掩模
    mask_blue = cv2.inRange(hsv, Params.lower_blue, Params.upper_blue)
    mask_yellow = cv2.inRange(hsv, Params.lower_yellow, Params.upper_yellow)

    # 计算各颜色像素占比
    total_pixels = w * h
    blue_ratio = cv2.countNonZero(mask_blue) / total_pixels
    yellow_ratio = cv2.countNonZero(mask_yellow) / total_pixels

    # 判断主要颜色（阈值可调整）
    if blue_ratio > 0.3 and blue_ratio > yellow_ratio:
        return "blue"
    elif yellow_ratio > 0.3 and yellow_ratio > blue_ratio:
        return "yellow"
    return "other"


def find_square_contour(image, color_lower, color_upper):
    """
    寻找图像中的某种颜色的矩形轮廓
    :param image: 输入图像
    :param color_lower: 颜色的HSV范围下限
    :param color_upper: 颜色的HSV范围上限
    :return: 矩形轮廓列表
    """
    # 将图像转换为HSV色彩空间
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # 根据颜色范围创建掩膜
    mask = cv2.inRange(hsv, color_lower, color_upper)

    # 对掩膜进行形态学操作，去除噪声
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 寻找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 筛选矩形轮廓
    square_contours = []
    for contour in contours:
        # 计算轮廓的周长和面积
        perimeter = cv2.arcLength(contour, True)
        area = cv2.contourArea(contour)

        # 过滤掉过小的轮廓
        if perimeter < 50 or area < 500:
            continue

        # 获取轮廓的近似多边形
        approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)

        # 如果轮廓有4个顶点，则认为是矩形
        if len(approx) == 4:
            square_contours.append(contour)

    return square_contours


def cut_roi(cv_image, roi):
    """
    保存指定区域的图像到文件
    """
    x, y, w, h = roi
    roi_img = cv_image[y : y + h, x : x + w]
    filename = f"roi_{x}_{y}_{w}_{h}.png"
    cv2.imwrite(filename, roi_img)


def pixel_to_camera(x, y, depth) -> np.ndarray:
    """将深度像素坐标转换为相机坐标系3D坐标
    :param x, y: 深度图中的像素坐标
    :param depth: 深度值（单位：米）
    :return: (X, Y, Z) 相机坐标系下的坐标 单位：米
    """
    inv_camera_matrix = np.linalg.inv(Params.camera_matrix)
    point_2d = np.array([[x], [y], [1]])
    point_3d = inv_camera_matrix @ point_2d
    point_3d = point_3d * depth
    return point_3d.flatten()


def create_handeye_matrix():
    """
    创建手眼标定变换矩阵
    """
    # 将欧拉角（度）转换为弧度
    # rx ry rz
    rotation_euler_rad = np.radians(Params.handeye_rotation_euler)
    # 将欧拉角转换为四元数
    quaternion = tf.quaternion_from_euler(
        rotation_euler_rad[0], rotation_euler_rad[1], rotation_euler_rad[2], axes="rxyz"
    )
    # 构建齐次变换矩阵
    T_tool_cam = tf.concatenate_matrices(
        tf.translation_matrix(Params.handeye_translation),
        tf.quaternion_matrix(quaternion),
    )
    return T_tool_cam


def camera_to_tool(X_c, Y_c, Z_c, T_tool_cam) -> np.ndarray:
    """
    将相机坐标系下的点转换为工具坐标系下的点 单位：米
    :param X_c, Y_c, Z_c: 相机坐标系下的点坐标
    :param T_tool_cam: 手眼标定变换矩阵
    :return: 工具坐标系下的点坐标(X, Y, Z)
    """
    point_camera = np.array([X_c, Y_c, Z_c, 1.0])
    # 应用变换矩阵
    point_tool = np.dot(T_tool_cam, point_camera)[:3]
    return point_tool


def tool_to_base(point_tool, current_pose):
    """
    将工具坐标系下的点转换为基坐标系下的点
    :param point_tool: 工具坐标系下的坐标 [x, y, z] (单位：米)
    :param current_pose: 当前机器人位姿 [x, y, z, rx, ry, rz] (单位：米, 弧度)
    :return: 基坐标系下的坐标 (x, y, z) (单位：米)
    """

    cur_position = current_pose[:3]
    cur_orientation = current_pose[3:6]

    # 将欧拉角转换为四元数
    quaternion = tf.quaternion_from_euler(
        cur_orientation[0], cur_orientation[1], cur_orientation[2]
    )

    # 构建基坐标系到工具坐标系的变换矩阵
    T_base_tool = tf.concatenate_matrices(
        tf.translation_matrix(cur_position), tf.quaternion_matrix(quaternion)
    )

    # 将工具坐标系下的点转换为齐次坐标
    point_tool_hom = np.array([point_tool[0], point_tool[1], point_tool[2], 1.0])

    # 应用变换矩阵
    point_base = np.dot(T_base_tool, point_tool_hom)[:3]

    return point_base


# 示例用法
# cv_image = cv2.imread("./image-336.jpeg")

# roi1 = [629, 159, 62, 38]
# roi2 = [423, 128, 32, 32]
# roi3 = [431, 316, 37, 34]
# roi4 = [453, 174, 32, 34]
# print(get_color(cv_image, roi1))
# print(get_color(cv_image, roi2))
# print(get_color(cv_image, roi3))
# print(get_color(cv_image, roi4))
# cut_roi(cv_image, roi1)
# cut_roi(cv_image, roi2)
# cut_roi(cv_image, roi3)
# cut_roi(cv_image, roi4)