import copy  # 导入copy模块，用于深复制对象
from typing import Tuple  # 从typing模块导入Tuple，用于类型注解

import numpy as np  # 导入numpy库，用于科学计算

from src.constanst import MathConst  # 从src.constants模块导入MathConst，其中定义了数学常量
from ..simplex import Point, SimplexFactoryPool, SimplexParameter, UnitVector, Support  # 从simplex模块导入相关类和接口

# 定义GJK类
class GJK:

    @staticmethod
    def calculate_distance(shape0: Support, shape1: Support) -> float:
        # 计算两个形状之间的最短距离，仅返回距离值
        return GJK.calculate_distance_and_points(shape0, shape1)[0]

    @staticmethod
    def calculate_distance_and_points(shape0: Support, shape1: Support) -> Tuple[float, Tuple]:
        # 计算两个形状之间的最短距离及相应的最近点
        vec = UnitVector(np.array([1, 0, 0]))  # 初始化方向向量
        point0 = shape0.calculate_support_point(vec)  # 计算形状0在方向vec上的支撑点
        point1 = shape1.calculate_support_point(-vec)  # 计算形状1在方向-vec上的支撑点
        point = point0 - point1  # 计算Minkowski差

        origin = Point([0, 0, 0])  # 原点

        closest_point = point  # 初始化最近点为Minkowski差中的点
        comparator = lambda x: np.linalg.norm(x.get_t())  # 定义比较函数，计算点到原点的距离
        points = [point]  # 初始化点集合
        points0 = [point0]  # 形状0的支撑点集合
        points1 = [point1]  # 形状1的支撑点集合

        coordinates = [1]  # 初始化barycentric坐标

        finish = False  # 初始化结束标志

        while closest_point != origin:
            closest = copy.deepcopy(closest_point)  # 复制当前最近点
            vec = -UnitVector(closest_point)  # 更新搜索方向为最近点的反方向
            point0 = shape0.calculate_support_point(vec)
            point1 = shape1.calculate_support_point(-vec)
            point = point0 - point1

            for point_i in points:  # 检查是否重复点
                if np.linalg.norm((point - point_i).get_t()) < MathConst.ERROR:
                    finish = True
                    break
            if finish:
                break

            points.append(point)
            points0.append(point0)
            points1.append(point1)

            if len(points) == 5:  # 如果点集合中的点数量达到5，移除最远的点
                coordinate_min = min(coordinates)
                coordinate_min_index = coordinates.index(coordinate_min)
                points.pop(coordinate_min_index)
                points0.pop(coordinate_min_index)
                points1.pop(coordinate_min_index)
                break

            simplex_parameter = SimplexParameter(points)
            simplex = SimplexFactoryPool.create_product(simplex_parameter)
            closest_point = simplex.calculate_closest_point_to_origin()
            coordinates = simplex.calculate_barycentric_coordinates(closest_point)

            if np.linalg.norm((closest - closest_point).get_t()) < MathConst.EPS:
                break

            j = 0
            for i, coordinate in enumerate(coordinates):  # 移除贡献小的点
                if abs(coordinate) < MathConst.EPS:
                    points.pop(i - j)
                    points0.pop(i - j)
                    points1.pop(i - j)
                    j = j + 1

        simplex_parameter = SimplexParameter(points)
        simplex = SimplexFactoryPool.create_product(simplex_parameter)
        coordinates = simplex.calculate_barycentric_coordinates(closest_point)

        point0 = Point(np.zeros_like(point0.get_t()))
        point1 = Point(np.zeros_like(point1.get_t()))
        for i, coordinate_i in enumerate(coordinates):  # 计算最近点
            point0 += coordinate_i * points0[i]
            point1 += coordinate_i * points1[i]

        return np.linalg.norm(closest_point.get_t()), (point0, point1)  # 返回距离和最近点对

    @staticmethod
    def is_intersecting(shape0: Support, shape1: Support):
        # 检查两个形状是否相交的静态方法
        origin = Point([0, 0, 0])  # 创建原点
        unit_vector = UnitVector(np.array([1, 0, 0]))  # 初始化搜索方向为x轴正方向的单位向量

        # 计算两个形状在初始搜索方向上的Minkowski差
        point = shape0.calculate_support_point(unit_vector) - shape1.calculate_support_point(-unit_vector)
        points = [point]  # 初始化点集，包含Minkowski差中的第一个点
        closest_point = point  # 初始化最近点为Minkowski差中的第一个点

        coordinates = [1]  # 初始化barycentric坐标

        while closest_point != origin:
            # 当最近点不是原点时继续迭代
            unit_vector = -UnitVector(closest_point)  # 更新搜索方向为最近点到原点的反方向
            # 计算新的Minkowski差点
            point = shape0.calculate_support_point(unit_vector) - shape1.calculate_support_point(-unit_vector)
            if np.dot(point.get_t(), unit_vector.get_t()) < 0:
                # 如果新点与搜索方向的点积小于0，表示两形状不相交
                return False
            points.append(point)  # 添加新的Minkowski差点到点集

            if len(points) == 5:
                # 如果点集中的点数达到5，移除最远的点
                coordinate_min = min(coordinates)
                coordinate_min_index = coordinates.index(coordinate_min)
                points.pop(coordinate_min_index)
                break

            simplex_parameter = SimplexParameter(points)  # 创建Simplex参数
            simplex = SimplexFactoryPool.create_product(simplex_parameter)  # 通过Simplex工厂创建Simplex对象
            closest_point = simplex.calculate_closest_point_to_origin()  # 计算最接近原点的点
            coordinates = simplex.calculate_barycentric_coordinates(closest_point)  # 计算barycentric坐标

            j = 0
            for i, coordinate in enumerate(coordinates):
                # 移除贡献小的点
                if abs(coordinate) < MathConst.EPS:
                    points.pop(i - j)
                    j = j + 1

        return True  # 如果循环完成没有提前返回False，则表示两形状相交，返回True
