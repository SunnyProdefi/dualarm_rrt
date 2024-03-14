import copy  # 导入copy模块，用于深复制对象
from typing import List  # 从typing模块导入List类，用于类型注解

import numpy as np  # 导入numpy库，并将其重命名为np，用于高效的数学计算

from src.constanst import MathConst  # 从src.constants模块导入MathConst类，用于使用数学常量

from .geometry import Geometry  # 从当前包导入Geometry类
from .simplex import Simplex  # 从当前包导入Simplex类
from .interface import Support  # 从当前包导入Support接口
from .point import Point  # 从当前包导入Point类
from .line import Line  # 从当前包导入Line类


class LineSegment(Line, Simplex, Support):  # 定义LineSegment类，继承Line, Simplex, 和 Support
    def get_length(self) -> float:  # 定义获取线段长度的方法
        return self.length  # 返回线段的长度

    @property
    def points(self) -> List[Point]:  # 定义一个属性，返回线段的两个端点
        return copy.deepcopy([self.point0, self.point1])  # 返回线段端点的深复制列表

    def calculate_closest_point_to_origin(self) -> Geometry:  # 计算并返回线段上距原点最近的点
        v = self.point0.get_t()  # 获取第一个端点的坐标
        w = self.point1.get_t()  # 获取第二个端点的坐标
        p = np.zeros_like(v)  # 创建一个与v形状相同、元素全为0的数组

        if np.array_equal(v, w):  # 如果两个端点坐标相同
            return Point(v)  # 直接返回任一端点作为距原点最近的点

        t = (p - v).dot(w - v) / ((w - v).dot(w - v))  # 计算最近点在线段上的参数化表示中的参数t
        t = max(0.0, min(1.0, t))  # 确保t在[0, 1]区间内，以保证点在线段上
        return Point(v + t * (w - v))  # 返回计算出的最近点

    def calculate_barycentric_coordinates(self, geometry: Geometry) -> List[float]:  # 计算给定几何体相对于线段的重心坐标
        v0 = (self.points[1] - self.points[0]).get_t()  # 计算线段向量
        v1 = (geometry - self.points[0]).get_t()  # 计算几何体到线段起点的向量

        d00 = np.dot(v0, v0)  # 计算v0与自身的点积
        if np.abs(d00) < MathConst.EPS:  # 如果点积接近0，说明线段长度几乎为0
            return [1.0, 0.0]  # 在这种情况下，重心坐标为[1, 0]

        d01 = np.dot(v0, v1)  # 计算v0与v1的点积

        v = d01 / d00  # 计算v1在v0上的投影长度比例
        u = 1.0 - v  # 计算重心坐标的另一个值

        return [u, v]  # 返回重心坐标[u, v]

    def plot(self, ax, c=None):  # 定义绘制线段的方法
        ax.plot([self.point0.get_tx(), self.point1.get_tx()],
                [self.point0.get_ty(), self.point1.get_ty()],
                [self.point0.get_tz(), self.point1.get_tz()],
                c='g' if c is None else c)  # 使用matplotlib绘制线段，如果颜色未指定，则默认为绿色
