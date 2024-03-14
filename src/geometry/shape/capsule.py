from typing import List  # 从typing模块导入List类型，用于类型注解

import numpy as np  # 导入numpy库，并简称为np，用于科学计算
from spatialmath import SE3  # 从spatialmath包导入SE3类，用于表示3D空间中的位姿

from .. import Geometry  # 从上级目录导入Geometry类
from ..simplex import Point, UnitVector, Support  # 从simplex模块导入Point, UnitVector, Support类
from .geometry3d import Geometry3D  # 从当前目录的geometry3d模块导入Geometry3D类


class Capsule(Geometry3D, Support):  # 定义Capsule类，继承自Geometry3D和Support
    def __init__(self, base: SE3, radius: float, length: float) -> None:
        super().__init__(base)  # 调用父类构造器，初始化基础位姿
        self.radius = radius  # 设置胶囊体的半径
        self.length = length  # 设置胶囊体的长度

    @property
    def points(self) -> List[Geometry]:
        return []  # 返回一个空列表，Capsule类暂不具体实现此属性

    def calculate_support_point(self, d: UnitVector) -> Point:
        # 计算在给定方向d上的支撑点
        if np.dot(d.get_t(), self.base.a) > 0.0:
            # 如果方向d与胶囊体的轴向点乘结果大于0，则计算正半轴上的支撑点
            return Point(np.squeeze(self.base * np.array([0, 0, self.length * 0.5])) + d.get_t() * self.radius)
        # 否则，计算负半轴上的支撑点
        return Point(np.squeeze(self.base * np.array([0, 0, -self.length * 0.5])) + d.get_t() * self.radius)

    def plot(self, ax, c=None):
        # 绘制胶囊体的方法
        num1 = 20  # 横向分割数
        num2 = 20  # 纵向分割数

        theta = np.linspace(0, 2 * np.pi, num1)  # 生成theta值
        z = np.linspace(-0.5 * self.length, 0.5 * self.length, num2)  # 生成z值
        theta, z = np.meshgrid(theta, z)  # 生成网格数据
        x = self.radius * np.cos(theta)  # 计算x坐标
        y = self.radius * np.sin(theta)  # 计算y坐标

        coordinates = self.calculate_coordinates(x, y, z)  # 计算坐标变换
        ax.plot_surface(*coordinates, alpha=0.5, color='blue' if c is None else c)  # 绘制胶囊体中间部分的表面

        theta = np.linspace(0, 2 * np.pi, num1)  # 再次生成theta值
        phi = np.linspace(0, np.pi / 2, num2)  # 生成phi值
        theta, phi = np.meshgrid(theta, phi)  # 生成网格数据
        x = self.radius * np.cos(theta) * np.sin(phi)  # 计算x坐标
        y = self.radius * np.sin(theta) * np.sin(phi)  # 计算y坐标
        z = self.radius * np.cos(phi) + self.length * 0.5  # 计算z坐标
        coordinates = self.calculate_coordinates(x, y, z)  # 计算坐标变换
        ax.plot_surface(*coordinates, alpha=0.5, color='blue' if c is None else c)  # 绘制胶囊体顶部半球面

        x = self.radius * np.cos(theta) * np.sin(phi)  # 计算x坐标
        y = self.radius * np.sin(theta) * np.sin(phi)  # 计算y坐标
        z = -self.radius * np.cos(phi) - self.length * 0.5  # 计算z坐标，绘制胶囊体底部半球面
        coordinates = self.calculate_coordinates(x, y, z)  # 计算坐标变换
        ax.plot_surface(*coordinates, alpha=0.5, color='blue' if c is None else c)  # 绘制胶囊体底部半球面
