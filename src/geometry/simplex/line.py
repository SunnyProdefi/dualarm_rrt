import copy  # 导入copy模块，用于对象的深复制
from typing import overload, Union, Iterable, List, Tuple  # 导入类型注解工具

import numpy as np  # 导入numpy库，用于数学运算

from .point import Point  # 从当前包导入Point类


class Line:
    # 重载构造函数，允许不同的参数类型和数量
    @overload
    def __init__(self) -> None:  # 没有参数的情况
        ...

    @overload
    def __init__(self, point0: Point, point1: Point) -> None:  # 两个Point类型参数的情况
        ...

    @overload
    def __init__(self, point0: Union[np.ndarray, Iterable, int, float],  # 接受多种类型参数的情况
                 point1: Union[np.ndarray, Iterable, int, float]) -> None:
        ...

    @overload
    def __init__(self, point0: List[Point], point1: None) -> None:  # 一个列表和None的情况
        ...

    # 实际的构造函数实现
    def __init__(self, point0=None, point1=None) -> None:
        super().__init__()  # 调用父类的构造函数
        if (point0 is None) and (point1 is None):  # 如果两个参数都为None
            self.point0 = Point()  # 创建默认的Point实例
            self.point1 = Point()
        elif isinstance(point0, (List, Tuple)) and (point1 is None):  # 如果point0是列表或元组，且point1为None
            self.point0 = point0[0]  # 取列表或元组的第一个和第二个元素作为点
            self.point1 = point0[1]
        elif isinstance(point0, (Point, np.ndarray, Iterable, int, float)) \
                and isinstance(point1, (Point, np.ndarray, Iterable, int, float)):  # 如果point0和point1是指定的类型
            self.point0 = Point(point0)  # 创建Point实例
            self.point1 = Point(point1)
        else:  # 如果参数类型不符
            raise ValueError("构造函数的参数不正确")  # 抛出异常
        # 计算两点之间的距离作为线段长度
        self.length = np.linalg.norm((self.point1 - self.point0).get_t())

    # 获取第一个点的深复制
    def get_point0(self) -> Point:
        return copy.deepcopy(self.point0)

    # 获取第二个点的深复制
    def get_point1(self) -> Point:
        return copy.deepcopy(self.point1)


if __name__ == '__main__':  # 如果直接运行此文件
    t0 = np.array([0.0, 0.0])  # 定义第一个点的坐标
    t1 = np.array([0.2, 0.2])  # 定义第二个点的坐标

    point0 = Point(t0)  # 创建第一个点
    point1 = Point(t1)  # 创建第二个点

    line = Line(point0, point1)  # 创建线段实例
    print('长度: ', line.get_length())  # 打印线段长度

    line2 = Line()  # 创建默认线段实例
    print('长度: ', line2.get_length())  # 打印线段长度

    line3 = Line((0, 0, 0), (1, 1, 1))  # 使用元组直接创建线段实例
    print('长度: ', line3.get_length())  # 打印线段长度

    line4 = Line([0, 0, 0], [4, 4, 4])  # 使用列表直接创建线段实例
    print('长度: ', line4.get_length())  # 打印线段长度
