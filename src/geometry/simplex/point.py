import copy  # 导入copy模块，用于深复制对象
from typing import overload, Union, Iterable, List  # 导入类型注解

import numpy as np  # 导入numpy库，用于数学计算

from .geometry import Geometry  # 从当前目录的geometry模块导入Geometry类
from .simplex import Simplex  # 从当前目录的simplex模块导入Simplex类
from .interface import Support  # 从当前目录的interface模块导入Support接口


class Point(Geometry, Simplex, Support):  # 定义Point类，继承自Geometry, Simplex和Support

    @overload
    def __init__(self) -> None:  # 使用overload装饰器重载构造函数，无参数情况
        ...

    @overload
    def __init__(self, t: Union[np.ndarray, Iterable, int, float]) -> None:  # 接受多种类型参数的构造函数重载
        ...

    @overload
    def __init__(self, t: Geometry) -> None:  # 接受Geometry类型参数的构造函数重载
        ...

    def __init__(self, t=None) -> None:  # 实际的构造函数实现
        super().__init__()  # 调用父类构造函数
        if t is None:  # 如果参数t为None
            self.t = np.zeros(1)  # 创建一个元素为0的numpy数组
        elif isinstance(t, (np.ndarray, Iterable, int, float)):  # 如果t是指定的类型
            if isinstance(t[0], Point):  # 如果t的第一个元素是Point类型
                self.t = np.array(t[0].get_t(), dtype=np.float64)  # 获取t的坐标并转换为numpy数组
            else:
                self.t = np.array(t, dtype=np.float64)  # 直接将t转换为numpy数组
        elif isinstance(t, Point):  # 如果t是Point类型
            self.t = np.array(t.get_t())  # 直接获取t的坐标
        else:  # 如果t不符合上述任何一种情况
            raise ValueError("构造函数的参数不正确")  # 抛出异常
        self.dim: int = self.t.size  # 记录坐标的维度

    def get_tx(self) -> float:  # 获取x坐标的方法
        return self.t[0]

    def get_ty(self) -> float:  # 获取y坐标的方法
        return self.t[1]

    def get_tz(self) -> float:  # 获取z坐标的方法
        return self.t[2]

    def set_tx(self, num: float) -> None:  # 设置x坐标的方法
        self.t[0] = num

    def set_ty(self, num: float) -> None:  # 设置y坐标的方法
        self.t[1] = num

    def set_tz(self, num: float) -> None:  # 设置z坐标的方法
        self.t[2] = num

    @property
    def points(self):  # 定义一个属性，返回包含自身的列表
        return [copy.deepcopy(self)]

    def calculate_closest_point_to_origin(self) -> Geometry:  # 计算并返回距原点最近的点
        return copy.deepcopy(self)

    def calculate_barycentric_coordinates(self, geometry: Geometry) -> List[float]:  # 计算重心坐标
        return [1.0]

    def plot(self, ax, c=None):  # 绘制点的方法
        ax.scatter(*self.t, c='r' if c is None else c)  # 在ax上绘制点，颜色默认为红色


if __name__ == '__main__':  # 如果这个文件被直接运行
    t = np.array([2, 2])  # 定义一个坐标
    point = Point(t)  # 创建一个Point对象
   
    tx = point.get_tx()
    ty = point.get_ty()
    tt = point.get_t()

    print('tx: ', tx)
    print('ty: ', ty)
    print('tt: ', tt)

    t2 = np.array([3.0, 5.0])
    point2 = Point(t2)
    point3 = point + point2
    print(point3.get_t())

    point4 = point3 - point2
    print('sub')
    print(point4.get_t())

    print('mul')
    point5 = point4 * 2
    print(point5.get_t())
    point6 = 2 * point4
    print(point6.get_t())

    print('div')
    point7 = point4 / 3
    print(point7.get_t())

    point8 = Point()
    print(point8.get_t())

    point9 = Point((9, 9, 9))
    print(point9.get_t())

    point10 = Point([10, 10, 10])
    print(point10.get_t())
