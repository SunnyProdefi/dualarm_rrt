from typing import List  # 从typing模块导入List，用于类型注解
from abc import (
    ABC,
    abstractmethod,
)  # 从abc模块导入ABC和abstractmethod，用于定义抽象基类和抽象方法
import numpy as np  # 导入numpy库，重命名为np，用于进行数学计算

from src.geometry.simplex.geometry import Geometry  # 从项目指定路径导入Geometry类


class Support(ABC):  # 定义Support类，继承自抽象基类ABC
    @property  # 使用@property装饰器，定义一个抽象属性
    @abstractmethod  # 使用@abstractmethod装饰器，标记这是一个抽象方法
    def points(self) -> List[Geometry]:  # 抽象属性points，返回一个Geometry对象的列表
        pass  # 抽象方法，不提供实现

    def calculate_support_point(self, d: Geometry) -> Geometry:  # 定义计算支撑点的方法
        dot = -np.inf  # 初始化点积值为负无穷大，用于比较找出最大点积
        support_point = self.points[0]  # 默认将支撑点初始化为points列表中的第一个点
        for point in self.points:  # 遍历points中的每一个点
            dot_new = np.dot(point.get_t(), d.get_t())  # 计算当前点与方向d的点积
            if dot_new > dot:  # 如果当前点积大于之前的最大点积
                dot = dot_new  # 更新最大点积值
                support_point = point  # 更新支撑点为当前点

        return support_point  # 返回计算得到的支撑点
