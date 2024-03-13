import copy  # 导入copy模块，用于执行深复制
from typing import Union, Iterable  # 从typing模块导入Union和Iterable，用于类型注解

import numpy as np  # 导入numpy库，用于科学计算

from .i_check_collision import ICheckCollision  # 从当前目录导入ICheckCollision接口
from .rrt_map import RRTMap  # 从当前目录导入RRTMap类，表示RRT算法的地图
from .check_collision import CheckCollision  # 从当前目录导入CheckCollision类，用于碰撞检测

# 定义RRTParameter类，存储RRT算法的参数
class RRTParameter:

    def __init__(self, start: Union[np.ndarray, Iterable], goal: Union[np.ndarray, Iterable],
                 expand_dis: float = 1.0, goal_sample_rate: float = 10.0, max_iter: int = 100,
                 radius: float = 10.0, animation: bool = False) -> None:
        super().__init__()  # 调用父类的初始化方法
        self.__start = np.array(start)  # 将起始点转换为numpy数组
        self.__goal = np.array(goal)  # 将目标点转换为numpy数组
        self.__expand_dis = expand_dis  # 设置每次扩展的距离
        self.__goal_sample_rate = goal_sample_rate  # 设置采样目标点的概率
        self.__max_iter = max_iter  # 设置最大迭代次数
        self.__radius = radius  # 设置路径规划中考虑的节点的搜索半径
        self.__animation = animation  # 设置是否显示动画

    # 下面的@property装饰器用于创建只读属性
    @property
    def start(self):
        return copy.deepcopy(self.__start)  # 返回起始点的深复制

    @property
    def goal(self):
        return copy.deepcopy(self.__goal)  # 返回目标点的深复制

    @property
    def expand_dis(self):
        return self.__expand_dis  # 返回每次扩展的距离

    @property
    def goal_sample_rate(self):
        return self.__goal_sample_rate  # 返回采样目标点的概率

    @property
    def max_iter(self):
        return self.__max_iter  # 返回最大迭代次数

    @property
    def radius(self):
        return self.__radius  # 返回搜索半径

    @property
    def animation(self):
        return self.__animation  # 返回是否显示动画的标志

    def create_check_collision(self, rrt_map: RRTMap) -> ICheckCollision:
        # 创建并返回一个碰撞检测对象，用于检测路径上的碰撞
        return CheckCollision(rrt_map.obstacles)  # 根据地图中的障碍物信息创建CheckCollision实例
