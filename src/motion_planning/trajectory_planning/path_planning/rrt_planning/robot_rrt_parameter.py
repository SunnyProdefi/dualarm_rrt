import copy  # 导入copy模块，用于深复制对象
from typing import Union, Iterable  # 从typing模块导入Union和Iterable，用于类型注解

import numpy as np  # 导入numpy库，用于科学计算

from .rrt_map import RRTMap  # 从当前目录导入RRTMap类，表示RRT算法的地图
from .i_check_collision import ICheckCollision  # 从当前目录导入ICheckCollision接口，定义了碰撞检测方法的接口

from .rrt_parameter import RRTParameter  # 从当前目录导入RRTParameter类，存储RRT算法的参数
from .check_collision_robot import CheckCollisionRobot  # 从当前目录导入CheckCollisionRobot类，用于机器人的碰撞检测
from src.robot import Robot  # 从src.robot模块导入Robot类

# 定义RobotRRTParameter类，继承自RRTParameter类
class RobotRRTParameter(RRTParameter):
    def __init__(self, start: Union[np.ndarray, Iterable], goal: Union[np.ndarray, Iterable], robot: Robot,
                 expand_dis: float = 1.0, goal_sample_rate: float = 10.0, max_iter: int = 100, radius: float = 10.0,
                 animation: bool = False) -> None:
        # 初始化函数，接收机器人起始位置、目标位置、机器人对象及RRT算法参数
        super().__init__(start, goal, expand_dis, goal_sample_rate, max_iter, radius, animation)
        # 调用父类的初始化方法，并传入RRT算法的参数
        self.__robot = robot # 深复制传入的机器人对象，避免直接修改原始对象

    @property
    def robot(self):
        # 机器人属性的getter方法
        return self.__robot  # 返回机器人对象的深复制

    def create_check_collision(self, rrt_map: RRTMap) -> ICheckCollision:
        # 重写create_check_collision方法，用于创建针对机器人的碰撞检测对象
        return CheckCollisionRobot(rrt_map.obstacles, self.expand_dis, self.robot)
        # 返回一个CheckCollisionRobot对象，传入障碍物列表、扩展距离和机器人对象

