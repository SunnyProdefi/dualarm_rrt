import copy  # 导入copy模块，用于深复制对象
from typing import Union, Iterable  # 从typing模块导入Union和Iterable，用于类型注解

import numpy as np  # 导入numpy库，用于科学计算

from .rrt_map import RRTMap  # 从当前目录导入RRTMap类，表示RRT算法的地图
from .i_check_collision import ICheckCollision  # 从当前目录导入ICheckCollision接口，定义了碰撞检测方法的接口

from .rrt_parameter import RRTParameter  # 从当前目录导入RRTParameter类，存储RRT算法的参数
from .check_collision_robot import CheckCollisionRobot  # 从当前目录导入CheckCollisionRobot类，用于机器人的碰撞检测
from src.robot import Robot  # 从src.robot模块导入Robot类

# 修改RobotRRTParameter类，以支持接收多个机器人
class RobotRRTParameter(RRTParameter):
    def __init__(self, start: Union[np.ndarray, Iterable], goal: Union[np.ndarray, Iterable], robots: Iterable[Robot],
                 expand_dis: float = 1.0, goal_sample_rate: float = 10.0, max_iter: int = 100, radius: float = 10.0,
                 animation: bool = False) -> None:
        super().__init__(start, goal, expand_dis, goal_sample_rate, max_iter, radius, animation)
        self.__robots = robots  # 修改为接收一个机器人列表

    @property
    def robots(self):
        return self.__robots

    def create_check_collision(self, rrt_map: RRTMap) -> ICheckCollision:
        # 假设CheckCollisionRobot可以接受机器人列表
        return CheckCollisionRobot(rrt_map.obstacles, self.expand_dis, self.robots)
