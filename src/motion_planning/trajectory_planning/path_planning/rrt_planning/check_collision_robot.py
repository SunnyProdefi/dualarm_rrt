import copy  # 导入copy模块，用于复制对象
import math  # 导入math模块，提供数学函数支持
import multiprocessing  # 导入multiprocessing模块，支持进程并发执行
from typing import Union, List, Tuple  # 导入类型注解支持
from functools import partial  # 导入partial，用于固定函数的部分参数

from src.geometry import LineSegment, Collision  # 从geometry模块导入LineSegment和Collision类
from src.robot import Robot  # 从robot模块导入Robot类

from .check_collision import CheckCollision  # 从当前目录的check_collision模块导入CheckCollision类


class CheckCollisionRobot(CheckCollision):  # 定义CheckCollisionRobot类，继承自CheckCollision
    count = 10  # 设置静态变量count为10，用于在检测碰撞时定义检查的点数

    def __init__(self, obstacles: Union[List, Tuple], expand_dis: float, robot: Robot) -> None:
        super().__init__(obstacles)  # 调用父类构造函数，初始化障碍物列表
        self.__expand_dis = expand_dis  # 初始化扩展距离
        self.__robot = copy.deepcopy(robot)  # 深拷贝robot对象，以避免原始对象被修改

    def check_collision(self, line_segment: LineSegment, pool: multiprocessing.Pool = None):
        # 检测给定线段上的机器人是否与障碍物碰撞
        count = math.ceil(line_segment.get_length() * CheckCollisionRobot.count / self.__expand_dis)
        # 根据线段长度和扩展距离计算需要检查的点数
        if count == 0:
            count += 1  # 如果计算的点数为0，则至少检查一点

        if pool:
            func = partial(self._check_collision_robot_joint, line_segment, count)
            # 使用partial创建一个新函数，固定line_segment和count参数
            results = pool.imap_unordered(func, range(count + 1))
            # 使用进程池并行计算每个检查点的碰撞情况
            for result in results:
                if result:
                    return True  # 如果有任何检查点发生碰撞，则返回True
            return False  # 所有检查点都未发生碰撞，返回False

        for i in range(count + 1):
            if self._check_collision_robot_joint(line_segment, count, i):
                return True  # 顺序检查每个点，如果有碰撞则返回True
        return False  # 所有检查点都未发生碰撞，返回False

    def _check_collision_robot_joint(self, line_segment: LineSegment, count: int, num: int):
        # 检查特定点上的机器人是否与障碍物发生碰撞
        q0 = line_segment.get_point0().get_t()  # 获取线段起点的位置
        q1 = line_segment.get_point1().get_t()  # 获取线段终点的位置

        q = q0 + (q1 - q0) / count * num  # 根据检查点序号计算当前点的位置
        self.__robot.set_joint(q)  # 设置机器人关节的位置
        geometries = self.__robot.get_geometries()  # 获取机器人的几何形状
        for obstacles in self._obstacles:
            for geometry in geometries:
                if Collision.is_collision(obstacles, geometry):
                    return True  # 如果机器人的任何几何形状与障碍物发生碰撞，则返回True
        return False  # 机器人在当前点未与任何障碍物碰撞，返回False

