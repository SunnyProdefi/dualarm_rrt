import copy  # 导入copy模块，用于复制对象
import math  # 导入math模块，提供数学函数支持
import multiprocessing  # 导入multiprocessing模块，支持进程并发执行
from typing import Union, List, Tuple  # 导入类型注解支持
from functools import partial  # 导入partial，用于固定函数的部分参数

from src.geometry import (
    LineSegment,
    Collision,
)  # 从geometry模块导入LineSegment和Collision类
from src.robot import Robot  # 从robot模块导入Robot类

from .check_collision import (
    CheckCollision,
)  # 从当前目录的check_collision模块导入CheckCollision类


class CheckCollisionRobot(
    CheckCollision
):  # 定义CheckCollisionRobot类，继承自CheckCollision
    count = 10  # 设置静态变量count为10，用于在检测碰撞时定义检查的点数

    def __init__(
        self, obstacles: Union[List, Tuple], expand_dis: float, robots: List[Robot]
    ) -> None:
        super().__init__(obstacles)  # 调用父类构造函数，初始化障碍物列表
        self.__expand_dis = expand_dis  # 初始化扩展距离
        self.__robots = robots  # 修改为接受一个机器人列表

    def check_collision(
        self, line_segment: LineSegment, pool: multiprocessing.Pool = None
    ):
        # 计算应该检查的点数，基于线段长度、静态变量和扩展距离
        count = math.ceil(
            line_segment.get_length() * CheckCollisionRobot.count / self.__expand_dis
        )
        count = max(count, 1)  # 确保至少有一个点被检查

        if pool:  # 如果提供了进程池，则使用并行处理
            tasks = []
            for num in range(count + 1):
                tasks.append(
                    pool.apply_async(
                        self._check_collision_robot_joint, (line_segment, count, num)
                    )
                )

            # 收集并行任务的结果
            for task in tasks:
                if (
                    task.get()
                ):  # 如果任何一个任务返回True（表示发生碰撞），则立即返回True
                    return True
            return False  # 所有任务完成后，如果没有碰撞发生，返回False
        else:  # 没有提供进程池，使用顺序处理
            for num in range(count + 1):
                if self._check_collision_robot_joint(
                    line_segment, count, num
                ):  # 顺序检查每个机器人
                    return True
            return False

    def _check_collision_robot_joint(
        self, line_segment: LineSegment, count: int, num: int
    ):
        q0 = line_segment.get_point0().get_t()  # 获取线段起点的关节配置
        q1 = line_segment.get_point1().get_t()  # 获取线段终点的关节配置

        # 计算当前检查点的关节配置
        q = q0 + (q1 - q0) / count * num

        # 分别为两个机器人臂设置关节状态
        self.__robots[0].set_joint(q[:6])  # 为第一个机器人臂设置前6个DOF的关节状态
        self.__robots[1].set_joint(q[6:])  # 为第二个机器人臂设置后6个DOF的关节状态

        # 获取两个机器人的几何形状
        geometries1 = self.__robots[0].get_geometries()
        geometries2 = self.__robots[1].get_geometries()

        # 检查第一个机器人臂是否与障碍物碰撞
        for obstacle in self._obstacles:
            for geometry in geometries1:
                if Collision.is_collision(obstacle, geometry):
                    return True

        # 检查第二个机器人臂是否与障碍物碰撞
        for obstacle in self._obstacles:
            for geometry in geometries2:
                if Collision.is_collision(obstacle, geometry):
                    return True
        # 检查两个机器人臂之间是否存在碰撞
        for geometry1 in geometries1:
            for geometry2 in geometries2:
                if Collision.is_collision(geometry1, geometry2):
                    return True  # 如果两个机器人臂之间存在碰撞，返回True

        return False  # 如果两个机器人臂都没有与障碍物碰撞，返回False
