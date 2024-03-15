import copy  # 导入copy模块，用于执行深复制
from typing import (
    Union,
    List,
    Tuple,
)  # 从typing模块导入Union, List, Tuple，用于类型注解
import multiprocessing  # 导入multiprocessing模块，支持程序进行多进程处理

from src.geometry import (
    LineSegment,
    Collision,
)  # 从src.geometry模块导入LineSegment和Collision类

from .i_check_collision import ICheckCollision  # 从当前目录导入ICheckCollision接口


# 定义CheckCollision类，实现ICheckCollision接口
class CheckCollision(ICheckCollision):

    def __init__(self, obstacles: Union[List, Tuple]) -> None:
        # 构造函数，初始化时接收一组障碍物
        self._obstacles = copy.deepcopy(
            obstacles
        )  # 深复制障碍物列表或元组，以防原始数据被修改

    def check_collision(
        self, line_segment: LineSegment, pool: multiprocessing.Pool = None
    ):
        # 检查给定线段是否与任何障碍物发生碰撞
        for obstacle in self._obstacles:
            # 遍历所有障碍物
            if Collision.is_collision(line_segment, obstacle):
                # 如果线段与障碍物发生碰撞，则返回True
                return True
        return False  # 如果线段与所有障碍物都未发生碰撞，则返回False
