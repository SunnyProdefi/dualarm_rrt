import random  # 导入random模块，用于生成随机数
from typing import List  # 从typing模块导入List，用于类型注解，指定列表中元素的类型
import multiprocessing  # 导入multiprocessing模块，支持程序进行多进程处理

import numpy as np  # 导入numpy库，一个用于科学计算的库
import matplotlib.pyplot as plt  # 从matplotlib库导入pyplot模块，用于数据可视化
from mpl_toolkits.mplot3d import (
    Axes3D,
)  # 从mpl_toolkits.mplot3d导入Axes3D，用于创建3D图形

# 从src.geometry模块导入UnitVector, LineSegment, Distance, Collision类
# 这些类可能用于表示单位向量、线段、距离计算和碰撞检测等
from src.geometry import UnitVector, LineSegment, Distance, Collision

# 从当前目录的node模块导入Node类，可能表示RRT算法中的节点
from .node import Node

# 从当前目录的rrt_map模块导入RRTMap类，可能用于定义RRT算法的地图环境和障碍物
from .rrt_map import RRTMap

# 从当前目录的rrt_parameter模块导入RRTParameter类，可能用于设置RRT算法的参数
from .rrt_parameter import RRTParameter

# 从上一级目录的path_parameter模块导入PathParameter类，可能用于描述路径的参数
from ..path_parameter import PathParameter

# 从上一级目录的joint_planning模块导入JointParameter类，可能用于描述关节的参数
from ..joint_planning import JointParameter


class RRTPlanner:
    def __init__(
        self,
        rrt_map: RRTMap,
        rrt_parameter: RRTParameter,
        pool: multiprocessing.Pool = None,
    ) -> None:
        # 初始化RRT规划器，接收RRT地图、参数和可选的进程池
        self._area = rrt_map.area  # 地图区域
        self._obstacles = rrt_map.obstacles  # 地图障碍物
        self.start = Node(rrt_parameter.start)  # 起始节点
        self.goal = Node(rrt_parameter.goal)  # 目标节点
        self._expand_dis = rrt_parameter.expand_dis  # 扩展距离
        self.goal_sample_rate = rrt_parameter.goal_sample_rate  # 目标采样率
        self.max_iter = rrt_parameter.max_iter  # 最大迭代次数
        self.animation = rrt_parameter.animation  # 是否动画显示
        self.nodes: List[Node] = []  # 初始化节点列表
        self._path = []  # 初始化路径
        self._path_length = float("inf")  # 初始化路径长度为无穷大
        self._check_collision = rrt_parameter.create_check_collision(
            rrt_map
        )  # 创建碰撞检测函数
        self.plan(pool)  # 开始规划路径

    def plan(self, pool: multiprocessing.Pool = None) -> None:
        # 路径规划函数
        self.nodes = [self.start]  # 将起始节点加入节点列表

        for i in range(self.max_iter):  # 循环直至最大迭代次数
            rnd = self.sample()  # 采样一个随机节点
            n_ind = self.get_nearest_list_index(rnd)  # 获取最近节点索引

            nearest_node = self.nodes[n_ind]  # 获取最近的节点
            new_node = self.get_new_node(n_ind, rnd)  # 生成新节点

            if self._check_collision.check_collision(
                LineSegment(nearest_node.get_point(), new_node.get_point()), pool
            ):
                continue  # 如果新生成的线段与障碍物碰撞，则跳过当前迭代

            self.add_node(new_node, pool)  # 添加新节点到节点列表

            if self.animation:
                self.draw_graph(new_node, self._path)  # 如果开启动画，则绘制图形

            if not self.is_near_goal(pool):
                continue  # 如果新节点不靠近目标，则继续下一次迭代

            if self._check_collision.check_collision(
                LineSegment(new_node.get_point(), self.goal.get_point()), pool
            ):
                continue  # 如果新节点到目标节点的线段与障碍物碰撞，则跳过

            if self.get_path_and_length():
                break  # 如果找到路径，则结束循环

        if self.animation:
            self.draw_graph(
                self.goal, self._path, True
            )  # 如果开启动画，最后绘制完整路径图
        return

    def sample(self) -> Node:
        # 随机采样节点函数
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = [
                random.uniform(*self._area[i]) for i in range(len(self._area))
            ]  # 以一定概率随机采样点
        else:
            rnd = self.goal.get_point()  # 以一定概率直接采样目标点
        return Node(rnd)  # 返回采样的节点

    def get_nearest_list_index(self, rnd: Node) -> int:
        # 获取最近节点的索引
        d_list = [
            Distance.point_to_point(node.point, rnd.point) for node in self.nodes
        ]  # 计算所有节点到随机点的距离
        min_index = d_list.index(min(d_list))  # 找到最小距离的索引
        return min_index  # 返回最近节点的索引

    def get_new_node(self, n_ind: int, rnd: Node) -> Node:
        # 生成新节点函数
        nearest_node = self.nodes[n_ind]  # 获取最近节点
        unit_vector = UnitVector(
            nearest_node.get_point(), rnd.get_point()
        )  # 计算单位向量
        new_point = (
            nearest_node.get_point() + self._expand_dis * unit_vector
        )  # 沿单位向量扩展一定距离
        new_node = Node(
            new_point, cost=nearest_node.cost + self._expand_dis, parent=n_ind
        )  # 创建新节点
        return new_node  # 返回新节点

    def is_near_goal(self, pool: multiprocessing.Pool = None) -> bool:
        # 检查是否靠近目标
        d = Distance.point_to_point(
            self.nodes[-1].get_point(), self.goal.get_point()
        )  # 计算最新节点到目标的距离
        if d < self._expand_dis:
            self.set_goal(pool)  # 如果小于扩展距离，则认为达到目标
            return True
        return False  # 否则，没有达到目标

    def get_final_course(self) -> List[Node]:
        # 获取最终路径
        path = [self.goal]  # 从目标节点开始
        node = self.goal
        while node.parent != -1:  # 逆向遍历父节点，直到起点
            node = self.nodes[node.parent]  # 获取父节点
            path.append(node)  # 添加到路径列表
        return path  # 返回路径

    def get_path_length(self) -> float:
        # 返回到达目标的总成本（路径长度）
        return self.goal.get_cost()

    @property
    def success(self) -> bool:
        # 如果路径长度小于无穷大，表示找到了一条路径，规划成功
        return self._path_length < float("inf")

    def draw_graph(self, rnd: Node, path: List, show=False) -> None:
        # 绘制RRT搜索树和路径
        plt.clf()  # 清除当前图形
        ax = plt.axes(projection="3d")  # 创建3D坐标轴
        if rnd is not None:
            # 绘制随机点
            rnd.get_point().plot(ax)

        for node in self.nodes:
            if node.parent != -1:
                # 绘制节点到其父节点的线段
                LineSegment(self.nodes[node.parent].get_point(), node.get_point()).plot(
                    ax
                )

        for obstacle in self._obstacles:
            # 绘制障碍物
            obstacle.plot(ax)

        # 绘制起点和终点
        self.start.get_point().plot(ax, "b")
        self.goal.get_point().plot(ax, "b")

        if len(path) > 0:
            for node in path:
                if node.parent != -1:
                    # 绘制路径上的点
                    LineSegment(
                        self.nodes[node.parent].get_point(), node.get_point()
                    ).plot(ax, "r")

        self.draw_others(ax)  # 绘制其他可能的图形元素

        # 设置坐标轴范围
        ax.set_xlim(*self._area[0])
        ax.set_ylim(*self._area[1])
        ax.set_zlim(*self._area[2])
        ax.grid(True)  # 显示网格

        if show:
            plt.show()  # 显示图形
        else:
            plt.pause(0.01)  # 短暂暂停，用于动态更新图形

    def draw_others(self, ax) -> None:
        # 该方法预留给子类实现，用于绘制其他元素，这里为空实现
        pass

    def in_area(self, p: np.ndarray) -> bool:
        # 判断点是否在规划区域内
        for i in range(p.size):
            if (self._area[i][0] > p[i]) or (self._area[i][1] < p[i]):
                # 如果点的任意维度超出区域范围，则返回False
                return False
        return True  # 所有维度都在区域内，返回True

    def add_node(self, node: Node, pool: multiprocessing.Pool = None) -> None:
        # 向节点列表中添加新节点
        self.nodes.append(node)

    def get_path_and_length(self) -> bool:
        # 获取最终路径及其长度
        self._path = self.get_final_course()  # 获取从起点到终点的路径
        self._path_length = self.get_path_length()  # 计算路径长度
        return True  # 返回True，表示操作成功

    def set_goal(self, pool: multiprocessing.Pool = None) -> None:
        # 设置目标节点的成本和父节点
        d = Distance.point_to_point(self.nodes[-1].get_point(), self.goal.get_point())
        self.goal.set_cost(d + self.nodes[-1].get_cost())  # 设置目标节点成本
        self.goal.set_parent(len(self.nodes) - 1)  # 设置目标节点的父节点为最后一个节点

    def get_path_parameters(self) -> List[PathParameter]:
        # 获取路径参数，用于进一步处理或分析
        path_parameters = []
        points = self._path[::-1]  # 将路径逆序，从起点开始
        for i, point in enumerate(points[:-1]):
            # 为路径上的每一段创建一个路径参数对象
            path_parameters.append(JointParameter(point.get_t(), points[i + 1].get_t()))
        return path_parameters  # 返回路径参数列表
