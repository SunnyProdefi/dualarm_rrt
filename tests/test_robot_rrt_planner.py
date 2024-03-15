import os  # 导入os模块，用于操作系统功能，如添加DLL目录

import numpy as np  # 导入numpy库，用于进行科学计算
from spatialmath import SE3  # 从spatialmath库导入SE3，用于表示三维空间的刚体变换
import sys  # 导入sys模块，用于访问与Python解释器紧密相关的变量和函数

sys.path.append(
    "G://VS_Code_Document//improved_rrt_robot"
)  # 将自定义模块的路径添加到系统路径

from src.robot import Robot  # 从src目录的robot模块导入Robot类
from src.geometry import Brick  # 从src目录的geometry模块导入Brick类
from src.motion_planning import (
    RobotRRTParameter,
    RRTMap,
    BlendPlanner,
    RRTPlanner,
    RRTStarPlanner,
    InformedRRTStarPlanner,
)  # 从src目录的motion_planning模块导入多个路径规划相关的类

os.add_dll_directory(
    "C://Users//10501/.mujoco//mjpro150//bin"
)  # 添加MuJoCo库的DLL目录到系统路径

from mujoco_py import (
    load_model_from_path,
    MjSim,
    MjViewer,
)  # 从mujoco_py导入模型加载、仿真和可视化相关的类
from mujoco_py.generated import (
    const,
)  # 从mujoco_py.generated导入const，用于访问MuJoCo的常量
import multiprocessing  # 导入multiprocessing模块，用于支持并发执行

# 实例化Robot类，沿Y轴负方向偏移0.5米
robot1 = Robot(offset_position=(0, -0.5, 0))
# 实例化Robot类，沿Y轴正方向偏移0.5米
robot2 = Robot(offset_position=(0, 0.5, 0))

obstacles = [
    Brick(
        SE3.Trans(0.5, 0.0, 0.8), np.array([0.2, 0.8, 0.01])
    ),  # 创建一个障碍物，是一个位于特定位置的砖块
]

rrt_map = RRTMap(
    area=[
        (-np.pi / 2, np.pi / 2),
        (-np.pi / 2, np.pi / 2),
        (-np.pi, np.pi),
        (-np.pi, np.pi),
        (-np.pi, np.pi),
        (-np.pi / 2, np.pi / 2),
        (-np.pi / 2, np.pi / 2),
        (-np.pi / 2, np.pi / 2),
        (-np.pi, np.pi),
        (-np.pi, np.pi),
        (-np.pi, np.pi),
        (-np.pi / 2, np.pi / 2),
    ],
    obstacles=obstacles,
)  # 创建一个RRTMap对象，定义了机器人操作空间和障碍物

# 修改RobotRRTParameter对象的实例化，传入机器人列表
rrt_parameter = RobotRRTParameter(
    start=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    goal=[
        0.0,
        0.0,
        np.pi / 2,
        0.0,
        -np.pi / 2,
        0.0,
        0.0,
        0.0,
        np.pi / 2,
        0.0,
        -np.pi / 2,
        0.0,
    ],
    robots=[robot1, robot2],  # 使用列表传入多个机器人
    expand_dis=np.pi / 12,
    max_iter=1000,
    radius=5.0,
)
# 创建一个RobotRRTParameter对象，定义了起始点、目标点、机器人、扩展距离、最大迭代次数和搜索半径


def visualize(rrt_planner):
    # 从路径规划器获取路径参数
    path_parameters = rrt_planner.get_path_parameters()
    # 为路径参数中的每一段初始化一个半径列表，用于路径平滑
    radii = [0.0 for _ in range(len(path_parameters) - 1)]
    # 创建路径平滑规划器
    blend_planner = BlendPlanner(path_parameters, radii)

    # 如果路径规划失败，则直接返回
    if not rrt_planner.success:
        return

    # 加载模型
    model = load_model_from_path(
        "G:\\VS_Code_Document\\improved_rrt_robot\\assets\\universal_robots_ur5e\\sceneDualarm.xml"
    )

    sim = MjSim(model)  # 初始化仿真环境
    viewer = MjViewer(sim)  # 初始化仿真环境的可视化
    dof = 12  # 设置机器人的自由度
    num = 1001  # 设置用于插值的样本数
    ss = np.linspace(0.0, 1.0, num)  # 生成等间隔的样本点
    joints = np.zeros((num, dof))  # 初始化用于存储关节位置的矩阵
    s_step = 0  # 初始化步进变量
    forward = True  # 初始化方向标志
    j = 0  # 初始化计数器

    # 遍历所有样本点，进行插值和设置关节位置
    for i, si in enumerate(ss):
        qi = blend_planner.interpolate(si)  # 插值获取当前样本点的关节位置

        # 假设robot1和robot2是两个Robot对象，分别设置关节位置
        robot1.set_joint(qi[:6])  # 设置第一个机器人的关节位置
        robot2.set_joint(qi[6:])  # 设置第二个机器人的关节位置

        joints[i, :6] = robot1.get_joint()  # 获取第一个机器人的关节位置
        joints[i, 6:] = robot2.get_joint()  # 获取第二个机器人的关节位置

    # 初始化循环变量和方向标志
    s_step = 0
    forward = True
    j = 0

    # 为两个机器人设置初始和终端关节位置，并检索它们的笛卡尔坐标
    robot1.set_joint(joints[0, :6])
    T0 = robot1.get_cartesian()  # robot1的初始位置
    robot1.set_joint(joints[-1, :6])
    T1 = robot1.get_cartesian()  # robot1的终端位置

    robot2.set_joint(joints[0, 6:])
    T2 = robot2.get_cartesian()  # robot2的初始位置
    robot2.set_joint(joints[-1, 6:])
    T3 = robot2.get_cartesian()  # robot2的终端位置

    # 收集笛卡尔坐标以供可视化
    coms = [T0.t, T1.t, T2.t, T3.t]

    # 主循环，用于更新仿真环境并进行可视化
    while True:
        # 根据当前步骤的关节位置更新仿真环境
        for i in range(6):  # 在仿真中更新第一个机器人的关节
            sim.data.qpos[i] = joints[s_step, i]
        for i in range(6, 12):  # 在仿真中更新第二个机器人的关节
            sim.data.qpos[i] = joints[s_step, i]
        sim.data.qvel[:] = 0.0  # 假设速度为零以简化

        sim.step()  # 步进仿真
        viewer.render()  # 渲染视图

        # 在初始和终止位置添加标记
        for com in coms:
            viewer.add_marker(
                pos=com,
                size=np.array([0.01, 0.01, 0.01]),
                rgba=np.array([1.0, 0, 0, 1]),
                type=const.GEOM_SPHERE,
            )

        # 更新计数器和步进变量
        j += 1
        if j == 10:
            j = 0
            if forward:
                s_step += 1
                if s_step == num - 1:
                    forward = False
            else:
                s_step -= 1
                if s_step == 0:
                    forward = True


# 定义测试RRT算法的函数
def test_robot_rrt():
    # 初始化RRT路径规划器
    rrt_planner = RRTPlanner(rrt_map, rrt_parameter)
    # 调用可视化函数
    visualize(rrt_planner)


# 定义测试RRT*算法的函数
def test_robot_rrt_star():
    # 初始化RRT*路径规划器
    rrt_planner = RRTStarPlanner(rrt_map, rrt_parameter, pool)
    # 调用可视化函数
    visualize(rrt_planner)


# 定义测试Informed RRT*算法的函数
def test_robot_informed_rrt_star():
    # 初始化Informed RRT*路径规划器
    rrt_planner = InformedRRTStarPlanner(rrt_map, rrt_parameter, pool)
    # 调用可视化函数
    visualize(rrt_planner)


# 如果这个脚本是作为主程序运行
if __name__ == "__main__":
    pool = multiprocessing.Pool()  # 创建进程池以支持并发执行
    for i in range(100):
        test_robot_rrt()  # 测试标准RRT算法
        # test_robot_rrt_star()  # 测试RRT*算法
        # test_robot_informed_rrt_star()  # 测试Informed RRT*算法

    pool.close()  # 关闭进程池
