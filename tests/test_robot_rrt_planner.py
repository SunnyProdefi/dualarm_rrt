import os  # 导入os模块，用于操作系统功能，如添加DLL目录

import numpy as np  # 导入numpy库，用于进行科学计算
from spatialmath import SE3  # 从spatialmath库导入SE3，用于表示三维空间的刚体变换
import sys  # 导入sys模块，用于访问与Python解释器紧密相关的变量和函数
sys.path.append("G://VS_Code_Document//improved_rrt_robot")  # 将自定义模块的路径添加到系统路径

from src.robot import Robot  # 从src目录的robot模块导入Robot类
from src.geometry import Brick  # 从src目录的geometry模块导入Brick类
from src.motion_planning import RobotRRTParameter, RRTMap, BlendPlanner, RRTPlanner, RRTStarPlanner, \
    InformedRRTStarPlanner  # 从src目录的motion_planning模块导入多个路径规划相关的类

os.add_dll_directory("C://Users//10501/.mujoco//mjpro150//bin")  # 添加MuJoCo库的DLL目录到系统路径

from mujoco_py import load_model_from_path, MjSim, MjViewer  # 从mujoco_py导入模型加载、仿真和可视化相关的类
from mujoco_py.generated import const  # 从mujoco_py.generated导入const，用于访问MuJoCo的常量
import multiprocessing  # 导入multiprocessing模块，用于支持并发执行

robot = Robot()  # 实例化Robot类，创建一个机器人对象

obstacles = [
    Brick(SE3.Trans(0.5, 0.0, 0.8), np.array([0.4, 0.4, 0.01])),  # 创建一个障碍物，是一个位于特定位置的砖块
]

rrt_map = RRTMap(
    area=[
        (-np.pi / 2, np.pi / 2),
        (-np.pi / 2, np.pi / 2),
        (-np.pi, np.pi),
        (-np.pi, np.pi),
        (-np.pi, np.pi),
        (-np.pi / 2, np.pi / 2)
    ],
    obstacles=obstacles)  # 创建一个RRTMap对象，定义了机器人操作空间和障碍物

rrt_parameter = RobotRRTParameter(start=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                  goal=[0.0, 0.0, np.pi / 2, 0.0, -np.pi / 2, 0.0],
                                  robot=robot, expand_dis=np.pi / 12, max_iter=500, radius=5.0)
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
    model = load_model_from_path("G:\\VS_Code_Document\\improved_rrt_robot\\assets\\universal_robots_ur5e\\scene.xml")
    # 初始化仿真环境
    sim = MjSim(model)
    # 初始化仿真环境的可视化
    viewer = MjViewer(sim)

    # 设置机器人的自由度
    dof = 6

    # 设置用于插值的样本数
    num = 1001
    # 生成等间隔的样本点
    ss = np.linspace(0.0, 1.0, num)
    # 初始化用于存储关节位置的矩阵
    joints = np.zeros((num, dof))

    # 初始化步进变量
    s_step = 0
    # 初始化方向标志
    forward = True
    # 初始化计数器
    j = 0

    # 遍历所有样本点，进行插值
    for i, si in enumerate(ss):
        qi = blend_planner.interpolate(si)
        robot.set_joint(qi)
        joints[i, :] = robot.get_joint()

    # 设置机器人的初始和终止关节位置
    robot.set_joint(joints[0, :])
    T0 = robot.get_cartesian()
    robot.set_joint(joints[-1, :])
    T1 = robot.get_cartesian()
    # 获取初始和终止位置的笛卡尔坐标
    coms = [T0.t, T1.t]

    # 主循环，用于更新仿真环境并进行可视化
    while True:
        # 设置仿真环境中的关节位置和速度
        for i in range(dof):
            sim.data.qpos[i] = joints[s_step, i]
            sim.data.qvel[i] = 0.0

        # 步进仿真环境
        sim.step()
        # 渲染视图
        viewer.render()

        # 在初始和终止位置添加标记
        for com in coms:
            viewer.add_marker(pos=com, size=np.array([0.01, 0.01, 0.01]), rgba=np.array([1., 0, 0, 1]),
                              type=const.GEOM_SPHERE)

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

    for i in range(1):
        test_robot_rrt()  # 测试标准RRT算法
        # test_robot_rrt_star()  # 测试RRT*算法（被注释掉了）
        # test_robot_informed_rrt_star()  # 测试Informed RRT*算法（被注释掉了）

    pool.close()  # 关闭进程池
