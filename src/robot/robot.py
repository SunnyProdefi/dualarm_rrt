import copy  # 导入copy模块，用于对象的深复制
from typing import List  # 从typing模块导入List，用于类型注解

import numpy as np  # 导入numpy库，用于科学计算
import roboticstoolbox as rtb  # 导入roboticstoolbox库，用于机器人模型和运动学计算
from spatialmath import SE3  # 导入spatialmath库的SE3，用于三维空间的刚体变换

from src.geometry import Geometry, Capsule  # 从src.geometry模块导入Geometry和Capsule类

# 定义Robot类
class Robot:
    def __init__(self):
        # 定义UR5机器人的DH参数
        d1 = 0.163
        d4 = 0.134
        d5 = 0.1
        d6 = 0.1

        a3 = 0.425
        a4 = 0.392

        self.dof = 6  # 机器人的自由度
        self.q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 初始关节角度

        # 定义DH参数
        alpha_array = [0.0, -np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2]
        a_array = [0.0, 0.0, a3, a4, 0.0, 0.0]
        d_array = [d1, 0.0, 0.0, d4, d5, d6]
        theta_array = [0.0, -np.pi / 2, 0.0, np.pi / 2, 0.0, 0.0]
        tool = SE3.Trans(0.0, 0.0, 0.2)  # 工具的变换矩阵

        # 创建机器人模型
        links = []
        for i in range(6):
            links.append(rtb.DHLink(d=d_array[i], alpha=alpha_array[i], a=a_array[i], offset=theta_array[i], mdh=True))
        self.robot = rtb.DHRobot(links)

        # 存储DH参数
        self.alpha_array = alpha_array
        self.a_array = a_array
        self.d_array = d_array
        self.theta_array = theta_array

    def fkine(self, q) -> SE3:
        # 计算给定关节角度的正运动学
        return self.robot.fkine(q)

    def ikine(self, Tep):
        # 计算给定末端位姿的逆运动学
        result = self.robot.ikine_NR(Tep, q0=self.q0)
        if result.success:
            return result.q
        return []

    def move_cartesian(self, T: SE3):
        # 移动到给定的笛卡尔位姿
        q = self.ikine(T)

        assert len(q)  # 若逆运动学失败，则抛出异常
        self.set_joint(q)

    def set_joint(self, q):
        # 设置当前关节角度
        self.q0 = q[:]

    def get_joint(self):
        # 获取当前关节角度
        return copy.deepcopy(self.q0)

    def get_cartesian(self):
        # 获取当前末端执行器的位姿
        return self.fkine(self.q0)

    def get_geometries(self) -> List[Geometry]:
        # 初始化变换矩阵列表
        Ts = []
        # 初始化为单位矩阵
        T = SE3()
        # 遍历所有关节
        for i in range(self.dof):
            # 根据MDH参数计算当前关节的变换矩阵，并累乘到总变换矩阵
            T = T * Robot.transform_mdh(self.alpha_array[i], self.a_array[i], self.d_array[i], self.theta_array[i], self.q0[i])
            # 将计算得到的变换矩阵添加到列表中
            Ts.append(T)

        # 计算每个部分的几何形状并初始化
        T1 = Ts[0] * SE3.Trans(0, 0, -0.04)
        geometry1 = Capsule(T1, 0.06, 0.12)

        T2 = Ts[1] * SE3.Trans(0.2, 0, 0.138) * SE3.Ry(-np.pi / 2)
        geometry2 = Capsule(T2, 0.05, 0.4)

        T3 = Ts[2] * SE3.Trans(0.2, 0, 0.007) * SE3.Ry(-np.pi / 2)
        geometry3 = Capsule(T3, 0.038, 0.38)

        T4 = Ts[3] * SE3.Trans(0.0, 0.0, -0.07)
        geometry4 = Capsule(T4, 0.04, 0.14)

        T5 = Ts[4] * SE3.Trans(0.0, 0.0, -0.06)
        geometry5 = Capsule(T5, 0.04, 0.12)

        T6 = Ts[5] * SE3.Trans(0.0, 0.0, -0.06)
        geometry6 = Capsule(T6, 0.04, 0.12)

        # 返回包含所有几何形状的列表
        return [geometry1, geometry2, geometry3, geometry4, geometry5, geometry6]

    def __getstate__(self):
        # 获取对象状态，用于序列化
        state = {"dof": self.dof,  # 自由度
                 "q0": self.q0,  # 当前关节角度
                 "alpha_array": self.alpha_array,  # α数组
                 "a_array": self.a_array,  # a数组
                 "d_array": self.d_array,  # d数组
                 "theta_array": self.theta_array,  # θ数组
                 # "robot": self.robot  # 机器人模型，此处被注释掉，因为模型可能不易于直接序列化
                 }
        return state

    def __setstate__(self, state):
        # 设置对象状态，用于反序列化
        self.dof = state["dof"]
        self.q0 = state["q0"]
        self.alpha_array = state["alpha_array"]
        self.a_array = state["a_array"]
        self.d_array = state["d_array"]
        self.theta_array = state["theta_array"]
        # 根据状态中的DH参数重新构建机器人模型
        links = []
        for i in range(6):
            links.append(
                rtb.DHLink(d=self.d_array[i], alpha=self.alpha_array[i], a=self.a_array[i], offset=self.theta_array[i], mdh=True))
        self.robot = rtb.DHRobot(links)

    @staticmethod
    def transform_mdh(alpha, a, d, theta, q) -> SE3:
        # 根据MDH参数进行变换，返回SE3类型的变换矩阵
        return SE3.Rx(alpha) * SE3.Trans(a, 0, d) * SE3.Rz(theta + q)

# 测试代码
if __name__ == '__main__':
    ur_robot = Robot()  # 创建机器人实例
    q0 = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]  # 定义测试用的关节角度
    T1 = ur_robot.fkine(q0)  # 计算正运动学
    print(T1)  # 打印末端执行器的位姿
    ur_robot.move_cartesian(T1)  # 尝试移动到该位姿
    q_new = ur_robot.get_joint()  # 获取新的关节角度
    print(q_new)  # 打印新的关节角度
