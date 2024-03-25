import os

# 添加MuJoCo库的DLL目录到系统路径
os.add_dll_directory("C://Users//10501/.mujoco//mjpro150//bin")

from mujoco_py import (
    load_model_from_path,
    MjSim,
    MjViewer,
)  # 从mujoco_py导入模型加载、仿真和可视化相关的类
from mujoco_py.generated import (
    const,
)  # 从mujoco_py.generated导入const，用于访问MuJoCo的常量


def main():
    # 加载MuJoCo模型文件
    model = load_model_from_path(
        "G://VS_Code_Document//improved_rrt_robot//assets//universal_robots_ur5e//sceneDualarm.xml"
    )
    # 初始化MuJoCo仿真
    sim = MjSim(model)
    # 初始化仿真环境的可视化
    viewer = MjViewer(sim)

    # 无限循环来代替固定步数的循环
    try:
        while True:
            sim.step()
            viewer.render()
    except KeyboardInterrupt:
        # 当按下Ctrl+C时，捕获键盘中断异常，退出程序
        print("仿真结束，退出程序。")


if __name__ == "__main__":
    main()
