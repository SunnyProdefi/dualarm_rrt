import mujoco
import mujoco.viewer
import numpy as np

def run_simulation(xml_path):
    # 从 XML 文件加载模型
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    # 设置关节的目标位置
    # 这里我们假设所有关节的目标位置都是 0
    target_positions = np.ones(model.nu)  # model.nu 是模型中执行器的数量

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # 设置执行器的目标位置
            data.ctrl[:] = target_positions

            # 进行一步物理仿真
            mujoco.mj_step(model, data)

            # 同步视图器状态
            viewer.sync()

if __name__ == "__main__":
    xml_path = "D:/VS_Code_Document/dualarm_rrt/assets/spaceRobot/scene.xml"
    run_simulation(xml_path)


