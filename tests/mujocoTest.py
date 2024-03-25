import time
import mujoco
import mujoco.viewer

def run_simulation(xml_path):
    # 从 XML 文件加载模型
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 设置自动关闭视图器的时间（30秒）
        start = time.time()
        while viewer.is_running() and time.time() - start < 30:
            step_start = time.time()

            # 进行一步物理仿真
            mujoco.mj_step(model, data)

            # 示例：每两秒切换接触点的可视化
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

            # 同步视图器状态
            viewer.sync()

            # 控制仿真步进与实际时间的同步
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    # 请替换以下路径为您的 MJCF XML 文件的实际路径
    xml_path = "D:/VS_Code_Document/dualarm_rrt/assets/universal_robots_ur5e/scene.xml"
    run_simulation(xml_path)

