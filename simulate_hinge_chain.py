#!/usr/bin/env python3
"""
多体仿真测试题：铰链链式系统 (Hinge Chain)
使用MuJoCo模拟多个Capsule形状的刚体通过HingeJoint连接的链式系统
"""

import mujoco
import mujoco.viewer
import numpy as np

# Configuration constants
INITIAL_ANGLE_RADIANS = 0.9  # Initial disturbance angle for first joint (≈51.4°)


def main():
    """
    主函数：加载模型并运行仿真
    """
    # 加载MuJoCo模型
    model_path = "hinge_chain.xml"
    print(f"正在加载模型: {model_path}")

    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"加载模型失败: {e}")
        return

    print(f"模型加载成功!")
    print(f"Capsule刚体数量: {model.nbody - 2}")  # 减去world body和anchor
    print(f"HingeJoint数量: {model.njnt}")
    print(f"自由度: {model.nv}")
    print(f"重力加速度: {model.opt.gravity}")
    print(f"时间步长: {model.opt.timestep}s")

    # 设置初始状态（给系统一些初始扰动）
    _initialize_joint_positions(model, data)

    # 前向动力学计算初始状态
    mujoco.mj_forward(model, data)

    print("\n开始仿真...")
    print("按 ESC 或关闭窗口退出")
    print("按 Tab 键 切换联系力显示")
    print("使用鼠标拖拽旋转视角，滚轮缩放\n")

    # 创建可视化窗口并运行仿真
    # 注意：在macOS上使用launch而不是launch_passive
    mujoco.viewer.launch(model, data)

    print("仿真结束")


def _initialize_joint_positions(model, data):
    """
    初始化关节位置以给系统一个初始扰动
    
    Args:
        model: MuJoCo模型对象
        data: MuJoCo数据对象
    """
    if model.nq == 0:
        return
    
    data.qpos[0] = INITIAL_ANGLE_RADIANS  # ≈28.6° (0.5 rad)

def print_model_info(model):
    """
    打印模型详细信息
    """
    print("\n=== 模型详细信息 ===")
    print(f"模型名称: {model.names().decode()}")
    print(f"刚体数量: {model.nbody}")
    print(f"关节数量: {model.njnt}")
    print(f"几何体数量: {model.ngeom}")
    print(f"位置自由度: {model.nq}")
    print(f"速度自由度: {model.nv}")
    print(f"时间步长: {model.opt.timestep}s")
    print(f"积分器: {model.opt.integrator}")

    print("\n=== 刚体信息 ===")
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        print(f"Body {i}: {body_name}")

    print("\n=== 关节信息 ===")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_type = model.jnt_type[i]
        joint_axis = model.jnt_axis[i]
        print(f"Joint {i}: {joint_name}, Type: {joint_type}, Axis: {joint_axis}")


if __name__ == "__main__":
    main()
