"""
纯 Python 逆运动学求解器库


主要特性：
- ET/ETS 类：基本变换表示和运动学链
- 多种求解算法：Newton-Raphson, Gauss-Newton, Levenberg-Marquardt, QP
- 支持冗余机械臂（>6 DoF）
- 关节限位避免和可操作度优化
- 清晰的 API 和详细的文档
"""

# ET 和 ETS 类
from .et import ET, ETS, SE3_to_xyzrpy, xyzrpy_to_SE3

# DH 参数建模
from .DH import DHRobot, create_dh_robot, create_puma560, create_stanford_arm, create_so101_5dof

# IK 基类和解
from .base import IKSolution, IKSolver

# IK 求解器
from .solvers import IK_NR, IK_GN, IK_LM, IK_QP

# 工具函数
from .utils import angle_axis, p_servo

# 机器人模型
from .robot import (

    create_so101_5dof,
    get_robot,
    smooth_joint_motion,
    Robot,
)

__version__ = "0.1.0"

__all__ = [
    # ET/ETS
    "ET",
    "ETS",
    "SE3_to_xyzrpy",
    "xyzrpy_to_SE3",
    
    # DH 参数建模
    "DHRobot",
    "create_dh_robot",
    "create_puma560",
    "create_stanford_arm",
    "create_so101_5dof",
    
    # IK 基类
    "IKSolution",
    "IKSolver",
    
    # IK 求解器
    "IK_NR",
    "IK_GN", 
    "IK_LM",
    "IK_QP",
    
    # 工具函数
    "angle_axis",
    "p_servo",
    
    # 机器人模型
    "create_so101_5dof",
    "get_robot",
    "Robot",
    "smooth_joint_motion",
]
