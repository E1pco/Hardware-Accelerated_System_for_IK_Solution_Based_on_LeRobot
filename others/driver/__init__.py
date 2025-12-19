"""
LeRobot RDT - Robot Demonstration Toolkit
主要模块包括:
- ftservo_controller: 舵机控制器
- ik.robot: 机器人运动学模型
- joyconrobotics: Joy-Con 控制
"""

# 导出常用的类和函数，方便外界引用
from .ftservo_controller import ServoController

__version__ = "1.0.0"
__all__ = [
    "ServoController",
    "create_so101_5dof",
]
