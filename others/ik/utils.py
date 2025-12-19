"""
逆运动学工具函数

包括角轴误差计算、位置伺服等实用工具。
"""

import numpy as np
from typing import Union


def angle_axis(Te: np.ndarray, Tep: np.ndarray) -> np.ndarray:
    """
    计算两个 SE3 变换之间的角轴误差
    
    计算当前末端执行器位姿 Te 和期望位姿 Tep 之间的 6 维误差向量。
    前 3 个元素是位置误差，后 3 个元素是使用角轴表示的旋转误差。
    
    Parameters
    ----------
    Te : np.ndarray
        当前末端执行器位姿 (4x4)
    Tep : np.ndarray
        期望末端执行器位姿 (4x4)
        
    Returns
    -------
    e : np.ndarray
        6 维误差向量 [位置误差(3), 旋转误差(3)]
        
    Examples
    --------
    >>> import numpy as np
    >>> Te = np.eye(4)
    >>> Tep = np.eye(4)
    >>> Tep[:3, 3] = [0.1, 0, 0]  # 沿 x 轴平移 0.1
    >>> e = angle_axis(Te, Tep)
    >>> print(e[:3])  # 位置误差
    [0.1 0.  0. ]
    """
    e = np.zeros(6)
    
    # 位置误差
    e[:3] = Tep[:3, 3] - Te[:3, 3]
    
    # 旋转误差：使用角轴表示
    R = Tep[:3, :3] @ Te[:3, :3].T
    
    # 计算 li = [R[2,1] - R[1,2], R[0,2] - R[2,0], R[1,0] - R[0,1]]
    li = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ])
    
    li_norm = np.linalg.norm(li)
    
    if li_norm < 1e-6:
        # 对角矩阵情况
        if np.trace(R) > 0:
            # (1,1,1) 情况 - 无旋转
            e[3:] = np.zeros(3)
        else:
            # 180度旋转情况
            e[3:] = np.pi / 2 * (np.diag(R) + 1)
    else:
        # 非对角矩阵情况
        angle = np.arctan2(li_norm, np.trace(R) - 1)
        e[3:] = angle * li / li_norm
    
    return e


def p_servo(
    wTe: Union[np.ndarray, object],
    wTep: Union[np.ndarray, object],
    gain: Union[float, np.ndarray] = 1.0,
    threshold: float = 0.1,
    method: str = "rpy"
) -> tuple:
    """
    基于位置的伺服控制
    
    返回使机器人接近期望位姿的末端执行器速度。
    
    Parameters
    ----------
    wTe : np.ndarray or SE3
        当前末端执行器在基座标系中的位姿
    wTep : np.ndarray or SE3
        期望末端执行器在基座标系中的位姿
    gain : float or np.ndarray
        控制器增益。可以是标量（应用于所有轴）或 6 维向量
    threshold : float
        机器人位姿与期望位姿之间最终误差的阈值或容差
    method : str
        计算误差的方法：
        - 'rpy': 末端执行器坐标系中的误差（默认）
        - 'angle-axis': 基座标系中使用角轴方法的误差
        
    Returns
    -------
    v : np.ndarray
        使机器人接近 wTep 的末端执行器速度 (6维)
    arrived : bool
        如果机器人在期望位姿的阈值内则为 True
        
    Examples
    --------
    >>> import numpy as np
    >>> wTe = np.eye(4)
    >>> wTep = np.eye(4)
    >>> wTep[:3, 3] = [0.1, 0, 0]
    >>> v, arrived = p_servo(wTe, wTep, gain=1.0)
    >>> print(v[:3])  # 平移速度
    [0.1 0.  0. ]
    >>> print(arrived)
    False
    """
    # 处理 SE3 对象
    if hasattr(wTe, 'A'):
        wTe = wTe.A
    if hasattr(wTep, 'A'):
        wTep = wTep.A
    
    if method == "rpy":
        # 位姿差异（在末端执行器坐标系中）
        eTep = np.linalg.inv(wTe) @ wTep
        e = np.zeros(6)
        
        # 平移误差
        e[:3] = eTep[:3, 3]
        
        # 旋转误差（使用 RPY）
        e[3:] = _tr2rpy(eTep)
    else:
        # 使用角轴方法
        e = angle_axis(wTe, wTep)
    
    # 应用增益
    if np.isscalar(gain):
        k = gain * np.eye(6)
    else:
        k = np.diag(gain)
    
    v = k @ e
    
    # 检查是否到达
    arrived = np.sum(np.abs(e)) < threshold
    
    return v, arrived


def _tr2rpy(T: np.ndarray, order: str = 'zyx') -> np.ndarray:
    """
    从齐次变换矩阵中提取 Roll-Pitch-Yaw 角度
    
    Parameters
    ----------
    T : np.ndarray
        4x4 齐次变换矩阵
    order : str
        欧拉角顺序（默认 'zyx' 对应 RPY）
        
    Returns
    -------
    rpy : np.ndarray
        [roll, pitch, yaw] 角度（弧度）
    """
    R = T[:3, :3]
    
    if order == 'zyx':
        # Roll-Pitch-Yaw (ZYX 欧拉角)
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
        
        if np.abs(pitch - np.pi/2) < 1e-6:
            # 万向节锁：pitch = 90度
            roll = 0
            yaw = np.arctan2(R[0, 1], R[1, 1])
        elif np.abs(pitch + np.pi/2) < 1e-6:
            # 万向节锁：pitch = -90度
            roll = 0
            yaw = -np.arctan2(R[0, 1], R[1, 1])
        else:
            roll = np.arctan2(R[2, 1], R[2, 2])
            yaw = np.arctan2(R[1, 0], R[0, 0])
        
        return np.array([roll, pitch, yaw])
    
    raise ValueError(f"Unsupported order: {order}")


def null_space_projection(
    ets,
    q: np.ndarray,
    J: np.ndarray,
    lambda_sigma: float = 0.0,
    lambda_m: float = 0.0,
    ps: float = 0.0,
    pi: Union[float, np.ndarray] = 0.3,
) -> np.ndarray:
    """
    计算零空间运动以进行关节限位避免和可操作度最大化
    
    Parameters
    ----------
    ets : ETS
        机械臂运动学
    q : np.ndarray
        当前关节配置
    J : np.ndarray
        雅可比矩阵
    lambda_sigma : float
        关节限位避免的增益
    lambda_m : float
        可操作度最大化的增益
    ps : float
        关节允许接近其限位的最小角度/距离
    pi : float or np.ndarray
        零空间运动变得活跃的影响角度/距离
        
    Returns
    -------
    qnull : np.ndarray
        期望的零空间运动
    """
    qnull = np.zeros(ets.n)
    
    if lambda_sigma > 0:
        # 关节限位避免
        Sigma = _compute_sigma(ets, q, ps, pi)
        qnull += (1.0 / lambda_sigma) * Sigma.flatten()
    
    if lambda_m > 0:
        # 可操作度最大化
        Jm = _compute_manipulability_jacobian(ets, q)
        qnull += (1.0 / lambda_m) * Jm.flatten()
    
    # 投影到零空间
    if lambda_sigma > 0 or lambda_m > 0:
        null_space = np.eye(ets.n) - np.linalg.pinv(J) @ J
        qnull = null_space @ qnull
    
    return qnull


def _compute_sigma(ets, q: np.ndarray, ps: float, pi: Union[float, np.ndarray]) -> np.ndarray:
    """计算关节限位避免的梯度"""
    if isinstance(pi, float):
        pi = pi * np.ones(ets.n)
    
    Sigma = np.zeros((ets.n, 1))
    qlim = ets.qlim
    
    for i in range(ets.n):
        qi = q[i]
        ql0 = qlim[0, i]
        ql1 = qlim[1, i]
        
        if qi - ql0 <= pi[i]:
            Sigma[i, 0] = -((qi - ql0) - pi[i])**2 / (ps - pi[i])**2
        if ql1 - qi <= pi[i]:
            Sigma[i, 0] = ((ql1 - qi) - pi[i])**2 / (ps - pi[i])**2
    
    return -Sigma


def _compute_manipulability_jacobian(ets, q: np.ndarray) -> np.ndarray:
    """计算可操作度对关节的雅可比"""
    # 简化实现：返回零向量
    # 完整实现需要计算可操作度的梯度
    return np.zeros((ets.n, 1))


class IKResult:
    """IK 求解结果封装类，兼容 roboticstoolbox 接口"""
    def __init__(self, success, q, reason="", iterations=0, total_time=0.0, final_error=0.0):
        self.success = success
        self.q = q
        self.reason = reason
        self.iterations = iterations
        self.total_time = total_time
        self.final_error = final_error

    def __repr__(self):
        return f"IKResult(success={self.success}, q={self.q}, reason='{self.reason}')"
