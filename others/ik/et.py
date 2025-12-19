"""
Pure Python implementation of Elementary Transform (ET) class

Provides basic transformation representations for robot kinematics, including:
- ET: Single elementary transform (translation, rotation)
- ETS: Sequence of elementary transforms for building complete kinematic chains

This is a fully Python-implemented version that does not depend on C++ extensions.
"""

import numpy as np
from typing import List, Optional, Union, Tuple
from enum import Enum


class ETType(Enum):
    """Elementary transform type enumeration"""
    RX = 'Rx'  # Rotation around X axis
    RY = 'Ry'  # Rotation around Y axis
    RZ = 'Rz'  # Rotation around Z axis
    TX = 'tx'  # Translation along X axis
    TY = 'ty'  # Translation along Y axis
    TZ = 'tz'  # Translation along Z axis


class ET:
    """
    Elementary Transform
    
    Represents a single rigid body transformation, which can be translation or rotation.
    Supports constant transforms (fixed values) or variable transforms (joint angles).
    
    Examples
    --------
    >>> # Create fixed translation
    >>> t1 = ET.tx(0.1)
    >>> 
    >>> # Create rotational joint
    >>> r1 = ET.Ry()  # Variable rotation
    >>> 
    >>> # Combine transforms
    >>> ets = t1 * r1
    """
    
    def __init__(self, et_type: ETType, value: Optional[float] = None, 
                 joint: bool = False, flip: bool = False):
        """
        初始化基本变换
        
        Parameters
        ----------
        et_type : ETType
            Transform type (Rx, Ry, Rz, tx, ty, tz)
        value : float, optional
            Value of constant transform (radians or meters)
        joint : bool
            Whether it is a joint variable
        flip : bool
            Whether to flip the transform direction
        """
        self.et_type = et_type
        self._value = value
        self.joint = joint if value is None else False
        self.flip = flip
        
    @property
    def isjoint(self) -> bool:
        """Whether it is a joint variable"""
        return self.joint
    
    @property
    def value(self) -> Optional[float]:
        """Get constant value"""
        return self._value
    
    def __str__(self) -> str:
        """String representation"""
        if self.joint:
            return f"{self.et_type.value}(q)"
        elif self._value is not None:
            return f"{self.et_type.value}({self._value:.4f})"
        else:
            return f"{self.et_type.value}(0)"
    
    def __repr__(self) -> str:
        return self.__str__()
    
    def __mul__(self, other):
        """
        ET * ET -> ETS
        连接两个变换形成变换序列
        """
        if isinstance(other, ET):
            return ETS([self, other])
        elif isinstance(other, ETS):
            return ETS([self] + other.ets)
        else:
            raise TypeError(f"Cannot multiply ET with {type(other)}")
    
    def T(self, q: Optional[float] = None) -> np.ndarray:
        """
        Compute homogeneous transformation matrix
        
        Parameters
        ----------
        q : float, optional
            Value of joint variable (if it is a joint transform)
            
        Returns
        -------
        np.ndarray
            4x4 homogeneous transformation matrix
        """
        # Determine the value to use
        if self.joint:
            if q is None:
                raise ValueError("Joint variable requires a value")
            theta = q if not self.flip else -q
        else:
            theta = self._value if self._value is not None else 0.0
            if self.flip:
                theta = -theta
        
        # Build transformation matrix
        T = np.eye(4)
        
        if self.et_type == ETType.RX:
            c, s = np.cos(theta), np.sin(theta)
            T[:3, :3] = np.array([
                [1, 0,  0],
                [0, c, -s],
                [0, s,  c]
            ])
        elif self.et_type == ETType.RY:
            c, s = np.cos(theta), np.sin(theta)
            T[:3, :3] = np.array([
                [ c, 0, s],
                [ 0, 1, 0],
                [-s, 0, c]
            ])
        elif self.et_type == ETType.RZ:
            c, s = np.cos(theta), np.sin(theta)
            T[:3, :3] = np.array([
                [c, -s, 0],
                [s,  c, 0],
                [0,  0, 1]
            ])
        elif self.et_type == ETType.TX:
            T[0, 3] = theta
        elif self.et_type == ETType.TY:
            T[1, 3] = theta
        elif self.et_type == ETType.TZ:
            T[2, 3] = theta
            
        return T
    
    # 静态工厂方法
    @staticmethod
    def Rx(q: Optional[float] = None, flip: bool = False):
        """绕 X 轴旋转变换"""
        return ET(ETType.RX, value=q, joint=(q is None), flip=flip)
    
    @staticmethod
    def Ry(q: Optional[float] = None, flip: bool = False):
        """绕 Y 轴旋转变换"""
        return ET(ETType.RY, value=q, joint=(q is None), flip=flip)
    
    @staticmethod
    def Rz(q: Optional[float] = None, flip: bool = False):
        """绕 Z 轴旋转变换"""
        return ET(ETType.RZ, value=q, joint=(q is None), flip=flip)
    
    @staticmethod
    def tx(d: float = 0.0, flip: bool = False):
        """沿 X 轴平移变换"""
        return ET(ETType.TX, value=d, joint=False, flip=flip)
    
    @staticmethod
    def ty(d: float = 0.0, flip: bool = False):
        """沿 Y 轴平移变换"""
        return ET(ETType.TY, value=d, joint=False, flip=flip)
    
    @staticmethod
    def tz(d: float = 0.0, flip: bool = False):
        """沿 Z 轴平移变换"""
        return ET(ETType.TZ, value=d, joint=False, flip=flip)


class ETS:
    """
    基本变换序列（Elementary Transform Sequence）
    
    表示一系列基本变换的组合，用于描述机器人的运动学链。
    支持正运动学、雅可比计算等功能。
    

    """
    
    def __init__(self, ets: List[ET]):
        """
        初始化变换序列
        
        Parameters
        ----------
        ets : List[ET]
            基本变换列表
        """
        self.ets = ets
        self._qlim = None
        
        # 统计关节数量
        self._n = sum(1 for et in ets if et.isjoint)
        
        # 构建关节索引映射
        self._joint_indices = [i for i, et in enumerate(ets) if et.isjoint]
    
    @property
    def n(self) -> int:
        """关节数量"""
        return self._n
    
    @property
    def qlim(self) -> Optional[np.ndarray]:
        """
        关节限位
        
        Returns
        -------
        np.ndarray or None
            2 x n 数组，第一行为下限，第二行为上限
        """
        return self._qlim
    
    @qlim.setter
    def qlim(self, limits: np.ndarray):
        """设置关节限位"""
        if limits.shape != (2, self.n):
            raise ValueError(f"qlim must be 2 x {self.n}, got {limits.shape}")
        self._qlim = limits
    
    def __str__(self) -> str:
        """字符串表示"""
        return " * ".join(str(et) for et in self.ets)
    
    def __repr__(self) -> str:
        return f"ETS({self.__str__()})"
    
    def __mul__(self, other):
        """
        ETS * ETS -> ETS
        ETS * ET -> ETS
        连接变换序列
        """
        if isinstance(other, ETS):
            return ETS(self.ets + other.ets)
        elif isinstance(other, ET):
            return ETS(self.ets + [other])
        else:
            raise TypeError(f"Cannot multiply ETS with {type(other)}")
    
    def fkine(self, q: np.ndarray) -> np.ndarray:
        """
        计算正运动学
        
        Parameters
        ----------
        q : np.ndarray
            关节角度向量，长度必须为 n
            
        Returns
        -------
        np.ndarray
            4x4 齐次变换矩阵，表示末端执行器相对于基座标系的位姿
            
        Raises
        ------
        ValueError
            如果 q 的长度与关节数不匹配
        """
        if len(q) != self.n:
            raise ValueError(f"q must have length {self.n}, got {len(q)}")
        
        T = np.eye(4)
        q_idx = 0
        
        for et in self.ets:
            if et.isjoint:
                T = T @ et.T(q[q_idx])
                q_idx += 1
            else:
                T = T @ et.T()
        
        return T
    
    def eval(self, q: np.ndarray) -> np.ndarray:
        """
        计算正运动学（fkine 的别名）
        
        Parameters
        ----------
        q : np.ndarray
            关节角度向量
            
        Returns
        -------
        np.ndarray
            4x4 齐次变换矩阵
        """
        return self.fkine(q)
    
    def jacob0(self, q: np.ndarray, tool: Optional[np.ndarray] = None) -> np.ndarray:
        """
        计算基座标系下的雅可比矩阵
        
        使用微分法计算几何雅可比矩阵。
        
        Parameters
        ----------
        q : np.ndarray
            关节角度向量
        tool : np.ndarray, optional
            工具变换矩阵（4x4）
            
        Returns
        -------
        np.ndarray
            6 x n 雅可比矩阵
            前3行为线速度，后3行为角速度
        """
        if len(q) != self.n:
            raise ValueError(f"q must have length {self.n}, got {len(q)}")
        
        J = np.zeros((6, self.n))
        
        # 计算末端位置
        T_end = self.fkine(q)
        if tool is not None:
            T_end = T_end @ tool
        p_end = T_end[:3, 3]
        
        # 对每个关节计算雅可比列
        T_current = np.eye(4)
        q_idx = 0
        
        for et in self.ets:
            if et.isjoint:
                # 当前关节的位置和轴
                p_joint = T_current[:3, 3]
                
                # 获取关节轴向量
                if et.et_type == ETType.RX:
                    z = T_current[:3, 0]  # X轴
                elif et.et_type == ETType.RY:
                    z = T_current[:3, 1]  # Y轴
                elif et.et_type == ETType.RZ:
                    z = T_current[:3, 2]  # Z轴
                else:
                    # 平移关节（不常见）
                    z = np.zeros(3)
                    if et.et_type == ETType.TX:
                        z = T_current[:3, 0]
                    elif et.et_type == ETType.TY:
                        z = T_current[:3, 1]
                    elif et.et_type == ETType.TZ:
                        z = T_current[:3, 2]
                
                # 旋转关节的雅可比
                if et.et_type in [ETType.RX, ETType.RY, ETType.RZ]:
                    J[:3, q_idx] = np.cross(z, p_end - p_joint)
                    J[3:, q_idx] = z
                else:
                    # 平移关节
                    J[:3, q_idx] = z
                    J[3:, q_idx] = 0
                
                T_current = T_current @ et.T(q[q_idx])
                q_idx += 1
            else:
                T_current = T_current @ et.T()
        
        return J
    
    def jacobe(self, q: np.ndarray, tool: Optional[np.ndarray] = None) -> np.ndarray:
        """
        计算末端坐标系下的雅可比矩阵
        
        Parameters
        ----------
        q : np.ndarray
            关节角度向量
        tool : np.ndarray, optional
            工具变换矩阵
            
        Returns
        -------
        np.ndarray
            6 x n 雅可比矩阵（末端坐标系）
        """
        J0 = self.jacob0(q, tool)
        T = self.fkine(q)
        if tool is not None:
            T = T @ tool
        
        R = T[:3, :3]
        
        # 构建旋转变换矩阵
        R_block = np.zeros((6, 6))
        R_block[:3, :3] = R.T
        R_block[3:, 3:] = R.T
        
        return R_block @ J0
    
    def hessian0(self, q: np.ndarray, J0: Optional[np.ndarray] = None) -> np.ndarray:
        """
        计算基座标系下的Hessian矩阵（二阶导数）
        
        使用数值微分法计算。
        
        Parameters
        ----------
        q : np.ndarray
            关节角度向量
        J0 : np.ndarray, optional
            预先计算的雅可比矩阵（用于加速）
            
        Returns
        -------
        np.ndarray
            6 x n x n Hessian张量
        """
        H = np.zeros((6, self.n, self.n))
        
        eps = 1e-6
        
        for i in range(self.n):
            # 计算 ∂J/∂qi
            q_plus = q.copy()
            q_plus[i] += eps
            
            q_minus = q.copy()
            q_minus[i] -= eps
            
            J_plus = self.jacob0(q_plus)
            J_minus = self.jacob0(q_minus)
            
            dJ_dqi = (J_plus - J_minus) / (2 * eps)
            
            H[:, :, i] = dJ_dqi
        
        return H
    
    def manipulability(self, q: np.ndarray, method: str = 'yoshikawa') -> float:
        """
        计算可操作度
        
        Parameters
        ----------
        q : np.ndarray
            关节角度向量
        method : str
            计算方法：'yoshikawa', 'asada', 'minsingular'
            
        Returns
        -------
        float
            可操作度指标
        """
        J = self.jacob0(q)
        
        if method == 'yoshikawa':
            # Yoshikawa 可操作度：det(JJ^T)^0.5
            return np.sqrt(np.linalg.det(J @ J.T))
        elif method == 'asada':
            # Asada 可操作度：最小奇异值
            return np.linalg.svd(J, compute_uv=False)[-1]
        elif method == 'minsingular':
            # 最小奇异值
            return np.min(np.linalg.svd(J, compute_uv=False))
        else:
            raise ValueError(f"Unknown method: {method}")
    
    def __len__(self) -> int:
        """变换序列的长度"""
        return len(self.ets)
    
    def __getitem__(self, idx) -> ET:
        """获取指定索引的变换"""
        return self.ets[idx]


# 辅助函数
def SE3_to_xyzrpy(T: np.ndarray) -> np.ndarray:
    """
    将齐次变换矩阵转换为 XYZ + RPY 表示
    
    Parameters
    ----------
    T : np.ndarray
        4x4 齐次变换矩阵
        
    Returns
    -------
    np.ndarray
        [x, y, z, roll, pitch, yaw]
    """
    x, y, z = T[0, 3], T[1, 3], T[2, 3]
    
    R = T[:3, :3]
    
    # 计算 ZYX 欧拉角（RPY）
    beta = np.arctan2(-R[2, 0], np.sqrt(R[0, 0]**2 + R[1, 0]**2))
    
    if np.abs(np.cos(beta)) > 1e-6:
        alpha = np.arctan2(R[1, 0] / np.cos(beta), R[0, 0] / np.cos(beta))
        gamma = np.arctan2(R[2, 1] / np.cos(beta), R[2, 2] / np.cos(beta))
    else:
        # 万向节锁
        alpha = 0
        gamma = np.arctan2(R[0, 1], R[1, 1])
    
    return np.array([x, y, z, gamma, beta, alpha])


def xyzrpy_to_SE3(pose: np.ndarray) -> np.ndarray:
    """
    将 XYZ + RPY 表示转换为齐次变换矩阵
    
    Parameters
    ----------
    pose : np.ndarray
        [x, y, z, roll, pitch, yaw]
        
    Returns
    -------
    np.ndarray
        4x4 齐次变换矩阵
    """
    x, y, z, roll, pitch, yaw = pose
    
    # 构建旋转矩阵（ZYX顺序）
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr           ]
    ])
    
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    
    return T
