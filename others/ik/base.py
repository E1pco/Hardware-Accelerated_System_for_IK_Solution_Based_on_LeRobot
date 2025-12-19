"""
Base classes and data structures for inverse kinematics solvers
"""

import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Union, Optional, Tuple


@dataclass
class IKSolution:
    """
    Data class for inverse kinematics solution results
    
    Attributes
    ----------
    q : np.ndarray
        Joint coordinate solution (note: if solving fails, this value may be invalid)
    success : bool
        Whether a valid solution was found
    iterations : int
        Number of iterations performed
    searches : int
        Number of searches performed
    residual : float
        Final error value
    reason : str
        Reason for solving failure (if applicable)
    """
    q: np.ndarray
    success: bool
    iterations: int = 0
    searches: int = 0
    residual: float = 0.0
    reason: str = ""

    def __iter__(self):
        """支持解构赋值"""
        return iter((
            self.q,
            self.success,
            self.iterations,
            self.searches,
            self.residual,
            self.reason,
        ))

    def __str__(self):
        """Friendly string representation"""
        if self.q is not None:
            q_str = np.array2string(
                self.q,
                separator=", ",
                formatter={
                    "float": lambda x: "{:.4g}".format(0 if abs(x) < 1e-6 else x)
                },
            )
        else:
            q_str = None

        if self.iterations == 0 and self.searches == 0:
            # Analytical solution
            if self.success:
                return f"IKSolution: q={q_str}, success=True"
            else:
                return f"IKSolution: q={q_str}, success=False, reason={self.reason}"
        else:
            # Numerical solution
            if self.success:
                return (
                    f"IKSolution: q={q_str}, success=True, "
                    f"iterations={self.iterations}, searches={self.searches}, "
                    f"residual={self.residual:.3g}"
                )
            else:
                return (
                    f"IKSolution: q={q_str}, success=False, reason={self.reason}, "
                    f"iterations={self.iterations}, searches={self.searches}, "
                    f"residual={self.residual:.3g}"
                )


class IKSolver(ABC):
    """
    Abstract base class for numerical inverse kinematics solvers
    
    This class provides basic functionality for performing numerical inverse kinematics. 
    Subclasses need to implement the step method to define the specific optimization algorithm.
    
    Parameters
    ----------
    name : str
        Solver name
    ilimit : int
        Maximum number of iterations allowed per search
    slimit : int
        Maximum number of searches allowed
    tol : float
        Maximum residual error allowed
    mask : np.ndarray, optional
        6维向量，用于分配笛卡尔自由度的权重
    joint_limits : bool
        是否拒绝违反关节限位的解
    seed : int, optional
        随机数生成器的种子
    """

    def __init__(
        self,
        name: str = "IK Solver",
        ilimit: int = 30,
        slimit: int = 100,
        tol: float = 1e-6,
        mask: Optional[np.ndarray] = None,
        joint_limits: bool = True,
        seed: Optional[int] = None,
    ):
        self.name = name
        self.ilimit = ilimit
        self.slimit = slimit
        self.tol = tol
        self.joint_limits = joint_limits
        
        # Error weight matrix
        if mask is None:
            mask = np.ones(6)
        self.We = np.diag(mask)
        
        # Private random number generator
        self._rng = np.random.default_rng(seed=seed)

    def solve(
        self,
        ets,
        Tep: Union[np.ndarray, object],
        q0: Optional[np.ndarray] = None,
        mask: Optional[np.ndarray] = None,
    ) -> IKSolution:
        """
        Solve the inverse kinematics problem
        
        Attempt to find joint coordinates that bring the end-effector to the desired pose Tep.
        
        Parameters
        ----------
        ets : ETS
            ETS object representing the robot kinematics
        Tep : np.ndarray or SE3
            Desired end-effector pose (4x4 matrix or SE3 object)
        q0 : np.ndarray, optional
            初始关节坐标向量
        mask : np.ndarray, optional
            6 元素权重数组，分配笛卡尔自由度的权重 [x, y, z, rx, ry, rz]
            值为 0 表示忽略该自由度，> 0 表示该自由度的权重
            默认为 None（所有自由度权重为 1.0）
            
        Returns
        -------
        IKSolution
            Data class containing the solution result
        """
        # Handle SE3 object
        if hasattr(Tep, 'A'):
            Tep = Tep.A
        
        # Ensure it's a 4x4 matrix
        if Tep.shape != (4, 4):
            raise ValueError("Tep must be a 4x4 SE3 matrix")
        
        # Handle mask 
        if mask is None:
            mask = np.ones(6)
        else:
            mask = np.array(mask, dtype=float)
            if mask.shape != (6,):
                raise ValueError("mask must be a 6-element weight array")
        
        # Prepare initial guess
        if q0 is None:
            q0_array = self._random_q(ets, self.slimit)
        elif q0.ndim == 1:
            q0_array = np.tile(q0, (self.slimit, 1))
        else:
            q0_array = q0
            if q0_array.shape[0] < self.slimit:
                # Fill more random initial guesses
                extra = self._random_q(ets, self.slimit - q0_array.shape[0])
                q0_array = np.vstack([q0_array, extra])
        
        return self._solve(ets, Tep, q0_array, mask)

    def _solve(
        self,
        ets,
        Tep: np.ndarray,
        q0: np.ndarray,
        mask: np.ndarray,
    ) -> IKSolution:
        """
        内部求解方法
        
        执行多次搜索，每次从不同的初始位置开始。
        """
        E = 0.0
        q = q0[0].copy()
        found_with_limits = False
        linalg_error_count = 0
        
        for search in range(self.slimit):
            q = q0[search].copy()
            i = 0
            
            for i in range(self.ilimit):
                try:
                    # 执行一步优化
                    E, q = self.step(ets, Tep, q, mask)
                    
                    # 检查是否收敛
                    if E < self.tol:
                        # 将 q 归一化到 [-π, π]
                        q = (q + np.pi) % (2 * np.pi) - np.pi
                        
                        # 检查关节限位
                        if self.joint_limits:
                            jl_valid = self._check_jl(ets, q)
                            if not jl_valid:
                                found_with_limits = True
                                break
                        
                        return IKSolution(
                            q=q,
                            success=True,
                            iterations=(search * self.ilimit + i + 1),
                            searches=search + 1,
                            residual=E,
                            reason="Success",
                        )
                
                except np.linalg.LinAlgError:
                    linalg_error_count += 1
                    break
        
        # 构建失败原因
        reason = "iteration and search limit reached"
        if linalg_error_count > 0:
            reason += f", {linalg_error_count} LinAlgError encountered"
        if found_with_limits:
            reason += ", solution found but violates joint limits"
        
        return IKSolution(
            q=q,
            success=False,
            iterations=self.slimit * self.ilimit,
            searches=self.slimit,
            residual=E,
            reason=reason,
        )

    def error(self, Te: np.ndarray, Tep: np.ndarray) -> tuple:
        """
        计算当前位姿 Te 和期望位姿 Tep 之间的误差
        
        使用角轴表示法计算误差，并返回加权二次误差。
        
        Parameters
        ----------
        Te : np.ndarray
            当前末端执行器位姿 (4x4)
        Tep : np.ndarray
            期望末端执行器位姿 (4x4)
            
        Returns
        -------
        e : np.ndarray
            6维角轴误差向量
        E : float
            加权二次误差 E = 0.5 * e^T * We * e
        """
        from .utils import angle_axis
        
        e = angle_axis(Te, Tep)
        E = 0.5 * e @ self.We @ e
        
        return e, E

    @abstractmethod
    def step(self, ets, Tep: np.ndarray, q: np.ndarray, mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        执行一步迭代
        
        Parameters
        ----------
        ets : ETS
            机器人的运动学链
        Tep : np.ndarray
            目标末端位姿
        q : np.ndarray
            当前关节角度
        mask : np.ndarray
            6 元素权重数组，分配笛卡尔自由度的权重
            
        Returns
        -------
        E : np.ndarray
            当前误差
        q_new : np.ndarray
            更新后的关节角度
        """
        pass

    def _random_q(self, ets, n: int = 1) -> np.ndarray:
        """
        在关节限位内生成随机有效的关节配置
        
        Parameters
        ----------
        ets : ETS
            机械臂运动学
        n : int
            要生成的配置数量
            
        Returns
        -------
        q : np.ndarray
            形状为 (n, n_joints) 的随机关节配置
        """
        qlim = ets.qlim
        n_joints = ets.n
        
        q = np.zeros((n, n_joints))
        for i in range(n_joints):
            q[:, i] = self._rng.uniform(qlim[0, i], qlim[1, i], n)
        
        return q

    def _check_jl(self, ets, q: np.ndarray) -> bool:
        """
        检查关节是否在限位范围内
        
        Parameters
        ----------
        ets : ETS
            机械臂运动学
        q : np.ndarray
            关节坐标向量
            
        Returns
        -------
        bool
            如果关节在限位内返回 True，否则返回 False
        """
        qlim = ets.qlim
        
        for i in range(ets.n):
            if q[i] < qlim[0, i] or q[i] > qlim[1, i]:
                return False
        
        return True
