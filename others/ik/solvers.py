"""
各种数值逆运动学求解器的实现

包括：
- Newton-Raphson (IK_NR)
- Gauss-Newton (IK_GN)
- Levenberg-Marquardt (IK_LM)
- Quadratic Programming (IK_QP)
"""

import numpy as np
from typing import Union, Optional
from .base import IKSolver
from .utils import null_space_projection

try:
    import qpsolvers as qp
    _HAS_QP = True
except ImportError:
    _HAS_QP = False


class IK_NR(IKSolver):
    """
    Newton-Raphson 数值逆运动学求解器
    
    使用 Newton-Raphson 方法进行数值逆运动学求解。
    这个方法收敛速度很快，但对初始猜测敏感，并且要求雅可比矩阵非奇异。
    
    算法：q_{k+1} = q_k + J^{-1} * e_k
    
    Parameters
    ----------
    pinv : bool
        如果为 True，使用伪逆代替普通逆（用于冗余机器人）
    kq : float
        关节限位避免的增益
    km : float
        可操作度最大化的增益
    ps : float
        关节允许接近限位的最小角度/距离
    pi : float or np.ndarray
        零空间运动的影响角度/距离
    **kwargs
        传递给 IKSolver 的其他参数
        
    Examples
    --------
    >>> from ik import IK_NR
    >>> solver = IK_NR(pinv=True)  # 用于冗余机器人
    >>> solution = solver.solve(robot.ets(), Tep, q0)
    """
    
    def __init__(
        self,
        pinv: bool = False,
        kq: float = 0.0,
        km: float = 0.0,
        ps: float = 0.0,
        pi: Union[float, np.ndarray] = 0.3,
        **kwargs
    ):
        super().__init__(**kwargs)
        self.pinv = pinv
        self.kq = kq
        self.km = km
        self.ps = ps
        self.pi = pi
        
        self.name = f"NR (pinv={pinv})"
        if self.kq > 0:
            self.name += " Σ"
        if self.km > 0:
            self.name += " Jm"
    
    def step(self, ets, Tep: np.ndarray, q: np.ndarray, mask: np.ndarray) -> tuple:
        """
        执行一步 Newton-Raphson 优化
        
        Parameters
        ----------
        ets : ETS
            机械臂运动学
        Tep : np.ndarray
            期望位姿
        q : np.ndarray
            当前关节坐标
        mask : np.ndarray
            6 元素权重数组，分配笛卡尔自由度的权重
            
        Returns
        -------
        E : float
            新的误差值
        q : np.ndarray
            新的关节坐标
        """
        # 计算当前位姿
        Te = ets.eval(q)
        
        # 计算误差
        from .utils import angle_axis
        e = angle_axis(Te, Tep)
        
        # 构建权重矩阵
        We = np.diag(mask)
        
        # 计算加权误差
        E = 0.5 * e @ We @ e
        
        # 应用权重到误差向量
        e = We @ e
        
        # 计算雅可比
        J = ets.jacob0(q)
        
        # 计算零空间运动
        qnull = null_space_projection(
            ets, q, J,
            lambda_sigma=self.kq,
            lambda_m=self.km,
            ps=self.ps,
            pi=self.pi
        )
        
        # 计算关节速度
        if self.pinv:
            q += np.linalg.pinv(J) @ e + qnull
        else:
            q += np.linalg.solve(J, e) + qnull
        
        return E, q


class IK_GN(IKSolver):
    """
    Gauss-Newton 数值逆运动学求解器
    
    使用 Gauss-Newton 方法进行数值逆运动学求解。
    这个方法通过最小化误差的平方来工作。
    
    算法：q_{k+1} = q_k + (J^T W_e J)^{-1} J^T W_e e_k
    
    Parameters
    ----------
    pinv : bool
        如果为 True，使用伪逆代替普通逆（用于冗余机器人）
    kq : float
        关节限位避免的增益
    km : float
        可操作度最大化的增益
    ps : float
        关节允许接近限位的最小角度/距离
    pi : float or np.ndarray
        零空间运动的影响角度/距离
    **kwargs
        传递给 IKSolver 的其他参数
        
    Examples
    --------
    >>> from ik import IK_GN
    >>> solver = IK_GN(pinv=True)
    >>> solution = solver.solve(robot.ets(), Tep, q0)
    """
    
    def __init__(
        self,
        pinv: bool = False,
        kq: float = 0.0,
        km: float = 0.0,
        ps: float = 0.0,
        pi: Union[float, np.ndarray] = 0.3,
        **kwargs
    ):
        super().__init__(**kwargs)
        self.pinv = pinv
        self.kq = kq
        self.km = km
        self.ps = ps
        self.pi = pi
        
        self.name = f"GN (pinv={pinv})"
        if self.kq > 0:
            self.name += " Σ"
        if self.km > 0:
            self.name += " Jm"
    
    def step(self, ets, Tep: np.ndarray, q: np.ndarray, mask: np.ndarray) -> tuple:
        """
        执行一步 Gauss-Newton 优化
        """
        # 计算当前位姿
        Te = ets.eval(q)
        
        # 计算误差
        from .utils import angle_axis
        e = angle_axis(Te, Tep)
        
        # 构建权重矩阵
        We = np.diag(mask)
        
        # 计算加权误差
        E = 0.5 * e @ We @ e
        
        # 应用权重到误差向量（用于后续计算）
        weighted_e = We @ e
        
        # 计算雅可比
        J = ets.jacob0(q)
        
        # 计算零空间运动
        qnull = null_space_projection(
            ets, q, J,
            lambda_sigma=self.kq,
            lambda_m=self.km,
            ps=self.ps,
            pi=self.pi
        )
        
        # 计算关节速度
        if self.pinv:
            q += np.linalg.pinv(J) @ weighted_e + qnull
        else:
            # q += (J^T W J)^{-1} J^T W e
            JtW = J.T @ We
            A = JtW @ J
            g = JtW @ e
            q += np.linalg.solve(A, g) + qnull
        
        return E, q


class IK_LM(IKSolver):
    """
    Levenberg-Marquardt 数值逆运动学求解器
    
    使用 Levenberg-Marquardt 方法进行数值逆运动学求解。
    这个方法通过添加阻尼项来提高数值稳定性。
    
    算法：q_{k+1} = q_k + (J^T W_e J + W_n)^{-1} J^T W_e e_k
    
    其中 W_n 是阻尼矩阵，有多种计算方法。
    
    Parameters
    ----------
    k : float
        阻尼系数
    method : str
        阻尼矩阵计算方法：
        - 'chan': W_n = λ * E * I
        - 'wampler': W_n = λ * I
        - 'sugihara': W_n = E * I + λ * I
    kq : float
        关节限位避免的增益
    km : float
        可操作度最大化的增益
    ps : float
        关节允许接近限位的最小角度/距离
    pi : float or np.ndarray
        零空间运动的影响角度/距离
    **kwargs
        传递给 IKSolver 的其他参数
        
    Examples
    --------
    >>> from ik import IK_LM
    >>> solver = IK_LM(k=1.0, method='chan')
    >>> solution = solver.solve(robot.ets(), Tep, q0)
    """
    
    def __init__(
        self,
        k: float = 1.0,
        method: str = "chan",
        kq: float = 0.0,
        km: float = 0.0,
        ps: float = 0.0,
        pi: Union[float, np.ndarray] = 0.3,
        **kwargs
    ):
        super().__init__(**kwargs)
        self.k = k
        self.kq = kq
        self.km = km
        self.ps = ps
        self.pi = pi
        
        method_lower = method.lower()
        if method_lower.startswith("sugi"):
            self.method = "sugihara"
            method_name = "Sugihara"
        elif method_lower.startswith("wamp"):
            self.method = "wampler"
            method_name = "Wampler"
        else:
            self.method = "chan"
            method_name = "Chan"
        
        self.name = f"LM ({method_name} λ={k})"
        if self.kq > 0:
            self.name += " Σ"
        if self.km > 0:
            self.name += " Jm"
    
    def step(self, ets, Tep: np.ndarray, q: np.ndarray, mask: np.ndarray ) -> tuple:
        """
        执行一步 Levenberg-Marquardt 优化
        """
        # 计算当前位姿
        Te = ets.eval(q)
        
        # 计算误差
        from .utils import angle_axis
        e = angle_axis(Te, Tep)
        
        # 构建权重矩阵
        We = np.diag(mask)
        
        # 计算加权误差
        E = 0.5 * e @ We @ e
        
        # 计算雅可比
        J = ets.jacob0(q)
        
        # 计算阻尼矩阵
        n = ets.n
        if self.method == "chan":
            Wn = self.k * E * np.eye(n)
        elif self.method == "wampler":
            Wn = self.k * np.eye(n)
        else:  # sugihara
            Wn = E * np.eye(n) + self.k * np.eye(n)
        
        # 计算零空间运动
        qnull = null_space_projection(
            ets, q, J,
            lambda_sigma=self.kq,
            lambda_m=self.km,
            ps=self.ps,
            pi=self.pi
        )
        
        # 计算关节速度
        JtW = J.T @ We
        A = JtW @ J + Wn
        g = JtW @ e
        q += np.linalg.solve(A, g) + qnull
        
        return E, q


class IK_QP(IKSolver):
    """
    基于二次规划的数值逆运动学求解器
    
    使用二次规划方法进行数值逆运动学求解。
    这个方法可以显式地处理关节限位和其他约束。
    
    需要 qpsolvers 包。
    
    Parameters
    ----------
    kj : float
        关节速度范数最小化的增益
    ks : float
        松弛变量成本的增益
    kq : float
        关节限位避免的增益
    km : float
        可操作度最大化的增益
    ps : float
        关节允许接近限位的最小角度/距离
    pi : float or np.ndarray
        零空间运动的影响角度/距离
    **kwargs
        传递给 IKSolver 的其他参数
        
    Raises
    ------
    ImportError
        如果 qpsolvers 包未安装
        
    Examples
    --------
    >>> from ik import IK_QP
    >>> solver = IK_QP(kj=0.01, ks=1.0)
    >>> solution = solver.solve(robot.ets(), Tep, q0)
    """
    
    def __init__(
        self,
        kj: float = 0.01,
        ks: float = 1.0,
        kq: float = 0.0,
        km: float = 0.0,
        ps: float = 0.0,
        pi: Union[float, np.ndarray] = 0.3,
        **kwargs
    ):
        if not _HAS_QP:
            raise ImportError(
                "qpsolvers package is required for IK_QP. "
                "Install it with: pip install qpsolvers"
            )
        
        super().__init__(**kwargs)
        self.kj = kj
        self.ks = ks
        self.kq = kq
        self.km = km
        self.ps = ps
        self.pi = pi
        
        self.name = "QP"
        if self.kq > 0:
            self.name += " Σ"
        if self.km > 0:
            self.name += " Jm"
    
    def step(self, ets, Tep: np.ndarray, q: np.ndarray, mask: np.ndarray) -> tuple:
        """
        执行一步二次规划优化
        """
        # 计算当前位姿
        Te = ets.eval(q)
        
        # 计算误差
        from .utils import angle_axis
        e = angle_axis(Te, Tep)
        
        # 构建权重矩阵
        We = np.diag(mask)
        
        # 计算加权误差
        E = 0.5 * e @ We @ e
        
        # 应用权重到误差向量
        e = We @ e
        
        # 计算雅可比
        J = ets.jacob0(q)
        
        n = ets.n
        
        # 构建 QP 问题
        # 决策变量: x = [dq, delta]，其中 delta 是松弛变量
        
        # 目标函数: min 0.5 * x^T * Q * x + c^T * x
        Q = np.zeros((n + 6, n + 6))
        Q[:n, :n] = self.kj * np.eye(n)
        Q[n:, n:] = self.ks * (1.0 / np.sum(np.abs(e))) * np.eye(6)
        
        c = np.zeros(n + 6)
        
        # 等式约束: J * dq + delta = e
        Aeq = np.hstack([J, np.eye(6)])
        beq = e
        
        # 不等式约束（关节限位避免）
        if self.kq > 0:
            if isinstance(self.pi, float):
                pi = self.pi * np.ones(n)
            else:
                pi = self.pi
            
            Ain = np.zeros((n + 6, n + 6))
            bin = np.zeros(n + 6)
            
            qlim = ets.qlim
            for i in range(n):
                ql0 = qlim[0, i]
                ql1 = qlim[1, i]
                
                if ql1 - q[i] <= pi[i]:
                    bin[i] = ((ql1 - q[i]) - self.ps) / (pi[i] - self.ps)
                    Ain[i, i] = 1
                
                if q[i] - ql0 <= pi[i]:
                    bin[i] = -(((ql0 - q[i]) + self.ps) / (pi[i] - self.ps))
                    Ain[i, i] = -1
            
            bin = (1.0 / self.kq) * bin
        else:
            Ain = None
            bin = None
        
        # 求解 QP
        try:
            x = qp.solve_qp(
                Q, c, Ain, bin, Aeq, beq,
                solver="quadprog"
            )
            
            if x is None:
                raise np.linalg.LinAlgError("QP unsolvable")
            
            # 更新关节坐标
            q += x[:n]
        except Exception as ex:
            raise np.linalg.LinAlgError(f"QP failed: {ex}")
        
        return E, q
