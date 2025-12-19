#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hardware-accelerated inverse kinematics solver wrapper class
Supports FPGA/PYNQ accelerated LM inverse kinematics algorithm
"""

import numpy as np
import time
from typing import Dict, Optional, Tuple, Union
from ik.utils import angle_axis as angle_axis_error
from ik.utils import IKResult

class HWSolverIterator:
    """FPGA hardware-accelerated solver single-step iteration interface""" 
    
    def __init__(self, solver_ip, robot):
        """
        Initialize hardware solver iterator
        
        Args:
            solver_ip: PYNQ hardware IP core object
            robot: Robot kinematics model
        """
        self.solver_ip = solver_ip
        self.robot = robot
    
    def solve_step(self, q: np.ndarray, error: np.ndarray, 
                   lambda_damping: float = 0.01) -> Tuple[np.ndarray, int]:
        """
        Call hardware accelerator to compute single-step LM iteration
        
        Args:
            q: Current joint angles [5,]
            error: Current error vector [6,]
            lambda_damping: Damping parameter
        
        Returns:
            delta_q: Joint angle increments [5,]
            status: Execution status code
        """
        q = np.asarray(q, dtype=np.float32)
        error = np.asarray(error, dtype=np.float32)
        sin_q = np.sin(q)
        cos_q = np.cos(q)
        
        try:
            # Write sin(q) values
            # sin0=0x10, sin1=0x18, sin2=0x20, sin3=0x28, sin4=0x30
            sin_offsets = [0x10, 0x18, 0x20, 0x28, 0x30]
            for i in range(5):
                self.solver_ip.mmio.write(sin_offsets[i], 
                    int.from_bytes(sin_q[i].tobytes(), 'little'))
            
            # Write cos(q) values
            # cos0=0x38, cos1=0x40, cos2=0x48, cos3=0x50, cos4=0x58
            cos_offsets = [0x38, 0x40, 0x48, 0x50, 0x58]
            for i in range(5):
                self.solver_ip.mmio.write(cos_offsets[i], 
                    int.from_bytes(cos_q[i].tobytes(), 'little'))
            
            # Write error vector
            # e0=0x60, e1=0x68, e2=0x70, e3=0x78, e4=0x80, e5=0x88
            error_offsets = [0x60, 0x68, 0x70, 0x78, 0x80, 0x88]
            for i in range(6):
                self.solver_ip.mmio.write(error_offsets[i], 
                    int.from_bytes(error[i].tobytes(), 'little'))
            
            # Write damping parameter lambda (0x90)
            self.solver_ip.mmio.write(0x90, 
                int.from_bytes(np.float32(lambda_damping).tobytes(), 'little'))
            
            # Start computation (write to control register 0x00)
            self.solver_ip.mmio.write(0x00, 0x01)
            
            # Poll for completion (check bit 1 of status register 0x00)
            timeout = 10000
            for _ in range(timeout):
                status_reg = self.solver_ip.mmio.read(0x00)
                if status_reg & 0x02:  # Done flag
                    break
            else:
                raise TimeoutError("Hardware solver timeout")
            
            # Read output delta vector
            # d0=0x98, d1=0xA8, d2=0xB8, d3=0xC8, d4=0xD8
            delta = np.zeros(5, dtype=np.float32)
            delta_offsets = [0x98, 0xA8, 0xB8, 0xC8, 0xD8]
            for i in range(5):
                raw = self.solver_ip.mmio.read(delta_offsets[i])
                delta[i] = np.frombuffer(
                    raw.to_bytes(4, 'little', signed=False), 
                    dtype=np.float32)[0]
            
            # Read status code
            status = self.solver_ip.mmio.read(0xE8)
            
        except Exception as e:
            print(f"⚠ Hardware access error: {e}")
            return np.zeros(5), 1
        
        return delta, status


class HWSolver:
    """Hardware-accelerated LM inverse kinematics solver"""
    
    def __init__(self, robot, hw_iterator: Optional[HWSolverIterator] = None):
        """
        Initialize hardware solver
        
        Args:
            robot: Robot kinematics model
            hw_iterator: Hardware iterator (if None, hardware acceleration is disabled)
        """
        self.robot = robot
        self.hw_iterator = hw_iterator
        self.hw_available = hw_iterator is not None
    
    def solve(self, T_goal: np.ndarray, q0: Optional[np.ndarray] = None,
              ilimit: int = 5000, slimit: int = 250, tol: float = 1e-5,
              mask: Optional[np.ndarray] = None, k: float = 0.1,
              method: str = "wampler", verbose: bool = True) -> IKResult:
        """
        Solve inverse kinematics using hardware accelerator
        
        Args:
            T_goal: Target end-effector pose [4, 4]
            q0: Initial joint angles [5,], defaults to zero vector
            ilimit: Maximum number of iterations
            slimit: Maximum number of line search attempts
            tol: Convergence threshold (error norm)
            mask: Error mask [6,] for weighting different dimensions
            k: Initial damping coefficient
            method: Damping method ('wampler', 'sugihara')
            verbose: Whether to print detailed information
        
        Returns:
            IKResult object
        """
        # Parameter initialization
        if q0 is None:
            q0 = np.zeros(5, dtype=np.float32)
        else:
            q0 = np.asarray(q0, dtype=np.float32)
        
        if mask is None:
            mask = np.ones(6, dtype=np.float32)
        else:
            mask = np.asarray(mask, dtype=np.float32)
        
        q = q0.copy()
        lambda_val = k
        start_time = time.time()
        current_norm = float('inf')
        
        if verbose:
            print(f"\n{'='*70}")
            print(f"[Hardware Solver] Starting inverse kinematics solution")
            print(f"{'='*70}")
            print(f"  Initial joint angles: {q0}")
            print(f"  Maximum iterations: {ilimit}")
            print(f"  Convergence threshold: {tol}, Initial damping: {k}, Method: {method}")
            print(f"  Hardware acceleration: {'✓ Available' if self.hw_available else '✗ Unavailable'}")
        
        # Main iteration loop
        for iteration in range(ilimit):
            T_current = self.robot.fkine(q)
            error = angle_axis_error(T_current, T_goal).astype(np.float32)
            
            # Calculate weighted error E (for Sugihara damping)
            # IK_LM: E = 0.5 * e.T * We * e
            # We = diag(mask)
            E = 0.5 * np.sum((error ** 2) * mask)
            
            # Calculate convergence criterion (using weighted error)
            current_norm = np.sqrt(np.sum((error ** 2) * mask))

            # Calculate effective damping (Effective Lambda)
            if method == "sugihara":
                # Sugihara: λ_eff = E + λ
                lambda_eff = E + k
            else:
                # Wampler: λ_eff = λ
                lambda_eff = k

            # 定期输出
            if verbose and (iteration % 100 == 0 or iteration < 3):
                print(f"  迭代 {iteration:4d}: 误差={current_norm:.6e}, E={E:.6e}, λ_eff={lambda_eff:.4e}")
            
            # 收敛判断 (使用能量 E，与 roboticstoolbox 一致)
            if E < tol:
                total_time = time.time() - start_time
                # 归一化关节角
                q = (q + np.pi) % (2 * np.pi) - np.pi
                
                if verbose:
                    print(f"\n✓ 收敛成功 (迭代 {iteration}, 耗时 {total_time:.4f}s)")
                return IKResult(
                    success=True,
                    q=q.astype(np.float32),
                    iterations=iteration,
                    total_time=total_time,
                    final_error=E,  # 使用能量E，与收敛判断一致
                    reason=f'Converged at iteration {iteration}'
                )
            
            # 调用硬件求解单步
            if self.hw_available:
                # HLS 计算 J^T * e_input
                # 我们希望计算 J^T * We * e
                # 所以 e_input = We * e = mask * error
                error_weighted = error * mask
                
                delta_q, status = self.hw_iterator.solve_step(q, error_weighted, lambda_eff)
                
                if status != 0:
                    if verbose:
                        print(f"  ⚠ 迭代 {iteration} 硬件返回错误码 {status}")
                    # 硬件错误时尝试减小步长或退出？这里暂时继续
                    continue
            else:
                if verbose:
                    print(f"✗ 硬件不可用，无法继续求解")
                break
            q += delta_q
            

        
        # 超过迭代次数
        total_time = time.time() - start_time
        # 计算最终的 E
        T_current = self.robot.fkine(q)
        error = angle_axis_error(T_current, T_goal).astype(np.float32)
        E = 0.5 * np.sum((error ** 2) * mask)
        
        if verbose:
            print(f"\n✗ 未收敛 (达到最大迭代次数 {ilimit}，耗时 {total_time:.4f}s)")
        
        return IKResult(
            success=False,
            q=q.astype(np.float32),
            iterations=ilimit,
            total_time=total_time,
            final_error=E,  # 使用能量E，与收敛判断一致
            reason=f'Max iterations ({ilimit}) exceeded'
        )


def create_hw_solver(robot, bitstream_path: str = 'design_3.bit', 
                     base_addr: Optional[int] = None,
                     solver_name: str = 'xf_solver_lm_so101_0') -> Optional[HWSolver]:
    """
    工厂函数：创建硬件加速求解器
    
    Args:
        robot: 机器人运动学模型
        bitstream_path: FPGA 比特流文件路径
        base_addr: 求解器基地址 (可选，用于多求解器配置)
        solver_name: 求解器IP核的名称 (默认: 'xf_solver_lm_so101_0')
    
    Returns:
        HWSolver 实例，如果硬件不可用则返回禁用的求解器
    """
    try:
        from pynq import Overlay, MMIO
        overlay = Overlay(bitstream_path)
        
        # 尝试从 overlay 中获取求解器IP核
        try:
            solver_ip = getattr(overlay, solver_name)
            print(f"✓ 硬件求解器已初始化 (基址: 0x{solver_ip.mmio.base_addr:08x}, 核名: {solver_name})")
        except AttributeError:
            # 如果使用 base_addr，直接创建 MMIO 对象访问该地址
            if base_addr is not None:
                print(f"⚠ 未找到求解器IP核 {solver_name}，使用直接MMIO访问基地址 0x{base_addr:08x}")
                # 创建一个简单的对象来代表 MMIO 接口
                class DirectMMIO:
                    def __init__(self, base_addr):
                        self.base_addr = base_addr
                        self.mmio = MMIO(base_addr, 0x1000)  # 映射 4KB 空间
                    
                    def read(self, offset):
                        return self.mmio.read(offset)
                    
                    def write(self, offset, value):
                        return self.mmio.write(offset, value)
                
                solver_ip = DirectMMIO(base_addr)
                print(f"✓ 直接MMIO求解器已初始化 (基址: 0x{base_addr:08x})")
            else:
                raise
        
        hw_iter = HWSolverIterator(solver_ip, robot)
        solver = HWSolver(robot, hw_iter)
        return solver
    except Exception as e:
        print(f"⚠ 硬件加载失败: {e}")
        print(f"✓ 创建禁用模式的求解器 (仅用于测试)")
        return HWSolver(robot, hw_iterator=None)

