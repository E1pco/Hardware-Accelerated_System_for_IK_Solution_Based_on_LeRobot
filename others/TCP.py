#!/usr/bin/env python3
"""
Joycon 手柄数据采集与TCP发送程序
功能：读取手柄位姿和夹爪数据，编码后通过TCP网口发送到 PYNQ 板卡
"""

import socket
import time
import struct
import math
import argparse
import sys
import os

# 添加 joycon_robotics 目录到 Python 路径
joycon_path = os.path.join(os.path.dirname(__file__), 'joycon_robotics')
sys.path.insert(0, joycon_path)

from joyconrobotics import JoyconRobotics

class JoyconDataCodec:
    """Joycon 数据编解码器"""
    
    FRAME_HEADER = b'\xAA\x55'
    FRAME_TAIL = b'\x0D\x0A'
    
    def encode_position(self, value):
        """编码位置数据（3字节）：符号位 + 整数部分 + 小数部分"""
        if value >= 0:
            sign_byte = b'\x00'
            abs_value = value
        else:
            sign_byte = b'\x01'
            abs_value = -value
        
        int_part = int(abs_value)
        frac_part = abs_value - int_part
        
        # 限制范围
        if int_part > 255:
            int_part = 255
            frac_part = 0.996
        
        int_byte = struct.pack('B', int_part)
        frac_byte = struct.pack('B', int(frac_part * 255))
        
        return sign_byte + int_byte + frac_byte
    
    def encode_angle(self, radian):
        """编码角度数据（2字节）：符号位 + 角度值（度）"""
        degree = radian * 180.0 / math.pi
        
        if degree >= 0:
            sign_byte = b'\x00'
            abs_degree = degree
        else:
            sign_byte = b'\x01'
            abs_degree = -degree
        
        abs_degree = min(180, abs_degree)
        degree_byte = struct.pack('B', int(abs_degree))
        
        return sign_byte + degree_byte
    
    def encode_gripper(self, gripper_value):
        """编码夹爪数据（1字节）：0-255"""
        gripper_byte = struct.pack('B', int(gripper_value))
        return gripper_byte
    
    def encode_buttons(self, button_states):
        """编码按钮状态（1字节）：将多个按钮状态打包到一个字节
        
        参数:
            button_states: dict 包含按钮状态的字典
                - 'x': X按钮 (bit 0)
                - 'home': Home按钮 (bit 1)
                - 'plus': Plus按钮 (bit 2)
                - 'minus': Minus按钮 (bit 3)
                - 'zr': ZR按钮 (bit 4)
                - 'r': R按钮 (bit 5)
                - 'b': B按钮 (bit 6)
                - bit 7: 保留
        """
        button_byte = 0
        if button_states.get('x', False):
            button_byte |= (1 << 0)
        if button_states.get('home', False):
            button_byte |= (1 << 1)
        if button_states.get('plus', False):
            button_byte |= (1 << 2)
        if button_states.get('minus', False):
            button_byte |= (1 << 3)
        if button_states.get('zr', False):
            button_byte |= (1 << 4)
        if button_states.get('r', False):
            button_byte |= (1 << 5)
        if button_states.get('b', False):
            button_byte |= (1 << 6)
        
        return struct.pack('B', button_byte)
    
    def encode_joycon_data(self, pose, gripper, button_states):
        """
        编码完整的 Joycon 数据
        
        参数:
            pose: 位姿数据 [x, y, z, rx, ry, rz]
            gripper: 夹爪值 0-255
            button_states: 按钮状态字典
        """
        
        # 编码位置（X, Y, Z）
        pos_bytes = b''
        for i in range(3):
            pos_bytes += self.encode_position(pose[i])
        
        # 编码角度（Rx, Ry, Rz）
        angle_bytes = b''
        for i in range(3, 6):
            angle_bytes += self.encode_angle(pose[i])
        
        # 编码夹爪
        gripper_byte = self.encode_gripper(gripper)
        
        # 编码按钮状态
        button_byte = self.encode_buttons(button_states)
        
        # 组合数据
        data_bytes = pos_bytes + angle_bytes + gripper_byte + button_byte
        length_byte = struct.pack('B', len(data_bytes))
        
        # 构造完整帧
        frame = self.FRAME_HEADER + length_byte + data_bytes + self.FRAME_TAIL
        
        return frame
    
    def decode_frame(self, frame):
        """解码数据帧（用于调试）"""
        if len(frame) != 22:
            return None
        
        if frame[0:2] != self.FRAME_HEADER or frame[-2:] != self.FRAME_TAIL:
            return None
        
        data_bytes = frame[3:-2]
        
        if len(data_bytes) != 17:
            return None
        
        # 解码位置
        pose_data = []
        for i in range(3):
            sign = 1 if data_bytes[i*3] == 0 else -1
            int_part = data_bytes[i*3 + 1]
            frac_part = data_bytes[i*3 + 2] / 255.0
            pose_data.append(sign * (int_part + frac_part))
        
        # 解码角度
        for i in range(3):
            sign = 1 if data_bytes[9 + i*2] == 0 else -1
            degree = data_bytes[9 + i*2 + 1]
            radian = sign * (degree * math.pi / 180.0)
            pose_data.append(radian)
        
        # 解码夹爪
        gripper = data_bytes[15]
        
        # 解码按钮状态
        button_byte = data_bytes[16]
        button_states = {
            'x': bool(button_byte & (1 << 0)),
            'home': bool(button_byte & (1 << 1)),
            'plus': bool(button_byte & (1 << 2)),
            'minus': bool(button_byte & (1 << 3)),
            'zr': bool(button_byte & (1 << 4)),
            'r': bool(button_byte & (1 << 5)),
            'b': bool(button_byte & (1 << 6))
        }
        
        return pose_data, gripper, button_states


class TCPSender:
    """TCP 数据发送器 - 支持左右两个手柄"""
    
    def __init__(self, host='192.168.2.99', port_right=9091, port_left=9092):
        """
        初始化TCP发送器
        
        参数:
            host: PYNQ的IP地址 (默认: 192.168.2.99)
            port_right: 右手柄TCP端口号 (默认: 9091)
            port_left: 左手柄TCP端口号 (默认: 9092)
        """
        self.host = host
        self.port_right = port_right
        self.port_left = port_left
        self.socket_right = None
        self.socket_left = None
        self.connected_right = False
        self.connected_left = False
        self.joycon_right = None
        self.joycon_left = None
        self.codec = JoyconDataCodec()
        
        # 夹爪状态（0-255）
        self.gripper_right_value = 0
        self.gripper_left_value = 0
        # 按钮状态跟踪（防止重复触发）
        self.prev_r_right = False
        self.prev_sr_right = False
        self.prev_r_left = False
        self.prev_sr_left = False
    
    def connect_joycon_right(self):
        """连接右 Joycon 手柄"""
        print("正在连接右 Joycon...")
        try:
            self.joycon_right = JoyconRobotics('right')
            print("✓ 右 Joycon 连接成功")
            return True
        except Exception as e:
            print(f"✗ 右 Joycon 连接失败: {e}")
            return False
    
    def connect_joycon_left(self):
        """连接左 Joycon 手柄"""
        print("正在连接左 Joycon...")
        try:
            self.joycon_left = JoyconRobotics('left')
            print("✓ 左 Joycon 连接成功")
            return True
        except Exception as e:
            print(f"✗ 左 Joycon 连接失败: {e}")
            return False
    
    def connect_right(self):
        """连接到PYNQ的右手柄TCP服务器"""
        print(f"正在连接到 PYNQ 右手柄 ({self.host}:{self.port_right})...")
        try:
            self.socket_right = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 优化 TCP 延迟：禁用 Nagle 算法，立即发送小包
            self.socket_right.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # 设置较小的发送缓冲区，减少数据堆积
            self.socket_right.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256)
            self.socket_right.connect((self.host, self.port_right))
            self.connected_right = True
            print(f"✓ 右手柄TCP连接成功（低延迟模式）")
            return True
        except ConnectionRefusedError:
            print(f"✗ 右手柄连接被拒绝，请确保PYNQ上的TCP服务器正在运行")
            return False
        except socket.timeout:
            print(f"✗ 右手柄连接超时")
            return False
        except Exception as e:
            print(f"✗ 右手柄TCP连接失败: {e}")
            return False
    
    def connect_left(self):
        """连接到PYNQ的左手柄TCP服务器"""
        print(f"正在连接到 PYNQ 左手柄 ({self.host}:{self.port_left})...")
        try:
            self.socket_left = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # 优化 TCP 延迟：禁用 Nagle 算法，立即发送小包
            self.socket_left.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # 设置较小的发送缓冲区，减少数据堆积
            self.socket_left.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 256)
            self.socket_left.connect((self.host, self.port_left))
            self.connected_left = True
            print(f"✓ 左手柄TCP连接成功（低延迟模式）")
            return True
        except ConnectionRefusedError:
            print(f"✗ 左手柄连接被拒绝，请确保PYNQ上的TCP服务器正在运行")
            return False
        except socket.timeout:
            print(f"✗ 左手柄连接超时")
            return False
        except Exception as e:
            print(f"✗ 左手柄TCP连接失败: {e}")
            return False
    
    def connect(self):
        """连接两个手柄到PYNQ的TCP服务器"""
        success_right = self.connect_right()
        success_left = self.connect_left()
        return success_right and success_left
    
    def send_data(self, data_bytes, hand='right'):
        """
        发送数据
        
        参数:
            data_bytes: 要发送的字节数据
            hand: 手柄选择 ('right' 或 'left')
        
        返回:
            bool: 发送是否成功
        """
        if hand == 'right':
            socket_obj = self.socket_right
            connected = self.connected_right
        else:
            socket_obj = self.socket_left
            connected = self.connected_left
        
        if not connected or not socket_obj:
            print(f"✗ {hand.upper()}手柄TCP未连接")
            return False
        
        try:
            socket_obj.sendall(data_bytes)
            return True
        except BrokenPipeError:
            print(f"✗ {hand.upper()}手柄连接已断开")
            if hand == 'right':
                self.connected_right = False
            else:
                self.connected_left = False
            return False
        except Exception as e:
            print(f"✗ {hand.upper()}手柄数据发送错误: {e}")
            return False
    
    def receive_data(self, buffer_size=1024, hand='right'):
        """
        接收数据（可选功能）
        
        参数:
            buffer_size: 接收缓冲区大小
            hand: 手柄选择 ('right' 或 'left')
        
        返回:
            bytes: 接收到的数据，失败返回None
        """
        if hand == 'right':
            socket_obj = self.socket_right
            connected = self.connected_right
        else:
            socket_obj = self.socket_left
            connected = self.connected_left
        
        if not connected or not socket_obj:
            return None
        
        try:
            data = socket_obj.recv(buffer_size)
            return data
        except Exception as e:
            print(f"✗ {hand.upper()}手柄数据接收错误: {e}")
            return None
    
    def close(self):
        """关闭TCP连接并释放端口"""
        # 关闭右手柄连接
        if self.socket_right:
            try:
                # 设置SO_REUSEADDR允许端口快速释放
                self.socket_right.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket_right.close()
                print("✓ 右手柄TCP连接已关闭")
            except Exception as e:
                print(f"✗ 关闭右手柄连接时出错: {e}")
            finally:
                self.connected_right = False
                self.socket_right = None
        
        # 关闭左手柄连接
        if self.socket_left:
            try:
                # 设置SO_REUSEADDR允许端口快速释放
                self.socket_left.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket_left.close()
                print("✓ 左手柄TCP连接已关闭")
            except Exception as e:
                print(f"✗ 关闭左手柄连接时出错: {e}")
            finally:
                self.connected_left = False
                self.socket_left = None
    
    def send_joycon_data_right(self):
        """读取右手柄数据并通过TCP发送"""
        if not self.joycon_right or not self.connected_right:
            return False, None, None, None, None
        
        try:
            pose, gripper, control_button = self.joycon_right.get_control()
            
            final_pose = pose
            
            # 处理按钮状态
            if control_button is None or not isinstance(control_button, dict):
                button_states = {
                    'x': False, 'home': False, 'plus': False, 'minus': False,
                    'zr': False, 'r': False, 'b': False
                }
            else:
                button_states = control_button.copy()
            
            # 右手柄夹爪控制逻辑 - 使用直接按钮检查（参考joycon_ik_control_py.py）
            # R 按钮: 张开夹爪 (递增)
            # ZR 按钮: 收紧夹爪 (递减)
            if self.joycon_right.button.r == 1:
                # 边沿检测：从未按到按下
                if not self.prev_r_right:
                    self.gripper_right_value = min(255, self.gripper_right_value + 15)
                self.prev_r_right = True
            else:
                self.prev_r_right = False
            
            if self.joycon_right.button.zr == 1:
                # 边沿检测：从未按到按下
                if not self.prev_sr_right:
                    self.gripper_right_value = max(0, self.gripper_right_value - 15)
                self.prev_sr_right = True
            else:
                self.prev_sr_right = False
            
            gripper_byte = int(self.gripper_right_value)
            
            # 编码数据
            frame = self.codec.encode_joycon_data(final_pose, gripper_byte, button_states)
            
            # 发送数据
            self.socket_right.sendall(frame)
            
            return True, final_pose, gripper_byte, button_states, frame
        
        except Exception as e:
            return False, None, None, None, None
    
    def send_joycon_data_left(self):
        """读取左手柄数据并通过TCP发送"""
        if not self.joycon_left or not self.connected_left:
            return False, None, None, None, None
        
        try:
            pose, gripper, control_button = self.joycon_left.get_control()
            
            final_pose = pose
            
            # 处理按钮状态
            if control_button is None or not isinstance(control_button, dict):
                button_states = {
                    'x': False, 'home': False, 'plus': False, 'minus': False,
                    'zr': False, 'r': False, 'b': False
                }
            else:
                button_states = control_button.copy()
            
            # 左手柄夹爪控制逻辑 - 使用直接按钮检查（参考joycon_ik_control_py.py）
            # 注意：左 Joy-Con 使用 L 和 ZL 按钮（不是 R 和 ZR）
            # L 按钮: 张开夹爪 (递增)
            # ZL 按钮: 收紧夹爪 (递减)
            if self.joycon_left.button.l == 1:
                # 边沿检测：从未按到按下
                if not self.prev_r_left:
                    self.gripper_left_value = min(255, self.gripper_left_value + 15)
                self.prev_r_left = True
            else:
                self.prev_r_left = False
            
            if self.joycon_left.button.zl == 1:
                # 边沿检测：从未按到按下
                if not self.prev_sr_left:
                    self.gripper_left_value = max(0, self.gripper_left_value - 15)
                self.prev_sr_left = True
            else:
                self.prev_sr_left = False
            
            gripper_byte = int(self.gripper_left_value)
            
            # 编码数据
            frame = self.codec.encode_joycon_data(final_pose, gripper_byte, button_states)
            
            # 发送数据
            self.socket_left.sendall(frame)
            
            return True, final_pose, gripper_byte, button_states, frame
        
        except Exception as e:
            return False, None, None, None, None
    
    def check_exit_button(self):
        """检查退出按钮（右手X键）"""
        if not self.joycon_right:
            return False
        
        x_button = self.joycon_right.button.x == 1
        return x_button
    
    def run(self, frequency=100, show_data=True, verbose=False):
        """
        运行主循环，读取两个手柄数据并分别发送
        
        参数:
            frequency: 发送频率 Hz
            show_data: 是否显示发送的数据
            verbose: 是否显示详细信息
        """
        
        print("\n" + "="*80)
        print(" "*15 + "左右 Joycon → PYNQ (双TCP) 数据发送程序")
        print("="*80)
        print(f"目标地址: {self.host}")
        print(f"右手柄TCP: {self.port_right}")
        print(f"左手柄TCP: {self.port_left}")
        print(f"发送频率: {frequency} Hz")
        print(f"帧格式  : AA 55 + 长度 + 位姿(15B) + 夹爪(1B) + 按钮(1B) + 0D 0A")
        print("="*80)
        print("\n操作说明:")
        print("  - 移动右手柄改变右臂位姿数据（端口 {})".format(self.port_right))
        print("  - 移动左手柄改变左臂位姿数据（端口 {})".format(self.port_left))
        print("  - 右手柄 R 键: 张开右臂夹爪")
        print("  - 右手柄 ZR 键: 收紧右臂夹爪")
        print("  - 左手柄 L 键: 张开左臂夹爪")
        print("  - 左手柄 ZL 键: 收紧左臂夹爪")
        print("  - 按右手柄 X 键退出程序")
        print("="*80)
        
        # 连接两个手柄
        if not self.connect_joycon_right():
            return
        if not self.connect_joycon_left():
            if self.joycon_right:
                self.joycon_right.disconnnect()
            return
        
        # 连接到PYNQ
        if not self.connect():
            if self.joycon_right:
                self.joycon_right.disconnnect()
            if self.joycon_left:
                self.joycon_left.disconnnect()
            return
        
        print("\n✓ 系统就绪，开始发送数据...\n")
        
        sleep_time = 1.0 / frequency
        frame_count_right = 0
        frame_count_left = 0
        start_time = time.time()
        
        try:
            while True:
                loop_start = time.time()
                
                # 检查退出按钮
                if self.check_exit_button():
                    print('\n\n您按下了右手柄 X 键，程序退出')
                    break
                
                # 发送右手柄数据
                success_r, pose_r, gripper_r, button_r, frame_r = self.send_joycon_data_right()
                if success_r:
                    frame_count_right += 1
                    if show_data and verbose:
                        print(f"\n右手柄 - 帧 #{frame_count_right}")
                        print(f"  位置: X={pose_r[0]:+.4f}  Y={pose_r[1]:+.4f}  Z={pose_r[2]:+.4f}")
                        print(f"  角度: Rx={pose_r[3]*180/math.pi:+7.2f}°  Ry={pose_r[4]*180/math.pi:+7.2f}°  Rz={pose_r[5]*180/math.pi:+7.2f}°")
                
                # 发送左手柄数据
                success_l, pose_l, gripper_l, button_l, frame_l = self.send_joycon_data_left()
                if success_l:
                    frame_count_left += 1
                    if show_data and verbose:
                        print(f"左手柄 - 帧 #{frame_count_left}")
                        print(f"  位置: X={pose_l[0]:+.4f}  Y={pose_l[1]:+.4f}  Z={pose_l[2]:+.4f}")
                        print(f"  角度: Rx={pose_l[3]*180/math.pi:+7.2f}°  Ry={pose_l[4]*180/math.pi:+7.2f}°  Rz={pose_l[5]*180/math.pi:+7.2f}°")
                
                # 简洁模式输出
                if show_data and not verbose and (success_r or success_l):
                    output = ""
                    if success_r:
                        output += f"[右] pos=[{pose_r[0]:+.4f}, {pose_r[1]:+.4f}, {pose_r[2]:+.4f}] " \
                                 f"rpy=[{pose_r[3]*180/math.pi:+7.2f}°, {pose_r[4]*180/math.pi:+7.2f}°, {pose_r[5]*180/math.pi:+7.2f}°] " \
                                 f"gripper={gripper_r}/255"
                    if success_l:
                        if output:
                            output += " | "
                        output += f"[左] pos=[{pose_l[0]:+.4f}, {pose_l[1]:+.4f}, {pose_l[2]:+.4f}] " \
                                 f"rpy=[{pose_l[3]*180/math.pi:+7.2f}°, {pose_l[4]*180/math.pi:+7.2f}°, {pose_l[5]*180/math.pi:+7.2f}°] " \
                                 f"gripper={gripper_l}/255"
                    if output:
                        print(output)
                
                # 控制频率
                elapsed = time.time() - loop_start
                if elapsed < sleep_time:
                    time.sleep(sleep_time - elapsed)
        
        except KeyboardInterrupt:
            print("\n\n用户中断程序 (Ctrl+C)")
        
        except Exception as e:
            print(f"\n\n错误: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # 清理资源
            print("\n" + "="*80)
            print("正在关闭连接并释放端口...")
            print("="*80)
            
            # 断开Joycon连接
            if self.joycon_right:
                self.joycon_right.disconnnect()
                print("✓ 右 Joycon 已断开")
            if self.joycon_left:
                self.joycon_left.disconnnect()
                print("✓ 左 Joycon 已断开")
            
            # 关闭TCP连接并释放端口
            print("\n释放TCP端口...")
            self.close()
            print(f"✓ 端口 {self.port_right} 和 {self.port_left} 已释放")
            
            # 统计信息
            elapsed_time = time.time() - start_time
            if elapsed_time > 0:
                actual_freq_r = frame_count_right / elapsed_time
                actual_freq_l = frame_count_left / elapsed_time
                print(f"\n运行统计:")
                print(f"  右手柄发送帧数: {frame_count_right} (实际频率: {actual_freq_r:.1f} Hz)")
                print(f"  左手柄发送帧数: {frame_count_left} (实际频率: {actual_freq_l:.1f} Hz)")
                print(f"  总运行时间: {elapsed_time:.2f} 秒")
            
            print("="*80)


def main():
    """主函数"""
    
    parser = argparse.ArgumentParser(
        description='左右 Joycon 手柄数据采集与双TCP发送程序',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
示例:
  %(prog)s                                    # 使用默认参数
  %(prog)s -H 192.168.2.99                   # 指定PYNQ的IP地址
  %(prog)s -R 9091 -L 9092                   # 指定右左手柄TCP端口
  %(prog)s -f 50                             # 50Hz 发送频率
  %(prog)s -v                                # 详细模式
  %(prog)s -q                                # 安静模式（不显示数据）

网络配置说明:
  电脑端配置:
    - IP地址: 192.168.2.2
    - 子网掩码: 255.255.255.0
  
  PYNQ端配置:
    - IP地址: 192.168.2.99
    - Jupyter访问: http://192.168.2.99:9090/
    - 右手柄TCP端口: 9091 (默认，可通过-R修改)
    - 左手柄TCP端口: 9092 (默认，可通过-L修改)
    
手柄操作:
  - 移动右手柄改变右臂位姿数据
  - 移动左手柄改变左臂位姿数据
  - 按右手柄 X 键退出程序
  - 按键状态会实时编码发送:
    * X: 退出 (bit 0)
    * Home: 复位 (bit 1)
    * Plus: 增加速度 (bit 2)
    * Minus: 减少速度 (bit 3)
    * ZR: 夹爪收紧 (bit 4)
    * R: 夹爪松开 (bit 5)
    * B: Z轴增大 (bit 6)
        '''
    )
    
    parser.add_argument('-H', '--host', 
                        default='192.168.2.99',
                        help='PYNQ的IP地址 (默认: 192.168.2.99)')
    
    parser.add_argument('-R', '--port-right',
                        type=int,
                        default=46398,
                        help='右手柄TCP端口号 (默认: 9091)')
    
    parser.add_argument('-L', '--port-left',
                        type=int,
                        default=45511,
                        help='左手柄TCP端口号 (默认: 9092)')
    
    parser.add_argument('-f', '--frequency',
                        type=int,
                        default=10,
                        help='发送频率 Hz (默认: 50)')
    
    parser.add_argument('-v', '--verbose',
                        action='store_true',
                        help='显示详细信息')
    
    parser.add_argument('-q', '--quiet',
                        action='store_true',
                        help='安静模式（不显示数据）')
    
    args = parser.parse_args()
    
    # 创建TCP发送器
    sender = TCPSender(host=args.host, port_right=args.port_right, port_left=args.port_left)
    
    # 运行
    sender.run(
        frequency=args.frequency,
        show_data=not args.quiet,
        verbose=args.verbose
    )


if __name__ == "__main__":
    main()
