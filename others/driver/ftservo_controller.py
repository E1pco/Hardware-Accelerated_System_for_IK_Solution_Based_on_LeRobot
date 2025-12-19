import json
import time
import os
import sys
import numpy as np
# 添加父目录到路径以支持相对导入
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 相对导入
from .ftservo_driver import FTServo




class ServoController:
    def __init__(self, port="/dev/ttyACM0", baudrate=1_000_000, config_path="./servo_config.json"):
        self.servo = FTServo(port, baudrate)
        with open(config_path, "r", encoding="utf-8") as f:
            self.config = json.load(f)

        self.id_map = {v["id"]: name for name, v in self.config.items()}
        # home_pose 是舵机活动范围的中点：(range_max + range_min) / 2
        # 这是舵机的零位参考点，用于 IK 计算：q = gear_sign * (steps - home_pose) / counts_per_rad
        # homing_offset 仅用于机械臂的校准和零位定义，不影响 IK 计算
        self.home_pose = {}
        for name, cfg in self.config.items():
            if name == "gripper":
                home_position = cfg.get("range_min", 0)
            else:
                home_position = (cfg["range_max"] + cfg["range_min"]) // 2
            
            self.home_pose[name] = home_position

        print("✅ 已加载舵机配置:")
        for name, cfg in self.config.items():
            print(f"  {cfg['id']}: {name} (home_pose={self.home_pose[name]}, range={cfg['range_min']}~{cfg['range_max']})")

    # -------------------------
    # 基础功能
    # -------------------------
    def checksum(self, data):
        return (~sum(data)) & 0xFF

    def limit_position(self, name, target_pos):
        """限位保护"""
        cfg = self.config[name]
        minv, maxv = cfg["range_min"], cfg["range_max"]
        limited = max(min(target_pos, maxv), minv)
        if limited != target_pos:
            print(f"⚠️ {name} 超出限位 {minv}~{maxv}，修正为 {limited}")
        return limited

    def get_home_position(self, name):
        return self.home_pose[name]


    # -------------------------
    # 动作控制
    # -------------------------
    def move_servo(self, name, target_pos, speed=1000):
        """移动单个舵机"""
        cfg = self.config[name]
        sid = cfg["id"]
        limited_pos = self.limit_position(name, target_pos)

        # 数据格式：位置(2B) + 时间(2B) + 速度(2B)
        data = [
            limited_pos & 0xFF, (limited_pos >> 8) & 0xFF,
            0x00, 0x00,
            speed & 0xFF, (speed >> 8) & 0xFF
        ]
        resp = self.servo.write_data(sid, 0x2A, data)
        if resp and resp["valid"] and resp["error"] == 0:
            print(f"✅ {name}({sid}) → {limited_pos}")
        else:
            print(f"❌ {name}({sid}) 通信失败: {resp}")

    def move_group(self, targets_dict):
        """同步控制多个舵机"""
        servo_data = {}
        for name, pos in targets_dict.items():
            cfg = self.config[name]
            sid = cfg["id"]
            limited_pos = self.limit_position(name, pos)
            servo_data[sid] = [
                limited_pos & 0xFF, (limited_pos >> 8) & 0xFF,
                0x00, 0x00,
                0xE8, 0x03  # speed = 1000
            ]
        self.servo.sync_write(0x2A, 6, servo_data)
        print(f"🚀 同步移动 {len(targets_dict)} 个舵机完成")

    # -------------------------
    # 中位与缓动控制
    # -------------------------
    def move_to_home(self, name):
        """单个舵机立即回中位"""
        home = self.get_home_position(name)
        print(f"↩️ {name} 回中位 {home}")
        self.move_servo(name, home)

    def move_all_home(self):
        """全部舵机立即同步回中位"""
        servo_data = {}
        for name, cfg in self.config.items():
            sid = cfg["id"]
            home = self.home_pose[name]
            servo_data[sid] = [
                home & 0xFF, (home >> 8) & 0xFF,
                0x00, 0x00,
                0xE8, 0x03
            ]
        self.servo.sync_write(0x2A, 6, servo_data)
        print("🏠 全部舵机同步回中位完成")

    def soft_move_to_home(self, step_count=10, interval=0.15):
        """
        软启动（缓动）回中位：
        通过多步插值平滑过渡，避免瞬间加速冲击。
        """
        print("🌀 开始软启动回中位...")

        # 读取当前舵机位置
        ids = [cfg["id"] for cfg in self.config.values()]
        responses = self.servo.sync_read(0x38, 2, ids)
        current_pos = {}
        for name, cfg in self.config.items():
            sid = cfg["id"]
            if sid in responses:
                params = responses[sid]
                current_pos[name] = params[0] + (params[1] << 8)
            else:
                current_pos[name] = self.get_home_position(name)  # 若无响应，直接设为home


        # 插值逐步移动
        for step in range(1, step_count + 1):
            servo_data = {}
            for name, cfg in self.config.items():
                sid = cfg["id"]
                start = current_pos[name]
                end = self.home_pose[name]
                interp = int(start + (end - start) * (step / step_count))
                servo_data[sid] = [
                    interp & 0xFF, (interp >> 8) & 0xFF,
                    0x00, 0x00,
                    0xE8, 0x03  # speed=1000
                ]
            self.servo.sync_write(0x2A, 6, servo_data)
            print(f"  Step {step}/{step_count}")
            time.sleep(interval)

        print("✅ 所有舵机已平滑回中位")
    def soft_move_to_pose(self, target_dict, step_count=15, interval=0.15):
        """
        平滑移动到指定目标姿态
        target_dict: { "joint_name": target_position, ... }
        """
        print("🌀 开始软启动移动到目标姿态...")

        # 1️⃣ 获取所有舵机ID
        ids = [cfg["id"] for cfg in self.config.values()]

        # 2️⃣ 读取当前位置
        responses = self.servo.sync_read(0x38, 2, ids)
        current_pos = {}
        for name, cfg in self.config.items():
            sid = cfg["id"]
            if sid in responses:
                params = responses[sid]
                current_pos[name] = params[0] + (params[1] << 8)
            else:
                current_pos[name] = self.get_home_position(name)  # 无响应则取home
                print(f"⚠️ {name} 无反馈，默认home={current_pos[name]}")

        # 3️⃣ 限位保护 + 目标准备
        target_pos = {}
        for name, pos in target_dict.items():
            if name not in self.config:
                print(f"⚠️ 未知舵机: {name}")
                continue
            target_pos[name] = self.limit_position(name, int(pos))

        # 4️⃣ 插值并缓动发送
        for step in range(1, step_count + 1):
            servo_data = {}
            for name, cfg in self.config.items():
                sid = cfg["id"]
                start = current_pos[name]
                end = target_pos.get(name, start)  # 未指定的保持原位
                interp = int(start + (end - start) * (step / step_count))
                servo_data[sid] = [
                    interp & 0xFF, (interp >> 8) & 0xFF,
                    0x00, 0x00,
                    0xE8, 0x03  # 速度 = 1000
                ]
            self.servo.sync_write(0x2A, 6, servo_data)
            print(f"  Step {step}/{step_count}")
            time.sleep(interval)

        print("✅ 目标姿态已平滑到位")


    def fast_move_to_pose(self, target_dict, speed=1000):
        """
        🚀 非平滑同步运动（直接下发目标步数，支持自定义速度）
        target_dict: { "joint_name": target_position, ... }
        speed: int 或 dict
            - 若为 int：所有舵机使用同一速度（如 800~2000）
            - 若为 dict：可为不同舵机指定不同速度，如 {"elbow_flex": 600, "wrist_roll": 1200}
        """
        servo_data = {}

        for name, pos in target_dict.items():
            if name not in self.config:
                print(f"⚠️ 未知舵机: {name}")
                continue

            cfg = self.config[name]
            sid = cfg["id"]
            limited_pos = self.limit_position(name, int(pos))

            # --- 解析速度 ---
            if isinstance(speed, dict):
                spd = int(speed.get(name, 1000))  # 若未指定，默认1000
            else:
                spd = int(speed)

            spd = max(200, min(spd, 4095))  # 限制速度范围

            servo_data[sid] = [
                limited_pos & 0xFF, (limited_pos >> 8) & 0xFF,
                0x00, 0x00,
                spd & 0xFF, (spd >> 8) & 0xFF
            ]

        self.servo.sync_write(0x2A, 6, servo_data)
        print(f"🚀 Fast move ({'per-joint' if isinstance(speed, dict) else 'global'}) speed mode")

    # -------------------------
    # 读取舵机状态
    # -------------------------
    def read_servo_positions(self, joint_names=None, verbose=False):
        """
        读取指定关节的舵机步数
        
        Parameters
        ----------
        joint_names : list of str, optional
            要读取的关节名称列表。如果为 None，则读取所有配置的关节
        verbose : bool
            是否打印详细信息（默认 False）
        
        Returns
        -------
        positions : dict
            舵机步数字典 {"joint_name": position_steps}
        """
        if joint_names is None:
            joint_names = list(self.config.keys())
        
        # 获取所有关节的 ID
        ids = [self.config[name]["id"] for name in joint_names]
        
        # 同步读取舵机位置
        resp = self.servo.sync_read(0x38, 2, ids)
        
        positions = {}
        if verbose:
            print("\n📡 舵机步数：")
        
        for name in joint_names:
            sid = self.config[name]["id"]
            cur_pos = resp.get(sid, [0, 0])
            current = cur_pos[0] + (cur_pos[1] << 8)
            positions[name] = current
            
            if verbose:
                print(f"  {name:15s}: {current:4d}")
        
        return positions
    
    def read_single_position(self, name):
        """
        读取单个舵机的步数
        
        Parameters
        ----------
        name : str
            关节名称
        
        Returns
        -------
        int
            舵机步数
        """
        positions = self.read_servo_positions([name])
        return positions[name]

    # -------------------------
    # 监控功能
    # -------------------------
    def monitor_positions(self, ids, interval=0.3):
        """循环监控舵机位置"""
        try:
            while True:
                responses = self.servo.sync_read(0x38, 2, ids)
                if responses:
                    line = []
                    for sid, params in responses.items():
                        name = self.id_map.get(sid, f"ID{sid}")
                        pos = params[0] + (params[1] << 8)
                        line.append(f"{name}:{pos:4d}")
                    print(" ".join(line))
                else:
                    print("❌ 无同步读响应")
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n🛑 停止监控。")

    def close(self):
        self.servo.close()
if __name__ == "__main__":
    controller = ServoController("/dev/ttyACM0", 1000000, "left_arm.json")

    # 1️⃣ 平滑移动到指定目标姿态
    target_pose = {
        "shoulder_pan": 2096,
        "shoulder_lift": 1983,
        "elbow_flex": 2100,
        "wrist_flex": 1954,
        "wrist_roll": 2048,
        "gripper": 2037
    }

    # controller.soft_move_to_pose(target_pose, step_count=10, interval=0.1)
    controller.move_all_home()
    # 2️⃣ 完成后实时监控
    time.sleep(1)
    controller.monitor_positions([1, 2, 3, 4, 5, 6])

    controller.close()