import serial
import time


class FTServo:
    HEADER = [0xFF, 0xFF]

    # 指令码定义
    INST_PING = 0x01
    INST_READ = 0x02
    INST_WRITE = 0x03
    INST_REG_WRITE = 0x04
    INST_ACTION = 0x05
    INST_SYNC_READ = 0x82
    INST_SYNC_WRITE = 0x83
    INST_RECOVERY = 0x06
    INST_RESET = 0x0A

    BROADCAST_ID = 0xFE

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)

    # ------------------------------
    # 通用函数
    # ------------------------------
    def checksum(self, data):
        """计算校验和"""
        return (~sum(data)) & 0xFF

    def send_packet(self, packet):
        """发送指令包"""
        if not self.ser.is_open:
            self.ser.open()
        self.ser.write(bytes(packet))

    def build_packet(self, ID, instruction, params=None):
        """组包"""
        if params is None:
            params = []
        length = len(params) + 2
        body = [ID, length, instruction] + params
        chk = self.checksum(body)
        return self.HEADER + body + [chk]

    def read_response(self):
        """读取并解析舵机应答包"""
        header = self.ser.read(2)
        if header != b'\xff\xff':
            return None

        id_byte = self.ser.read(1)
        length_byte = self.ser.read(1)
        if len(id_byte) == 0 or len(length_byte) == 0:
            return None

        sid = id_byte[0]
        length = length_byte[0]
        body = self.ser.read(length)
        if len(body) != length:
            return None

        error = body[0]
        params = list(body[1:-1])
        checksum = body[-1]
        calc_chk = self.checksum([sid, length] + list(body[:-1]))
        valid = (calc_chk == checksum)

        return {
            "id": sid,
            "length": length,
            "error": error,
            "params": params,
            "checksum": checksum,
            "valid": valid,
        }

    # ------------------------------
    # 基本命令
    # ------------------------------
    def ping(self, ID):
        pkt = self.build_packet(ID, self.INST_PING)
        self.send_packet(pkt)
        return self.read_response()

    def read_data(self, ID, start_addr, length):     
        pkt = self.build_packet(ID, self.INST_READ, [start_addr, length])
        self.send_packet(pkt)
        return self.read_response()

    def write_data(self, ID, start_addr, data_bytes):
        params = [start_addr] + list(data_bytes)
        pkt = self.build_packet(ID, self.INST_WRITE, params)
        self.send_packet(pkt)
        return self.read_response()

    def reg_write(self, ID, start_addr, data_bytes):
        params = [start_addr] + list(data_bytes)
        pkt = self.build_packet(ID, self.INST_REG_WRITE, params)
        self.send_packet(pkt)
        return self.read_response()

    def action(self):
        pkt = self.build_packet(self.BROADCAST_ID, self.INST_ACTION)
        self.send_packet(pkt)

    def sync_write(self, start_addr, data_len, servo_data_dict):
        params = [start_addr, data_len]
        for sid, vals in servo_data_dict.items():
            params.append(sid)
            params.extend(vals)
        pkt = self.build_packet(self.BROADCAST_ID, self.INST_SYNC_WRITE, params)
        self.send_packet(pkt)

    def sync_read(self, start_addr, data_len, ids):
        """
        同步读取多个舵机的寄存器内容
        :param start_addr: 起始地址
        :param data_len: 读取长度
        :param ids: 要读取的舵机ID列表
        """
        params = [start_addr, data_len] + ids
        pkt = self.build_packet(self.BROADCAST_ID, self.INST_SYNC_READ, params)
        self.send_packet(pkt)

        responses = {}
        for _ in ids:
            resp = self.read_response()
            if resp and resp["valid"] and resp["error"] == 0:
                responses[resp["id"]] = resp["params"]
        return responses

    def recovery(self, ID):
        pkt = self.build_packet(ID, self.INST_RECOVERY)
        self.send_packet(pkt)
        return self.read_response()

    def reset(self, ID):
        pkt = self.build_packet(ID, self.INST_RESET)
        self.send_packet(pkt)
        return self.read_response()

    def close(self):
        self.ser.close()


# ------------------------------
# 主函数测试：监控1~6舵机位置
# ------------------------------
if __name__ == "__main__":
    servo = FTServo("/dev/ttyACM0", 1000000)

    print("==== 测试 PING ====")
    for i in range(1, 7):
        resp = servo.ping(i)
        if resp and resp["valid"]:
            print(f"舵机{i} 在线 (错误码={resp['error']})")
        else:
            print(f"舵机{i} 无响应")

    print("\n==== 同步读取 1-6 号舵机位置 ====")
    try:
        while True:
            responses = servo.sync_read(0x38, 2, [1, 2, 3, 4, 5, 6])
            if responses:
                line = []
                for sid, params in responses.items():
                    pos = params[0] + (params[1] << 8)
                    line.append(f"{sid}:{pos:4d}")
                print(" ".join(line))
            else:
                print("无同步读响应")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n退出监控。")

    servo.close()
