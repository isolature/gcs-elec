#!/usr/bin/env python3
"""树莓派视觉程序使用的救援车串口控制接口。"""

import glob
import os
import threading

import serial


BAUDRATE = 115200
MIN_MOVING_RPM = 20
MAX_RPM = 108


def find_stm32():
    """返回稳定的 STM32 USB 串口设备路径。"""
    candidates = (
        glob.glob("/dev/serial/by-id/*STM*")
        + glob.glob("/dev/serial/by-id/*STMicroelectronics*")
    )
    if candidates:
        return candidates[0]
    if os.path.exists("/dev/ttyACM0"):
        return "/dev/ttyACM0"
    raise FileNotFoundError(
        "没有找到 STM32。请检查 USB 数据线，并运行 ls /dev/ttyACM*"
    )


class CarControl:
    """把视觉层的动作转换成 STM32 串口协议。"""

    def __init__(self, port=None, baudrate=BAUDRATE, timeout=0.1, serial_port=None):
        self._serial = serial_port or serial.Serial(
            port=port or find_stm32(),
            baudrate=baudrate,
            timeout=timeout,
        )
        self._owns_serial = serial_port is None
        self._write_lock = threading.Lock()
        self._last_motion = None

    @property
    def serial(self):
        """供交互调试程序的后台日志线程共用串口。"""
        return self._serial

    @property
    def is_open(self):
        return bool(self._serial and self._serial.is_open)

    def _write(self, payload):
        if not self.is_open:
            raise ConnectionError("STM32 串口已关闭")
        with self._write_lock:
            self._serial.write(payload)
            self._serial.flush()
        return True

    def _send_line(self, command):
        return self._write(f"{command}\r\n".encode("ascii"))

    @staticmethod
    def _validate_rpm(name, rpm):
        if isinstance(rpm, bool) or not isinstance(rpm, int):
            raise TypeError(f"{name} RPM 必须是整数")
        if not -MAX_RPM <= rpm <= MAX_RPM:
            raise ValueError(f"{name} RPM 必须在 -{MAX_RPM}～{MAX_RPM} 之间")
        if rpm != 0 and abs(rpm) < MIN_MOVING_RPM:
            raise ValueError(
                f"{name}非零 RPM 的绝对值必须在 {MIN_MOVING_RPM}～{MAX_RPM} 之间"
            )

    def _fixed_motion(self, action):
        motion = ("AUTO", action)
        if motion == self._last_motion:
            return False
        self._send_line(f"AUTO,{action}")
        self._last_motion = motion
        return True

    def forward(self):
        return self._fixed_motion("F")

    def backward(self):
        return self._fixed_motion("B")

    def turn_left(self):
        return self._fixed_motion("L")

    def turn_right(self):
        return self._fixed_motion("R")

    def stop(self):
        """正常停车；连续重复的停车指令不会反复发送。"""
        return self._fixed_motion("0")

    def emergency_stop(self):
        """立即发送单字节 s；紧急停车不会去重。"""
        self._write(b"s")
        self._last_motion = ("AUTO", "0")
        return True

    def set_wheels(self, left_rpm, right_rpm):
        self._validate_rpm("左轮", left_rpm)
        self._validate_rpm("右轮", right_rpm)
        motion = ("WHEEL", left_rpm, right_rpm)
        if motion == self._last_motion:
            return False
        self._send_line(f"WHEEL,{left_rpm},{right_rpm}")
        self._last_motion = motion
        return True

    def drive(self, rpm, sync=False):
        """设置双轮共同目标；sync=False 表示独立 PI。"""
        self._validate_rpm("双轮", rpm)
        mode = "L" if sync else "I"
        motion = ("DRIVE", rpm, mode)
        if motion == self._last_motion:
            return False
        self._send_line(f"DRIVE,{rpm},{mode}")
        self._last_motion = motion
        return True

    def set_claw(self, percent):
        if isinstance(percent, bool) or not isinstance(percent, int):
            raise TypeError("夹爪开度必须是整数")
        if not 0 <= percent <= 100:
            raise ValueError("夹爪开度必须在 0～100 之间")
        return self._send_line(f"CLAW,{percent}")

    def send_raw_key(self, key):
        """发送交互调试程序保留的 STM32 单字节命令。"""
        if not isinstance(key, str) or len(key) != 1 or not key.isascii():
            raise ValueError("调试命令必须是单个 ASCII 字符")
        self._write(key.encode("ascii"))
        self._last_motion = None
        return True

    def close(self, stop=True):
        if not self.is_open:
            return
        if stop:
            try:
                self.emergency_stop()
            except (OSError, serial.SerialException, ConnectionError):
                pass
        if self._owns_serial:
            self._serial.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close(stop=True)
