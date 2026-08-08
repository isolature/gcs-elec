#!/usr/bin/env python3

import sys
import os
import glob
import tty
import termios
import threading
import serial


# ============================================================
# STM32 当前已有的单字符命令
# ============================================================

COMMANDS = {
    # 双轮直线闭环
    'y': '双轮 +85 RPM，启动直线同步闭环',
    's': '立即停止双轮',

    # 单轮 PI
    't': '切换左/右轮，停止电机并重置目标 +85 RPM',
    'i': '启动所选轮子的 PI 闭环',
    'p': '启动所选轮子的纯 PI 闭环',
    'v': '输出所选轮子的 PI 状态',
    'r': '清除 PI 积分',

    ']': 'Kp +1.0',
    '[': 'Kp -1.0',
    'k': 'Ki +0.1',
    'j': 'Ki -0.1',

    # 速度
    '1': '目标速度 +85 RPM',
    '2': '目标速度 +100 RPM',
    '3': '目标速度 +108 RPM',
    '4': '目标速度 -85 RPM',
    '5': '目标速度 -100 RPM',
    '6': '目标速度 -108 RPM',
    '0': '目标速度 0 RPM',
    'u': '目标速度 +5 RPM',
    'd': '目标速度 -5 RPM',

    # 舵机
    'a': '切换左/右舵机',
    '7': '舵机目标 1400 us',
    '8': '舵机目标 1500 us',
    '9': '舵机目标 1600 us',
    'n': '舵机目标脉宽 -50 us',
    'm': '舵机目标脉宽 +50 us',
    'h': '输出所选舵机状态',
    'q': '停止所选舵机 PWM',
    'w': '启动所选舵机 PWM，并回到约 1500 us',
}


# ============================================================
# 自动寻找 STM32
# ============================================================

def find_stm32():
    # 优先使用稳定的 by-id 路径
    candidates = (
        glob.glob('/dev/serial/by-id/*STM*') +
        glob.glob('/dev/serial/by-id/*STMicroelectronics*')
    )

    if candidates:
        return candidates[0]

    # 找不到则退回 ttyACM0
    if os.path.exists('/dev/ttyACM0'):
        return '/dev/ttyACM0'

    raise FileNotFoundError(
        "没有找到 STM32。\n"
        "请检查 USB 数据线，并运行 ls /dev/ttyACM*"
    )


# ============================================================
# 帮助页面
# ============================================================

def print_help():
    print("""
====================================================
        Rescue Robot Wireless Debug Console
====================================================

双轮：
    y       双轮 +85 RPM 直线闭环
    s       立即停止双轮

单轮 PI：
    t       左右轮切换
    i       PI 闭环
    p       纯 PI 闭环
    v       输出 PI 状态
    r       清积分

    [ / ]   Kp -/+ 1.0
    j / k   Ki -/+ 0.1

速度：
    1       +85 RPM
    2       +100 RPM
    3       +108 RPM

    4       -85 RPM
    5       -100 RPM
    6       -108 RPM

    0       0 RPM
    u / d   目标速度 +/- 5 RPM

舵机：
    a       左右舵机切换
    7       1400 us
    8       1500 us
    9       1600 us
    n / m   脉宽 -/+ 50 us
    h       输出舵机状态
    q       停止舵机 PWM
    w       启动 PWM，并回到约 1500 us

本地控制（不会发送给 STM32）：
    ?       显示帮助
    Ctrl+L  显示/隐藏 STM32 实时日志
    Ctrl+C  退出程序，并自动发送 s 停车

====================================================
""")


# ============================================================
# 打开串口
# ============================================================

port = find_stm32()

print(f"发现 STM32: {port}")

ser = serial.Serial(
    port=port,
    baudrate=115200,
    timeout=0.1
)

print("STM32 串口已打开")


# ============================================================
# 后台读取 STM32 数据
# ============================================================

running = True
show_logs = False


def serial_reader():
    while running:
        try:
            data = ser.read(ser.in_waiting or 1)

            if data and show_logs:
                text = data.decode(
                    'utf-8',
                    errors='replace'
                )

                sys.stdout.write(text)
                sys.stdout.flush()

        except serial.SerialException:
            break


reader_thread = threading.Thread(
    target=serial_reader,
    daemon=True
)

reader_thread.start()


# ============================================================
# 键盘控制
# ============================================================

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

try:
    # 改成单键读取，不需要按 Enter
    tty.setcbreak(fd)

    print_help()

    while True:
        key = sys.stdin.read(1)

        # Ctrl + L
        if key == '\x0c':
            show_logs = not show_logs

            if show_logs:
                print("\n[STM32 日志：开启]")
            else:
                print("\n[STM32 日志：关闭]")

            continue

        # ?
        if key == '?':
            print_help()
            continue

        # STM32 已定义命令
        if key in COMMANDS:
            ser.write(key.encode('ascii'))
            ser.flush()

            print(
                f"\rTX [{key}] -> {COMMANDS[key]}"
            )

except KeyboardInterrupt:
    print("\n收到退出指令")

finally:
    # 无论为什么退出，都尝试先停车
    try:
        ser.write(b's')
        ser.flush()
        print("已发送 s：停止双轮")
    except Exception:
        pass

    running = False

    ser.close()

    termios.tcsetattr(
        fd,
        termios.TCSADRAIN,
        old_settings
    )

    print("串口已关闭")