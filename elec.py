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
    'y': '双轮 +100 RPM，启动直线同步闭环',
    'z': '双轮 +100 RPM，启动独立 PI（同步环关闭）',
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
    y       双轮 +100 RPM 直线闭环
    z       双轮 +100 RPM 独立 PI（无同步环）
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
    c       输入夹爪开度 0..100（0=闭合，100=最大张开）
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
rx_line = bytearray()


def write_stm32(payload, description):
    """安全写串口，USB 断开时给出简洁提示而不是 traceback。"""
    if not ser.is_open:
        print(f"\n发送失败（{description}）：STM32 串口已关闭")
        return False

    try:
        ser.write(payload)
        ser.flush()
        return True
    except (OSError, serial.SerialException) as exc:
        print(f"\n发送失败（{description}）：{exc}")
        print("请检查车辆总电和 STM32 USB 连接，然后重新运行本程序。")
        return False


def serial_reader():
    global running

    while running:
        try:
            data = ser.read(ser.in_waiting or 1)

            if not data:
                continue

            if show_logs:
                sys.stdout.write(data.decode('utf-8', errors='replace'))
                sys.stdout.flush()
                continue

            # 测速日志隐藏时，仍显示夹爪命令的确认和错误。
            for value in data:
                if value in (10, 13):
                    if rx_line:
                        line = rx_line.decode('utf-8', errors='replace').strip()
                        rx_line.clear()
                        if line and ('CLAW' in line or line.startswith('ERR')):
                            print(f"\n{line}")
                elif len(rx_line) < 512:
                    rx_line.append(value)
                else:
                    rx_line.clear()

        except (OSError, serial.SerialException) as exc:
            if running:
                print(f"\nSTM32 串口连接中断：{exc}")
                print("请检查车辆总电和 USB 连接，然后重新运行本程序。")
            running = False
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

    while running:
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

        # c 是树莓派本地交互命令，不作为单字符直接发送。
        # 暂时恢复普通终端模式，方便输入多位百分比和按回车确认。
        if key.lower() == 'c':
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            try:
                value = input(
                    "\n请输入夹爪开度 0～100（直接回车取消）："
                ).strip()
            finally:
                tty.setcbreak(fd)

            if not value:
                print("已取消夹爪命令")
                continue

            try:
                percent = int(value, 10)
            except ValueError:
                print("输入无效：请输入 0～100 的整数")
                continue

            if not 0 <= percent <= 100:
                print("输入越界：夹爪开度必须在 0～100 之间")
                continue

            command = f"CLAW,{percent}\r\n".encode('ascii')
            if write_stm32(command, f"夹爪开度 {percent}%"):
                print(f"TX [CLAW,{percent}] -> 夹爪开度 {percent}%")
            continue

        # STM32 已定义命令
        if key in COMMANDS:
            if write_stm32(key.encode('ascii'), COMMANDS[key]):
                print(f"\rTX [{key}] -> {COMMANDS[key]}")

except KeyboardInterrupt:
    print("\n收到退出指令")

finally:
    # 无论为什么退出，都尝试先停车
    if write_stm32(b's', '退出前停车'):
        print("已发送 s：停止双轮")

    running = False

    try:
        if ser.is_open:
            ser.close()
    except (OSError, serial.SerialException):
        pass

    termios.tcsetattr(
        fd,
        termios.TCSADRAIN,
        old_settings
    )

    print("串口已关闭")
