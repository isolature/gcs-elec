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
    'z': '双轮 +85 RPM，启动独立 PI（同步环关闭）',
    'Y': '双轮 +100 RPM，启动直线同步闭环',
    'Z': '双轮 +100 RPM，启动独立 PI（同步环关闭）',
    'M': '双轮 +108 RPM，启动直线同步闭环',
    'N': '双轮 +108 RPM，启动独立 PI（同步环关闭）',
    's': '立即停止双轮',

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

FIXED_ACTIONS = {
    'F': ('F', '前进：左右轮 +85 RPM'),
    'B': ('B', '后退：左右轮 -85 RPM'),
    'L': ('L', '左转：左轮停止，右轮 +85 RPM'),
    'R': ('R', '右转：左轮 +85 RPM，右轮停止'),
    '0': ('0', '停车'),
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
    z       双轮 +85 RPM 独立 PI（无同步环）
    Y       双轮 +100 RPM 直线闭环
    Z       双轮 +100 RPM 独立 PI（无同步环）
    M       双轮 +108 RPM 直线闭环
    N       双轮 +108 RPM 独立 PI（无同步环）
    s       立即停止双轮

视觉动作（树莓派发送 AUTO 命令）：
    F       前进：左右轮 +85 RPM
    B       后退：左右轮 -85 RPM
    L       左转：左轮停止，右轮 +85 RPM
    R       右转：左轮 +85 RPM，右轮停止
    0       停车

舵机：
    a       左右舵机切换
    7       1400 us
    8       1500 us
    9       1600 us
    n / m   脉宽 -/+ 50 us
    h       输出舵机状态
    q       停止舵机 PWM
    w       启动 PWM，并回到约 1500 us

树莓派交互输入（按键本身不直接透传）：
    o       输入双轮目标 RPM 和模式（正数前进，负数后退）
    d       分别输入左轮和右轮目标 RPM
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


def send_wheel_targets(left_rpm, right_rpm):
    """发送左右轮独立目标；视觉程序也可以复用这个接口。"""
    for name, rpm in (("左轮", left_rpm), ("右轮", right_rpm)):
        if not -108 <= rpm <= 108:
            raise ValueError(f"{name} RPM 必须在 -108～108 之间")
        if rpm != 0 and abs(rpm) < 20:
            raise ValueError(f"{name}非零 RPM 的绝对值必须在 20～108 之间")

    command_text = f"WHEEL,{left_rpm},{right_rpm}"
    description = f"左右轮独立目标：L={left_rpm:+d}, R={right_rpm:+d} RPM"
    return write_stm32(
        f"{command_text}\r\n".encode('ascii'), description
    )


def send_fixed_action(action):
    """发送写死的前进、后退、左转、右转或停车动作。"""
    action = action.upper()
    if action not in ('F', 'B', 'L', 'R', '0'):
        raise ValueError("动作必须是 F/B/L/R/0")
    description = FIXED_ACTIONS[action][1]
    return write_stm32(
        f"AUTO,{action}\r\n".encode('ascii'), description
    )


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
                        if line and (
                            'CLAW' in line or
                            'DRIVE' in line or
                            'WHEEL' in line or
                            line.startswith('ERR')
                        ):
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

        # 大写 F/B/L/R 和数字 0 是给视觉联调准备的固定动作。
        if key in FIXED_ACTIONS:
            action, description = FIXED_ACTIONS[key]
            if send_fixed_action(action):
                print(f"\rTX [AUTO,{action}] -> {description}")
            continue

        # d 在树莓派端输入左右轮独立速度，发送 WHEEL,L,R 行命令。
        if key == 'd':
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            try:
                left_value = input(
                    "\n请输入左轮目标 RPM（-108～108，0=停止）："
                ).strip()
                right_value = input(
                    "请输入右轮目标 RPM（-108～108，0=停止）："
                ).strip()
            finally:
                tty.setcbreak(fd)

            if not left_value or not right_value:
                print("已取消左右轮独立速度命令")
                continue

            try:
                left_rpm = int(left_value, 10)
                right_rpm = int(right_value, 10)
                sent = send_wheel_targets(left_rpm, right_rpm)
            except ValueError as exc:
                print(f"输入无效：{exc}")
                continue

            if sent:
                print(
                    f"TX [WHEEL,{left_rpm},{right_rpm}] -> "
                    f"L={left_rpm:+d}, R={right_rpm:+d} RPM"
                )
            continue

        # o 是参数化双轮闭环命令。正 RPM 为机械前进，负 RPM 为后退；
        # I 表示两轮独立 PI，L 表示直线模式并在达到 90% 目标后启用同步环。
        if key.lower() == 'o':
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            try:
                value = input(
                    "\n请输入双轮目标 RPM（-108～108，0=停车）："
                ).strip()
                mode_value = ""
                if value and value != "0":
                    mode_value = input(
                        "请选择模式 [i=独立 PI / l=直线同步，默认 i]："
                    ).strip().lower()
            finally:
                tty.setcbreak(fd)

            if not value:
                print("已取消双轮速度命令")
                continue

            try:
                rpm = int(value, 10)
            except ValueError:
                print("输入无效：RPM 必须是整数")
                continue

            if not -108 <= rpm <= 108:
                print("输入越界：RPM 必须在 -108～108 之间")
                continue

            if rpm != 0 and abs(rpm) < 20:
                print("输入越界：非零目标的绝对值必须在 20～108 RPM 之间")
                continue

            if rpm == 0:
                mode = 'I'
                description = "双轮停车"
            elif mode_value in ('', 'i'):
                mode = 'I'
                description = f"双轮 {rpm:+d} RPM 独立 PI"
            elif mode_value == 'l':
                mode = 'L'
                description = f"双轮 {rpm:+d} RPM 直线同步"
            else:
                print("模式无效：请输入 i 或 l")
                continue

            command_text = f"DRIVE,{rpm},{mode}"
            if write_stm32(
                f"{command_text}\r\n".encode('ascii'), description
            ):
                print(f"TX [{command_text}] -> {description}")
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
