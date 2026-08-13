# 正式上位机控制链

正式比赛固件的唯一上位机控制路径是：

```text
终端键盘遥控 / 浏览器调试入口 / 自动决策
            ↓
       ControlCore
            ↓
 CompetitionLowerLink
            ↓
    RescueCarClient
            ↓
 rescue_car_protocol → USB CDC / STM32
```

- `ControlCore` 独占控制租约、命令门禁、失败关闭和安全停车策略。
- `CompetitionLowerLink` 负责业务状态映射、新鲜度、完成确认、投递不确定性，以及重连/session/boot 后使旧控制权失效。
- `RescueCarClient` 是正式协议运行时：唯一工作线程拥有串口，并集中实现 HELLO、心跳、运动刷新、状态接收和重连。
- `rescue_car_protocol` 只负责正式线协议编解码。

上层代码不得直接调用 `RescueCarClient` 或写串口。同一串口也不能与 `car_control.py`、`elec.py` 或 `draft_protocol_test.py` 同时打开。

## 最小使用方式

部署时显式传入实际稳定路径，不要写死 `/dev/ttyACM0`：

```python
import time

from rescue_control import CompetitionLowerLink, ControlCore, SafeStopReason

link = CompetitionLowerLink(
    port="/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_..."
)
core = ControlCore(link)

try:
    core.connect()
    lease = core.acquire_lease("autonomy", duration_s=3.0)
    core.arm(lease)
    core.set_chassis(lease, 150, 0, 300)
    time.sleep(2.0)
    core.stop(lease)
finally:
    core.shutdown(SafeStopReason.SHUTDOWN)
```

`formal_client_example.py` 提供同一控制链的命令行示例，并要求用 `--port` 显式指定串口。

## Pi 终端键盘遥控

部署到 `/home/gcs/gcs-elec` 后，使用稳定的设备唯一标识启动：

```bash
cd /home/gcs/gcs-elec
./run_teleop.sh \
  --port /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_348935793135-if00
```

等价的模块入口是：

```bash
python3 -m rescue_control teleop --port /dev/serial/by-id/<exact-device-name>
```

`--port` 必填且只接受 `/dev/serial/by-id/` 下的精确单设备路径；不会回退或自动发现为 `/dev/ttyACM0`。脚本会把参数和 Python 退出状态原样传递，不隐藏连接、协议或清理失败。

键位如下：

- `R`：取得控制权并 ARM；租约超时或安全停车后会取得新一代控制权，不复用旧运动目标。
- `U`：DISARM。
- `W` / `S` / `A` / `D`：前进、后退、左转、右转。
- `Space`：普通停车。
- `O` / `C`：夹爪张开、合拢。
- `E`：安全停车。
- `H` 或 `?`：显示帮助和当前 `ControlCore` 状态。
- `Q`：执行安全清理并退出。

终端无法可靠获得松键事件，因此运动依靠操作系统的按键重复持续刷新 500 ms 输入租约；停止收到成功运动输入后，租约超时由 `ControlCore` 触发安全停车。不要把键盘重复延迟设置得接近或超过 500 ms；松键时可按 `Space` 立即普通停车。EOF、Ctrl-C、未处理异常、连接中断和正常退出都会进入 `ControlCore.shutdown(reason)`，尝试安全停车后关闭唯一正式串口。

## 关键语义

- HELLO_ACK 后还必须收到当前连接代次内、结构有效且新鲜的 `SAFETY_STATUS` 和 `ROBOT_STATE`，业务层 `connect()` 才成功。
- `ACCEPTED` 只表示下位机接受命令。ARM、DISARM、普通停车和安全停车必须由命令后的新鲜状态确认，才能返回 `COMPLETED`。
- 当前夹爪无可靠到位反馈时，结果保持 `ACCEPTED`，不会虚构完成；只有新鲜 `OPEN`/`CLOSED` 状态才返回 `COMPLETED`。
- `stop()` 可由 `ControlCore` 去重；`safe_stop(reason)` 永不去重，并在核心事件、快照和适配器结果中保留原始原因。
- Draft v0.1 的 SAFE_STOP 线消息目前只编码 `USER_REQUEST`。更细的上位机原因先保留在 Pi 侧；STM32 回报的 `SAFETY_STATUS.stop_reason` 仍是物理状态的权威来源。扩展线协议原因必须与 STM32 一起更新 schema。
- 命令调用超时一律按 `MAY_HAVE_APPLIED` 保守处理；关键操作未确认时 `ControlCore` 会撤销租约并失败关闭。
- 每次重连都会产生新的 connection generation、session 和 boot 身份；任一变化都使旧租约和 ARM 权限失效，必须重新连接并显式 ARM。
- `close()` 不增加第二次无原因的停车。`ControlCore.shutdown()` 先执行一次带原因的安全停车，适配器再用 `client.close(stop=False)` 释放唯一串口。
- `validity_flags == 0` 的反馈被显式映射为 `UNKNOWN`；超龄反馈映射为 `STALE`；非法枚举或状态矛盾映射为 `INVALID`。Draft v0.1 尚未给每个 bit 命名，当前非零值只作为所用状态组可用的保守门禁，位分配仍须随 STM32 schema 冻结。

## WSL / 假串口验证

以下命令不会打开真实串口；正式客户端测试通过注入 `serial_factory` 使用假 STM32：

```bash
python3 -B -m unittest -v test_rescue_car_client.py
python3 -B -m unittest -v tests.test_competition_lower_link
python3 -B -m unittest -v tests.test_cli
python3 -B -m unittest discover -v
python3 -B -m rescue_control scenario all
sh -n run_teleop.sh
git diff --check
```

`tests.test_cli` 用 `FakeLowerLink` 覆盖全部键位、帮助、输入超时、新代际重新 ARM、EOF、Ctrl-C、异常、连接中断和正常退出；正式入口测试以注入 Fake 的方式断言 `CompetitionLowerLink` 构造路径，不会打开真实串口。

这些结果只证明 WSL 逻辑、Fake 后端与假串口行为。Pi 的 `/dev/serial/by-id/...` 权限和独占打开、终端按键重复节拍、正式 STM32 commit/schema、真实状态节拍、看门狗、架空车轮和实车闭环仍需分别验收。
