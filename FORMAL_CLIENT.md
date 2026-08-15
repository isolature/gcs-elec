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

- `R`：按一次取得控制权并 ARM；正常操控期间不需要按住或重复按。租约超时、安全停车或断连后，`R` 取得新一代控制权，不复用旧运动目标。
- `U`：DISARM（保留控制权，`R` 重新 ARM）。
- `W` / `S` / `A` / `D`：前进、后退、左转、右转；按住持续运动（依赖系统按键重复，约 25–30 Hz）。
- `Space`：普通停车，保留控制权和 ARM；随后可直接继续运动键。
- `O` / `C`：夹爪张开、合拢。
- `P`：双相机拍照（far+near 成对）；录像进行中会被状态机拒绝并记录事件。拍照不需要控制权。
- `V`：双路录像开/关。不需要控制权。
- `E`：安全停车并解除控制权；`R` 重新取权。
- `H` 或 `?`：显示帮助和当前 `ControlCore` 状态。
- `Q`：执行安全清理并退出。

输入超时与松键语义（`rescue_control/cli.py`）：

- 终端无法可靠获得物理松键事件。运动键依靠按键重复维持；当重复停止约 150 ms（`MOTION_RELEASE_S`）且当前存在非零速度设定时，交互循环主动经 `ControlCore` 发送一次零速度设定并同步核心状态。固件侧运动 TTL 与看门狗仍然是独立的物理兜底；上位机不承担看门狗职责。
- 控制租约默认 30 s（`INTERACTIVE_LEASE_S`），任意成功命令都会续租。正常操控期间控制权持续保持；只有超过 30 s 完全无按键输入才触发 `COMMAND_TIMEOUT` 安全停车并解除控制权（输入静默失败关闭）。
- 不要把键盘重复延迟设置得接近或超过 150 ms，否则按住运动键会被推断为松键（表现为短暂停车，再按即恢复）。EOF、Ctrl-C、未处理异常、连接中断和正常退出都会进入 `ControlCore.shutdown(reason)`，尝试安全停车后关闭唯一正式串口。

## 遥控中拍照与录像（P/V）

`P`（拍照）和 `V`（录像开关）把遥控会话桥接到主仓库的 `rescue_camera_capture` 采集子项目，不复制其逻辑：状态机（录像中拒拍）、双路录像回滚、session 存储、清单与回传全部仍归采集子项目所有。

架构与边界（`rescue_control/capture.py`）：

- 相机应用独占运行在一个专用工作线程上（与采集子项目自身的单线程拥有模型一致）；控制循环只投递动作和收取事件，永不阻塞，也永不因拍照/录像失败触发安全停车。相机线程异常只产生 `capture` NDJSON 事件。
- `P`/`V` 不需要控制权（`R`/租约与相机完全解耦）；相机路径零串口接触。
- 会话结束时（`Q`、EOF、Ctrl-C、断连、异常）自动先停录像、关相机、终结 session 并在 NDJSON 汇报 `session_id`/`session_dir`；数据回传（清单、断点续传、核验）是退出遥控后由笔记本执行的独立步骤，不在遥控会话内进行。
- 相机是独占设备：遥控相机会话期间不能同时运行交互式 `rescue_camera_capture` 终端程序；反之亦然。

启用方式（Pi）：用采集 venv 的解释器启动遥控，使 `rescue_camera_capture` 可导入（venv 以 `--system-site-packages` 建立，pyserial 等系统包仍可用）：

```bash
cd /home/gcs/gcs-elec
~/gcs-camera-capture/.venv/bin/python -m rescue_control teleop \
  --port /dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_348935793135-if00 \
  --camera-map /home/gcs/gcs-camera-capture/config/camera_map.local.json
```

`--camera-map` 启用 P/V；`--capture-root DIR` 可覆盖输出根目录（默认沿用采集子项目的 `~/gcs-data` 约定）。不传 `--camera-map` 时 P/V 明确报告 `CAMERA_NOT_ENABLED`，遥控不受影响。包缺失或相机初始化失败只产生 `capture/UNAVAILABLE` 事件，之后 P/V 报告 `CAMERA_SESSION_CLOSED`，驾驶继续。

NDJSON 事件：`capture/READY`（含 session 身份）、`key/PHOTO|VIDEO ACCEPTED|BUSY|SKIPPED|UNAVAILABLE`、`capture/KEY_RESULT`（`photo_captured`/`recording_started`/`recording_stopped`/`photo_rejected`/`debounced` 或 `error: ...`）、`capture/POLL_ERROR`、`capture/CLOSED`（含 session_id/session_dir 与回传提示）。

WSL 证据边界：以上桥接行为由注入 Fake 相机应用的单元与集成测试覆盖（线程、忙碌、失败隔离、退出收尾）；真实双相机拍照/录像的遥控中验收须在 Pi 上执行（相机已于 2026-08-15 由操作者现场确认安装并完成非遥控模式验收）。

## 输入延迟与 I/O 切片

`RescueCarClient` 以构造参数 `io_slice_s`（默认 `0.02`，独立使用行为不变）控制空闲串口读切片；工作线程在两次读切片之间才检查请求队列，因此该值也约束了排队命令的额外分发延迟。`CompetitionLinkConfig.io_slice_s` 默认 `0.005`，即正式控制链的取值。

`benchmark_teleop_latency.py` 在 WSL/假串口下以相同速度设定（线速度 200 mm/s、角速度 0）对比两种切片（2026-08-14，300 样本）：

| 模式 | io_slice_s | motion p50 | motion mean | motion p95 | stop p50 |
|---|---|---|---|---|---|
| 旧参数（main@adaae84 行为） | 0.02 | 7.5 ms | 8.6 ms | 14.8 ms | 7.4 ms |
| 新参数 | 0.005 | 2.9 ms | 3.1 ms | 4.4 ms | 2.9 ms |

运动指令键到串口帧延迟改善约 61%（p50）/ 64%（mean），远超 20% 目标；改善来自输入延迟，不来自提高速度设定（两模式速度完全相同）。`tests/test_teleop_latency.py` 在测试套件内复跑该对比并断言门槛。

WSL/假串口高压流量下偶发的 `HEARTBEAT_ACK timed out` 断线属于上游 `RescueCarClient` 传输层行为（两种切片取值下都会出现，与本仓库改动无关；失败方向为 fail-closed）。心跳与看门狗语义归电控队友所有，该观察已按交接记录反馈，不在本仓库修改。



`RescueCarClient` 以构造参数 `io_slice_s`（默认 `0.02`，独立使用行为不变）控制空闲串口读切片；工作线程在两次读切片之间才检查请求队列，因此该值也约束了排队命令的额外分发延迟。`CompetitionLinkConfig.io_slice_s` 默认 `0.005`，即正式控制链的取值。

`benchmark_teleop_latency.py` 在 WSL/假串口下以相同速度设定（线速度 200 mm/s、角速度 0）对比两种切片（2026-08-14，300 样本）：

| 模式 | io_slice_s | motion p50 | motion mean | motion p95 | stop p50 |
|---|---|---|---|---|---|
| 旧参数（main@adaae84 行为） | 0.02 | 7.5 ms | 8.6 ms | 14.8 ms | 7.4 ms |
| 新参数 | 0.005 | 2.9 ms | 3.1 ms | 4.4 ms | 2.9 ms |

运动指令键到串口帧延迟改善约 61%（p50）/ 64%（mean），远超 20% 目标；改善来自输入延迟，不来自提高速度设定（两模式速度完全相同）。`tests/test_teleop_latency.py` 在测试套件内复跑该对比并断言门槛。

WSL/假串口高压流量下偶发的 `HEARTBEAT_ACK timed out` 断线属于上游 `RescueCarClient` 传输层行为（两种切片取值下都会出现，与本仓库改动无关；失败方向为 fail-closed）。心跳与看门狗语义归电控队友所有，该观察已按交接记录反馈，不在本仓库修改。

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
python3 -B -m unittest -v tests.test_capture
python3 -B -m unittest discover -v
python3 -B -m rescue_control scenario all
python3 benchmark_teleop_latency.py --samples 300
sh -n run_teleop.sh
git diff --check
```

`tests.test_cli` 用 `FakeLowerLink` 覆盖全部键位、帮助、单次 `R` 跨暂停保持控制、松键自动零速、`Space` 停车保权、`E` 安全停车解权、`U` 后运动需重新 ARM、30 s 输入静默超时、新代际重新 ARM、EOF、Ctrl-C、异常、连接中断和正常退出；正式入口测试以注入 Fake 的方式断言 `CompetitionLowerLink` 构造路径，不会打开真实串口。`tests.test_capture` 与 `tests.test_cli` 的相机会话测试注入 Fake 相机应用，覆盖运动中拍照/录像、忙碌丢弃、初始化失败降级、异常隔离和退出收尾；全部自动化测试都不打开真实相机或串口。

这些结果只证明 WSL 逻辑、Fake 后端与假串口行为。Pi 的 `/dev/serial/by-id/...` 权限和独占打开、终端按键重复节拍、正式 STM32 commit/schema、真实状态节拍、看门狗、架空车轮和实车闭环仍需分别验收。
