# 正式树莓派控制客户端

`rescue_car_client.py` 是视觉程序连接正式 STM32 比赛固件的运行时接口。旧文本协议的 `car_control.py` 保留不变，二者不能同时打开同一个串口。

## 最小使用方式

```python
import time

from rescue_car_client import RescueCarClient

with RescueCarClient() as car:
    car.arm()
    car.set_velocity(150, 0)
    time.sleep(2)
    car.stop()
    car.disarm()
```

客户端后台自动完成：

- USB CDC 连接与 HELLO 兼容性检查；
- 10 Hz HEARTBEAT；
- 非零运动目标的 10 Hz 刷新与 300 ms TTL；
- `SAFETY_STATUS` 和 `ROBOT_STATE` 接收；
- USB 断线后的自动重连。

自动重连只重新建立一个保持 `DISARMED` 的新会话，不会自动 ARM，也不会恢复断线前的运动目标。视觉程序必须根据当前任务状态明确重新调用 `arm()`。

## API

```python
car.connect()
car.arm()
car.disarm()
car.safe_stop()

car.set_velocity(linear_mm_s, angular_mrad_s, ttl_ms=300)
car.stop()
car.forward(speed_mm_s=150)
car.backward(speed_mm_s=150)
car.turn_left(rate_mrad_s=1000)
car.turn_right(rate_mrad_s=1000)

car.open_gripper()
car.close_gripper()

snapshot = car.snapshot()
safety = car.latest_safety_status
state = car.latest_robot_state
```

`close()` 和上下文管理器退出时，若客户端仍处于 `ARMED`，会尽力发送 `SAFE_STOP`；已经 `DISARMED` 时直接关闭，不改变安全状态。这只是进程正常退出保护；进程崩溃、USB 断开或树莓派失联时，由 STM32 的 300 ms 心跳看门狗停车。

## 验证

离线测试不需要连接 STM32：

```bash
python3 -B -m unittest -v test_rescue_car_client.py
```

连接正式固件后先架空运行示例：

```bash
python3 formal_client_example.py
```
