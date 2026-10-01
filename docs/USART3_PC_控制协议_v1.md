# TerraMind USART3 PC 控制与状态协议 v1

本文对应固件 `TerraMind_FW/Driver/pc_protocol.*` 和 PC 参考实现 `tools/pc_protocol.py`。

## 1. 接口与运行方式

- 开发板控制接口：USART3，PD8=TX、PD9=RX，115200 bit/s、8 数据位、无校验、1 停止位、无流控。
- 调试文本：USART6，PC6=TX、PC7=RX，同为 115200 8N1。USART3 只发送和接收本文的二进制帧。
- PC 连接板卡引脚时使用 **3.3 V TTL USB 串口**，板卡 TX 接适配器 RX，板卡 RX 接适配器 TX，双方共地。先核对板卡接口是否已有板载串口转换器；不能直接接 RS-232 电平。
- PC 每 20–50 ms 发送一帧完整控制快照。开发板每 50 ms 发送一帧完整状态快照，不另发送 ACK 帧。每帧独立，不能只发送发生变化的字段。
- 帧在字节流中可连续发送，没有换行符。PC 接收器必须按帧头、长度、帧尾和 CRC 分帧，不能把一次 `read()` 等同于一帧。

当前 USB 串口上位机只需实现两种帧：`0x01` 控制下发、`0x81` 状态上报。

## 2. 通用帧

| 偏移 | 长度 | 字段 | 说明 |
|---:|---:|---|---|
| 0 | 2 | 帧头 | `FC FB` |
| 2 | 1 | 版本 | 固定 `01` |
| 3 | 1 | 类型 | `01` 控制；`81` 状态 |
| 4 | 2 | 帧序号 | `uint16`，每方向各自递增，回绕到 0 |
| 6 | 2 | 负载长度 N | `uint16`，0–256；v1 控制为 48，状态为 105 |
| 8 | N | 负载 | 功能块序列 |
| 8+N | 2 | CRC16 | 对偏移 **2 至 7+N** 计算，低字节先发 |
| 10+N | 2 | 帧尾 | `FD FE` |

总长度为 `N+12`：v1 控制帧 60 字节，状态帧 117 字节。所有整数和 IEEE-754 `float32` 使用**小端序**。CRC 为 **CRC-16/XMODEM**：多项式 `0x1021`，初值 `0x0000`，不反射，结果异或 `0x0000`。测试向量 `"123456789" → 0x31C3`。

### 功能块 TLV

负载由若干 `ID:u8 | LEN:u8 | DATA:LEN字节` 组成。v1 下列七个块各出现**恰好一次**；次序不限。缺少已知必需块、重复已知块、已知块长度错误或无效字段使整帧失效，固件不会应用其中任何部分。未知 ID 按 LEN 跳过，可用于新增外设。未知块不能替代 v1 必需块。新增兼容字段使用新 ID；改变既有字段含义或布局须升级协议版本。

## 3. PC → 开发板：完整控制帧 `TYPE=0x01`

| ID | LEN | DATA 字节布局 | 单位与取值 |
|---|---:|---|---|
| `01` 系统 | 1 | `flags:u8` | bit0 控制使能；bit1 停止请求；其他位必须为 0。未使能或请求停止时执行器目标全部归零。 |
| `10` 底盘 | 8 | `linear:f32, angular:f32` | 线速度 m/s：-0.35…0.35；角速度 rad/s：-2…2，正值左转、负值右转。固件已做整车转向标定，PC 无需额外取反。 |
| `20` 左播撒 | 5 | `on:u8, rpm:f32` | `on` 为 0/1；目标输出轴速度 -500…500 RPM。关闭时速度目标归零。 |
| `21` 右播撒 | 5 | `on:u8, rpm:f32` | 同上；当前默认工作方向为 -200 RPM。 |
| `30` 割草刀盘 | 5 | `on:u8, throttle:f32` | `on` 为 0/1；电调油门 0…100%。 |
| `40` 刀盘升降 | 5 | `on:u8, height:f32` | `on` 为 0/1；距标定零点的**绝对目标高度** 0…1000 mm。当前预留。 |
| `50` 喷洒装置 | 5 | `on:u8, throttle:f32` | `on` 为 0/1；B 口水泵逻辑油门 0…100%。0 或关闭时停泵；非零由固件映射到 70…100% PWM，见第 7 节。不是实测转速或流量。 |

PC 即使暂时不用某个装置，也须发送其功能块，并将 `on=0`、目标值置 0。浮点字段不能是 NaN、无穷大或越界值。当前刀盘升降尚未接入；在控制使能且未请求停止时，若升降 `on=1`，**整帧以 `UNSUPPORTED` 拒绝**，其他装置也不会动作。喷洒已经支持。目标值在 `on=0` 时仍须落入有效范围。

一个全零、未使能、序号为 0 的合法控制帧：

```text
FC FB 01 01 00 00 30 00
01 01 00
10 08 00 00 00 00 00 00 00 00
20 05 00 00 00 00 00
21 05 00 00 00 00 00
30 05 00 00 00 00 00
40 05 00 00 00 00 00
50 05 00 00 00 00 00
BB 1E FD FE
```

`BB 1E` 是这个示例帧的 CRC16 小端字节序。

## 4. 开发板 → PC：完整状态帧 `TYPE=0x81`

所有浮点量均为 `float32` 小端序。`actual` 表示已有传感器反馈；`target` 或 `throttle` 表示控制设定值。PC 应同时检查 `capabilities` 和反馈有效标志。

| ID | LEN | DATA 字节布局 |
|---|---:|---|
| `01` 系统 | 16 | `uptime_ms:u32, last_command_seq:u16, result:u8, mode:u8, faults:u16, capabilities:u16, command_age_ms:u16, rx_error_count:u16` |
| `10` 底盘 | 24 | `linear_mps:f32, angular_radps:f32, left_target_rpm:f32, right_target_rpm:f32, left_actual_rpm:f32, right_actual_rpm:f32` |
| `20` 左播撒 | 13 | `on:u8, target_rpm:f32, actual_rpm:f32, servo_set_angle_deg:f32` |
| `21` 右播撒 | 13 | 同左播撒 |
| `30` 割草刀盘 | 5 | `on:u8, throttle_set_percent:f32` |
| `40` 刀盘升降 | 10 | `on:u8, target_mm:f32, actual_mm:f32, feedback_valid:u8` |
| `50` 喷洒装置 | 10 | `on:u8, target_percent:f32, actual_percent:f32, feedback_valid:u8` |

`last_command_seq` 是最近**帧结构和 CRC 正确**的控制包序号；结合 `result` 判断是否应用。坏 CRC 或坏帧头不会更新序号。`command_age_ms` 是距最近**成功接纳**控制包的毫秒数，上限 65535；尚未接纳过控制包时为 65535。状态帧自身序号独立递增。

| 字段 | 值 | 含义 |
|---|---|---|
| `result` | 0 | OK；控制快照已接纳。 |
|  | 1 | BAD_FRAME；必需块、长度或字段编码错误。 |
|  | 2 | BAD_VALUE；浮点值无效或超限。 |
|  | 3 | UNSUPPORTED；尝试启用未接入的外设。 |
|  | 4 | TIMEOUT；预留结果码；当前超时通过故障位表示。 |
| `mode` | 0 | USART3 尚未接纳控制包，沿用 UART5 控制。 |
|  | 1 | USART3 控制使能且命令新鲜。 |
|  | 2 | USART3 已接管，但当前停机、未使能或超时。 |

`faults` 位：bit0=PC 控制超时，bit1=最近控制周期 CAN 发送失败，bit2=USART3 接收环形缓冲区曾溢出。`capabilities` 位：bit0=底盘、bit1=左播撒、bit2=右播撒、bit3=割草刀盘、bit4=刀盘升降、bit5=喷洒。水泵初始化成功时，当前固件上报 `0x002F`；水泵初始化失败时不置喷洒能力位。未接入的升降状态为 0、反馈无效。喷洒 `target_percent` 上报经过停机逻辑和限幅后的逻辑油门（例如 PC 下发 50，状态回报 50，PWM 为 85%），`on` 表示该逻辑目标大于 0；没有传感器，故 `actual_percent=0`、`feedback_valid=0`，PC 不应将其显示为实测零转速。当前 M3508 与播撒电机虽可上报 RPM，但状态帧尚未附带单独的反馈新鲜度标志；PC 应结合系统故障和设备现场状态使用。

## 5. 控制权、失联与异常

1. 上电时沿用现有 UART5 控制。USART3 第一次接纳合法控制快照后取得控制权；**本次上电期间不自动切回 UART5**，避免旧指令重新驱动装置。发送 `enable=0` 或 `stop=1` 可停机，但不会释放控制权。
   UART5 继续使用原有 CmdPort 格式，保留两个 CRC 字节的位置但不校验其数值（可填 `00 00`）；USART3 仍校验 CRC。两种帧不可互换。
2. USART3 连续超过 **250 ms** 未收到可接纳的完整控制帧，所有运动和作业目标置零，状态 `mode=2` 且 `faults.bit0=1`。再次接纳合法、使能的控制帧即可恢复。
3. CRC 错、帧尾错或版本不支持时直接丢弃；语义错误保持上一次已接纳命令，但不能续期失联计时。PC 可以从 `rx_error_count` 和 `result` 诊断。
4. 上位机启动时先发送未使能的全零快照，观察状态中的 `capabilities`、序号及结果，再发送使能命令。停止时发送 `stop=1` 的完整快照；如果 PC 意外退出，250 ms 超时保护生效。
5. UART 串口不是硬实时总线。超时保护依赖固件控制任务持续运行，不能代替物理急停电路。

## 6. PC 参考代码

`tools/pc_protocol.py` 使用 Python 标准库实现控制帧编码、CRC、状态帧流式分帧与 TLV 解析。运行只读监视示例需要 `pyserial`：

```powershell
python -m pip install pyserial
python tools/pc_protocol.py COM5
```

示例脚本每 50 ms 下发未使能的全零控制快照，并打印完整状态。它不会启动任何执行器。集成到上位机时，更新 `Control` 的全部字段并持续调用 `build_control(command, seq)`；对串口读出的任意长度字节块调用 `StatusParser.feed(data)`，该方法返回零个或多个 `(status_frame_seq, status_dict)`。不要按换行或固定 `read(117)` 分帧。

```python
from tools.pc_protocol import Control, StatusParser, build_control

command = Control(enable=True, linear_mps=0.1)
wire_bytes = build_control(command, seq=1)
parser = StatusParser()
# serial.write(wire_bytes)
# for status_seq, status in parser.feed(serial.read(256)):
#     assert status["last_command_seq"] == 1 and status["result"] == 0
```

新外设扩展时：分配新 TLV ID，定义长度、单位、取值、反馈有效性和能力位；旧 PC 忽略未知状态块，旧固件忽略未知控制块。对当前七个必需块的字节布局保持不变。

## 7. B 口水泵油门标定与 PC 控制

水泵使用 `PwmOpenLoopMotor`，不运行 PID，不启动或读取 TIM3 编码器。已实测确认正向正确：PE5/TIM9_CH1 输出、PE6/TIM9_CH2 关闭；停机时两路都关闭。沿用现有 TIM9 频率，不改其他电机的定时器参数。

实测水泵在约 70% PWM 时起转。`app_main.cpp` 的 `init_spray()` 将 `min_throttle_percent` 配为 `70.0f`、上限为 `100.0f`，方向固定为 `Forward`。映射由固件驱动统一完成，上位机只发送 0…100 的逻辑油门，不要提前映射：

```text
逻辑油门 = 0：PWM = 0，停泵
逻辑油门 > 0：PWM百分比 = 70 + 30 × 逻辑油门 / 100
```

| PC 逻辑油门 | PWM 设定值 |
|---:|---:|
| 0% | 0%（停止） |
| 1% | 70.3% |
| 20% | 76% |
| 50% | 85% |
| 100% | 100% |

PWM 按 ARR 换算并四舍五入为比较值，100% 为接口最大比较值 ARR；这是一项起转区间补偿，不代表实际转速或流量随逻辑油门线性变化。

独立固定油门测试注入及其宏已移除。上电和 UART5 模式下水泵均保持关闭；只有 USART3 PC 的有效控制快照可以启动水泵。需同时满足 `enable=1`、`stop=0`、`spray_on=1`、`spray_percent>0`。关闭喷洒、油门归零、取消使能、全局停止或超过 250 ms 未收到可接纳命令，均输出零 PWM。再次收到满足上述条件的命令即可恢复。

现有 `Control` 编码器已支持喷洒字段。例如下列命令对应 85% PWM：

```python
command = Control(enable=True, spray_on=True, spray_percent=50.0)
wire_bytes = build_control(command, seq=1)
```

每 20–50 ms 持续发送完整控制快照，包含其他执行器的当前目标。只关闭喷洒时在原 `command` 上设置 `spray_on=False`、`spray_percent=0.0`，其他字段不变。状态中的 `spray` 元组为 `(on, logical_target_percent, actual_percent, feedback_valid)`，上例为 `(1, 50.0, 0.0, 0)`。参考串口监视工具默认发送未使能帧，因此不会启动水泵。

帧版本与长度保持 v1 的 60 字节控制帧、117 字节状态帧。

主机回归测试：`python tests/run_host_tests.py`（需要支持 C++11 的 `g++`，可通过 `--cxx` 指定）。测试使用模拟 HAL，不连接或驱动硬件；覆盖起转油门映射、方向、零油门/开关/全局停止/超时、协议状态，以及改变喷洒指令时其他执行器输出一致性。
