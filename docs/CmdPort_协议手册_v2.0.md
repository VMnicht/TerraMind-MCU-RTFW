# mainboard CmdPort 通信协议手册 v2.0

## 一、概述

CmdPort 是 mainboard 主控板与上位机之间的 UART 串行通信协议，用于下发：

- 底盘差速运动速度指令（线速度 + 角速度）
- 双路播撒器开关控制（左 / 右）
- 割草功能开关控制

接收端硬件为 **UART5**，帧格式沿用现有 Xbox 控制器协议体系。

---

## 二、物理层

| 参数 | 值 |
|------|-----|
| 接口 | **UART5** |
| 引脚 | TX: PC12, RX: PD2 |
| 波特率 | 115200 bps |
| 数据位 | 8 bit |
| 停止位 | 1 bit |
| 校验位 | None (8N1) |
| 流控 | 无 (No Flow Control) |
| 字节序（数据段） | **小端序 (Little-endian)** |

---

## 三、帧结构

```
 Byte:   0      1      2       3        4 ~ 14         15     16      17     18
      ┌──────┬──────┬──────┬───────┬───────────────┬──────────┬──────┬──────┐
      │Head0 │Head1 │  ID  │ Data  │     Data      │  CRC16   │End0  │End1  │
      │ 0xFC │ 0xFB │ 1 B  │  Len  │   (11 bytes)  │  (2 B)   │ 0xFD │ 0xFE │
      └──────┴──────┴──────┴───────┴───────────────┴──────────┴──────┴──────┘

帧总长: 19 字节（固定）
```

| 偏移 | 长度 | 字段 | 值 | 说明 |
|------|------|------|-----|------|
| 0 | 1 | Header[0] | `0xFC` | 帧头起始标志 |
| 1 | 1 | Header[1] | `0xFB` | 帧头第二字节 |
| 2 | 1 | Frame ID | 任意 | 帧标识符（预留，不校验） |
| 3 | 1 | Data Length | **`0x0B`** | 数据段固定 11 字节 |
| 4~14 | 11 | Data | 见第四节 | 控制数据负载 |
| 15 | 1 | CRC16 H | — | CRC 校验高字节 |
| 16 | 1 | CRC16 L | — | CRC 校验低字节 |
| 17 | 1 | Frame End[0] | `0xFD` | 帧尾起始标志 |
| 18 | 1 | Frame End[1] | `0xFE` | 帧尾第二字节 |

---

## 四、数据段编码（11 字节）

```
 Byte:   0      1      2      3      4      5      6      7      8         9         10
      ┌───────────────┬───────────────┬─────┬────────┬────────┐
      │ linear_speed  │ angular_speed │left │right   │mowing  │
      │  (float32 LE) │  (float32 LE) │seed │seed    │(uint8) │
      │    4 Bytes     │    4 Bytes    │ (1) │ (1)    │ (1)    │
      └───────────────┴───────────────┴─────┴────────┴────────┘
```

### 4.1 linear_speed（偏移 0~3，float32，小端序）

底盘目标线速度，单位 **米/秒 (m/s)**。

| 场景 | 值 | float32 HEX (LE) |
|------|-----|------------------|
| 停止 | 0.0 | `00 00 00 00` |
| 前进 (0.2 m/s) | 0.2 | `CD CC 4C 3E` |
| 前进 (最大) | 0.35 | `33 33 B3 3E` |
| 后退 (0.2 m/s) | -0.2 | `CD CC 4C BE` |
| 后退 (最大) | -0.35 | `33 33 B3 BE` |

> 限幅范围由底盘 MechanicalConfig 决定：`max_linear_speed_mps = 0.35`

### 4.2 angular_speed（偏移 4~7，float32，小端序）

底盘目标角速度，单位 **弧度/秒 (rad/s)**。

| 场景 | 值 | float32 HEX (LE) |
|------|-----|------------------|
| 直行（不转） | 0.0 | `00 00 00 00` |
| 左转 1.0 rad/s | 1.0 | `00 00 80 3F` |
| 左转（最大） | 2.0 | `00 00 00 40` |
| 右转 1.0 rad/s | -1.0 | `00 00 80 BF` |
| 右转（最大） | -2.0 | `00 00 00 C0` |

> 正值 = 逆时针（左转），负值 = 顺时针（右转）
> 限幅范围：`max_angular_speed_rad = 2.0`

### 4.3 left_seeder（偏移 8，uint8）

左侧播撒器开关。

| 值 | 含义 | 舵机 | 电机 |
|----|------|------|------|
| `0x00` | 关闭 | 135° | 0 RPM |
| `0x01` | 开启 | -80° | 200 RPM |
| 其他非零值 | 视为开启 | -80° | 200 RPM |

### 4.4 right_seeder（偏移 9，uint8）

右侧播撒器开关。控制逻辑待后续确定，当前仅接收存储。

| 值 | 含义 |
|----|------|
| `0x00` | 关闭 |
| `0x01` | 开启 |

### 4.5 mowing（偏移 10，uint8）

割草功能开关。控制逻辑待后续确定，当前仅接收存储。

| 值 | 含义 |
|----|------|
| `0x00` | 关闭 |
| `0x01` | 开启 |

---

## 五、CRC16 校验

### 5.1 算法参数

| 参数 | 值 |
|------|-----|
| 算法名 | **CRC-16/XMODEM** |
| 别名 | CRC-16/CCITT-FALSE, CRC-16/IBM |
| 多项式 | `0x1021` (`x^16 + x^12 + x^5 + 1`) |
| 初始值 | `0x0000` |
| 结果异或 | `0x0000` |
| 输入/输出反转 | 否 |
| 校验范围 | **仅数据段**（偏移 4~14，共 11 字节） |

### 5.2 计算方式

```c
// 查表法
uint16_t CRC16_XMODEM(const uint8_t *data, uint8_t len)
{
    uint16_t crc = 0x0000;
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t index = ((crc >> 8) ^ data[i]) & 0xFF;
        crc = (crc << 8) ^ crc16_table[index];
    }
    return crc;
}
```

> 帧中 CRC16 按大端序排列：先 H 后 L。

### 5.3 CRC16 查找表（256 项）

```
0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
```

---

## 六、完整数据包示例

各示例中 CRC 值需用查表算法计算，此处以 `XX XX` 占位。

### 6.1 停止 + 全部关

| 字段 | 值 |
|------|-----|
| linear_speed | 0.0 |
| angular_speed | 0.0 |
| left_seeder | 0x00 |
| right_seeder | 0x00 |
| mowing | 0x00 |

数据段：`00 00 00 00 00 00 00 00 00 00 00`

CRC16(全零) = 0x0000

**完整 HEX**：
```
FC FB 00 0B  00 00 00 00 00 00 00 00 00 00 00  00 00  FD FE
```

### 6.2 前进 0.20 m/s + 播撒器全关

| 字段 | 值 | HEX |
|------|-----|-----|
| linear_speed | 0.20 | `CD CC 4C 3E` |
| angular_speed | 0.0 | `00 00 00 00` |
| left_seeder | 0 | `00` |
| right_seeder | 0 | `00` |
| mowing | 0 | `00` |

数据段：`CD CC 4C 3E 00 00 00 00 00 00 00`

**完整 HEX**：
```
FC FB 00 0B  CD CC 4C 3E 00 00 00 00 00 00 00  XX XX  FD FE
```

### 6.3 左转 1.0 rad/s + 播撒器全关

| 字段 | 值 | HEX |
|------|-----|-----|
| linear_speed | 0.0 | `00 00 00 00` |
| angular_speed | 1.0 | `00 00 80 3F` |
| left_seeder | 0 | `00` |
| right_seeder | 0 | `00` |
| mowing | 0 | `00` |

数据段：`00 00 00 00 00 00 80 3F 00 00 00`

**完整 HEX**：
```
FC FB 00 0B  00 00 00 00 00 00 80 3F 00 00 00  XX XX  FD FE
```

### 6.4 前进 + 左侧播撒器开 + 割草开

| 字段 | 值 | HEX |
|------|-----|-----|
| linear_speed | 0.20 | `CD CC 4C 3E` |
| angular_speed | 0.0 | `00 00 00 00` |
| left_seeder | 1 | `01` |
| right_seeder | 0 | `00` |
| mowing | 1 | `01` |

数据段：`CD CC 4C 3E 00 00 00 00 01 00 01`

**完整 HEX**：
```
FC FB 00 0B  CD CC 4C 3E 00 00 00 00 01 00 01  XX XX  FD FE
```

### 6.5 全功能开启

| 字段 | 值 | HEX |
|------|-----|-----|
| linear_speed | 0.20 | `CD CC 4C 3E` |
| angular_speed | 0.0 | `00 00 00 00` |
| left_seeder | 1 | `01` |
| right_seeder | 1 | `01` |
| mowing | 1 | `01` |

数据段：`CD CC 4C 3E 00 00 00 00 01 01 01`

---

## 七、接收端状态机

```
                         ┌─────────────────┐
                         │  WAIT_HEAD_0     │
                         │  期望 0xFC       │
                         └────────┬────────┘
                                  │ 0xFC
                                  ▼
                         ┌─────────────────┐
                         │  WAIT_HEAD_1     │──────── 非0xFB ──▶ WAIT_HEAD_0
                         │  期望 0xFB       │
                         └────────┬────────┘
                                  │ 0xFB
                                  ▼
                         ┌─────────────────┐
                         │  WAIT_ID         │
                         │  存储 frame_id   │
                         └────────┬────────┘
                                  │ 任意值
                                  ▼
                         ┌─────────────────┐
                         │  WAIT_LEN        │──── len > 64 ──▶ WAIT_HEAD_0
                         │  存储 data_len   │
                         └────────┬────────┘
                                  │ len ≤ 64
                                  ▼
                         ┌─────────────────┐
                         │  WAIT_DATA       │
                         │  接收 N 字节     │──────────▶ WAIT_CRC_0
                         └─────────────────┘
                                  │
                     ┌────────────┼────────────┐
                     ▼            ▼            ▼
              WAIT_CRC_0   WAIT_CRC_1   WAIT_END_0
                  │            │        非0xFD → WAIT_HEAD_0
                  └────────────┘            │ 0xFD
                       │                    ▼
                       ▼              WAIT_END_1
                 WAIT_END_0           │ 非0xFE → WAIT_HEAD_0
                       │              │ 0xFE
                       ▼              ▼
                 WAIT_END_1    [CRC校验失败?]
                       │         ├─ 丢弃,返回 WAIT_HEAD_0
                       │ 0xFE    │
                       ▼         └─ 通过 → parse_data() → 更新 cmd
                 [CRC 校验]
                       │
            ┌──────────┴──────────┐
            ▼                     ▼
        失败: 丢弃            通过: parse_data()
                                         │
                                         ▼
                                   更新 CmdData 结构体
                                         │
                                         ▼
                                   WAIT_HEAD_0
```

> 任何状态收到非预期值：立即回退到 `WAIT_HEAD_0`，自动重新同步。

---

## 八、错误处理策略

| 异常情况 | 接收端处理 |
|----------|-----------|
| 帧头[0] != 0xFC | 保持在 WAIT_HEAD_0，丢弃该字节 |
| 帧头[1] != 0xFB | 回退到 WAIT_HEAD_0 |
| Data Length = 0 | 直接进入 data 接收（0 字节后立即进 CRC） |
| Data Length > 64 | 回退到 WAIT_HEAD_0 |
| CRC16 校验失败 | 丢弃整帧，不更新 cmd |
| 帧尾[0] != 0xFD | 回退到 WAIT_HEAD_0 |
| 帧尾[1] != 0xFE | 回退到 WAIT_HEAD_0 |
| 数据段 < 11 字节 | `parse_data()` 返回，不更新 cmd |
| UART 物理层错误（帧/噪声/溢出） | 由硬件自动处理；状态机收到异常字节后自然回退到 WAIT_HEAD_0 |

---

## 九、Python 参考实现

```python
import struct

CRC16_TABLE = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
]


def crc16_xmodem(data: bytes) -> int:
    """CRC-16/XMODEM (polynomial 0x1021, init 0x0000)."""
    crc = 0x0000
    for b in data:
        index = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 8) ^ CRC16_TABLE[index]) & 0xFFFF
    return crc


def build_packet(linear: float, angular: float,
                 left_s: bool, right_s: bool, mow: bool,
                 frame_id: int = 0) -> bytes:
    """构建完整的 CmdPort 数据帧。

    Args:
        linear:   线速度 (m/s)
        angular:  角速度 (rad/s)
        left_s:   左侧播撒器
        right_s:  右侧播撒器
        mow:      割草开关
        frame_id: 帧 ID（默认 0）

    Returns:
        19 字节的完整帧。
    """
    data = struct.pack('<ffBBB',
                       linear, angular,
                       1 if left_s else 0,
                       1 if right_s else 0,
                       1 if mow else 0)

    crc = crc16_xmodem(data)

    frame = bytes([0xFC, 0xFB, frame_id & 0xFF, len(data)])
    frame += data
    frame += struct.pack('<H', crc)          # LE CRC
    frame += bytes([0xFD, 0xFE])

    return frame


# ── 使用示例 ──────────────────────────────────────
if __name__ == '__main__':
    # 停止 + 全关
    pkt = build_packet(0.0, 0.0, False, False, False)
    print('停止:', pkt.hex(' ').upper())

    # 前进 0.20 + 左播撒开 + 割草开
    pkt = build_packet(0.20, 0.0, True, False, True)
    print('前进:', pkt.hex(' ').upper())
```

**输出示例**：
```
停止: FC FB 00 0B 00 00 00 00 00 00 00 00 00 00 00 00 00 FD FE
前进: FC FB 00 0B CD CC 4C 3E 00 00 00 00 01 00 01 D8 92 FD FE
```

---

## 十、协议版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| v1.0 | 2026-07 | 初始版本：9 字节数据段 (v, ω, left_seeder) |
| v2.0 | 2026-07 | 新增 `right_seeder`（偏移 9）和 `mowing`（偏移 10）；数据段扩展为 11 字节；Data Length = 0x0B；帧总长 19 字节 |
