# rsi2soem — RMP 风格运动控制 API 完整文档（v0.1）

> 对应英文版：[api-reference.md](api-reference.md)。
> 本文档以"参考机台"（MOONS' 一拖四步进伺服 + STM32H743 SOEM 主站）为实测背景，
> 所有结论均来自真机验证。

## 目录

1. 系统架构
2. 机台轴映射与坐标系（含一拖四多通道寻址）
3. RSI RMP 功能覆盖总表
4. 安装与 30 秒上手
5. API 参考：MotionController
6. API 参考：Axis
7. API 参考：MultiAxis 与 NetworkNode
8. 安全机制与机械顶死根因报告（必读）
9. 底层串口协议
10. 测试与硬件实测结果
11. 异常体系
12. 常见问题 FAQ

## 1. 系统架构

```text
┌─────────────────────────────┐
│  PC 上位机 (Windows/Linux)   │
│  Python → rsi2soem 包        │   本 API, 对应 RMP 的 RapidCode 层
└──────────────┬──────────────┘
               │ CH340 USB 转串口  115200 8N1
               ▼
┌─────────────────────────────┐
│  STM32H743 SOEM 主站开发板    │   对应 RMP 的实时固件层
│  SOEM EtherCAT Master       │   1 ms 周期, CSP 目标位置流
└──────────────┬──────────────┘
               │ EtherCAT (100 Mbit, 网线接驱动器 EtherCAT IN)
               ▼
┌─────────────────────────────┐
│  MOONS' AMA 一拖四步进伺服     │   1 个 EtherCAT 从站, 4 个 CiA402 通道
│  ch0=Z  ch1=X  ch2=Y  ch3空闲 │   通道索引 = 标准索引 + 0x800×ch
└─────────────────────────────┘
```

- 上电后固件自动完成 EtherCAT 扫描、**通道数探测（0x6841/0x7041）**、PDO 重映射
  （每通道 cw+目标位置 / sw+实际位置）并进入 **OP**（约 2~3 秒），无需任何初始化命令。
- 固件按探测到的通道数自适应（1~3 通道），X、Y 电机插上驱动器后重启即可识别，
  上位机零改动可用。
- 实时闭环在驱动器内部完成；STM32 主站每 1 ms 同步一次三通道目标位置（CSP 流）。

## 2. 机台轴映射与坐标系

### 2.1 轴映射（axes_config）

开源库本身不带机台绑定：轴命名和软限位通过 `axes_config` 注入。
参考机台的配置：

```python
MACHINE_AXES = {
    0: ("Z", (0.0, 91.5)),    # Z 底部为零点 -> 正向向上 (行程实测 92.1 mm)
    1: ("X", (0.0, 110.0)),   # X 左侧为零点 -> 正向向右 (行程实测 110.7 mm)
    2: ("Y", (0.0, 111.0)),   # Y 上侧为零点 -> 正向向下 (行程实测 111.5 mm)
}
mc = MotionController.create("COM12", axes_config=MACHINE_AXES)
```

| 轴号 | 名称 | 驱动器通道 | 零点 | 正方向 | 软限位 (mm) |
|---|---|---|---|---|---|
| 0 | **Z** | ch0（0x6000 区） | 底部挡块 | 向上 | `[0, +91.5]` |
| 1 | **X** | ch1（0x6800 区） | 左挡块 | 向右 | `[0, +110.0]` |
| 2 | **Y** | ch2（0x7000 区） | 上挡块 | 向下 | `[0, +111.0]` |

> 行程均为 2026-10-23 轻柔停滞法实测（0.5 mm/s 慢爬，无顶死）。
> ch3（0x7800 区）空闲未接电机。不传 `axes_config` 时轴名为 AX0..AXn、
> 软限位禁用（此时**没有行程保护**，务必自行设置 `software_limits_set`）。

### 2.1.1 一拖四多通道寻址（关键拓扑）

参考机台的 Z/X/Y **不是三台独立驱动器级联**，而是**一台"一拖四"多轴驱动器**：

- EtherCAT 链上永远只有 **1 个从站**（VendorID `0x168`，ProductCode `0x12`）。
- 内部 4 个完整 CiA402 轴通道，**每通道对象索引偏移 +0x800**：

| 通道 | 接的轴 | 控制字 | 状态字 | 目标位置 | 实际位置 | 错误码 | 模式 |
|---|---|---|---|---|---|---|---|
| ch0 | Z | 0x6040 | 0x6041 | 0x607A | 0x6064 | 0x603F | 0x6060/61 |
| ch1 | X | 0x6840 | 0x6841 | 0x687A | 0x6864 | 0x683F | 0x6860/61 |
| ch2 | Y | 0x7040 | 0x7041 | 0x707A | 0x7064 | 0x703F | 0x7060/61 |
| ch3 | （空闲） | 0x7840 | 0x7841 | 0x787A | 0x7864 | 0x783F | 0x7860/61 |

- 固件在 PRE-OP 自动探测 0x6841/0x7041 判定通道数，PDO 映射为
  **每通道 2 条**（cw+目标位置 / sw+实际位置），三通道共 18 字节进出。
  条数变多曾担心触发该软 ESC 的 PDO 丢失问题——固件带了**自动降级**：
  映射被拒或进不了 OP 时通道上限自动减 1（最低 1），实测三通道稳定运行。
- 上层 API **完全透明**：`AxisGet(0/1/2)` 就是 Z/X/Y，无需关心通道偏移；
  只有用 `NetworkNode.SDORead/SDOWrite` 裸读写时才需要按上表选索引
  （便捷方法 `status_word()` 等已自动加偏移）。
- 0x603F 错误码会被驱动器**锁存保留**（如 X/Y 曾报 0xFF36=电机未接），
  故障复位后状态字已正常但历史码仍可读——判断当前是否故障看
  `StatusGet().fault`，不要只看错误码。

### 2.2 单位约定

| 量 | 约定 |
|---|---|
| 丝杆导程 | **2 mm / rev**（三轴相同） |
| 每转脉冲数 | **2048 counts / rev**（RSI User Units 基准） |
| 默认用户单位 | **1 user unit = 1 mm**，即 `1024 counts = 1 mm` |
| 速度单位 | user units / s（默认 mm/s） |

想 "1 单位 = 1 转 = 2 mm"：`ax.UserUnitsSet(2048)`，软限位与速度自动随动换算。

### 2.3 坐标零点与标定（每轴必做）

编码器读数是驱动器内部的原始 counts，**掉电/手推后都会变**。
每次上电、以及确认轴向后的第一件事，是对每根轴做标定：

```python
ax = mc.axis_get('Z')
# 1. 先慢速把轴推到行程一端（见 8.3 慢爬探测法, 或 examples/limit_homing.py）
# 2. 到达端点后:
ax.PositionSet(0.0)     # 定义该端点为用户坐标 0
# 3. 按实测行程设置软限位, 撞墙侧留 ≥0.4 mm 余量
ax.software_limits_set(0.0, 实测行程mm - 0.4)
```

> ⚠️ 零点偏移只保存在 API 对象里（掉电不保存）。软限位随零点平移，
> 重新标定零点后必须重新确认软限位。

## 3. RSI RMP 功能覆盖总表

依据 RSI 官方文档（support.roboticsys.com/rmp，11.x）。✅ 已支持 / ⚠️ 部分 / ❌ 存根（调
用抛 `RsiNotImplementedError`，见 6.6）。

### 3.1 MotionController

| RMP 功能 | 本实现 |
|---|---|
| Create / 自动找串口 | ✅ `MotionController.create("COM12")` |
| AxisGet（序号或名称）/ AxisCountGet | ✅ `axis_get()` / `z_axis` `x_axis` `y_axis` 属性 |
| MultiAxisGet | ✅ `multi_axis_get([0,1,2])` |
| NetworkNodeGet / 任意 SDO 读写 | ✅ `network_node_get()` → `SDORead/SDOWrite` |
| NetworkStart / StateGet / Shutdown | ✅ 上电自动 OP；`network_state_get()` / `network_wait_op()` |
| SampleRateGet | ✅ 1000 Hz（固件 1 ms 周期） |
| NetworkEniGenerate | ❌（固件内建配置，无需 ENI） |
| Recorder / UserLimit / Compensator / MathBlock / Interrupt | ❌ 存根 |
| Trace / ErrorLog | ⚠️ ErrorLog 空实现；固件事件环可经调试通道读 |

### 3.2 Axis（单轴）——核心全部对齐

| RMP 功能 | 本实现 |
|---|---|
| **MoveTrapezoidal / MoveRelative / MoveVelocity** | ✅（软限位校验后下发） |
| **MoveSCurve / MoveVelocitySCurve** | ⚠️ 驱动器内部平滑，jerk 参数忽略 |
| Stop / Resume / Abort / EStop 系列 / TriggeredModify | ✅（Stop 闩锁，Resume 解除） |
| AmpEnableSet / ClearFaults | ✅（ClearFaults 后链路自动重建回 OP） |
| StateGet / StatusWordGet / StatusBitGet / StatusBitsGet / SourceGet | ✅（CiA402 状态字实时解码） |
| MotionDoneGet / MotionDoneWait | ✅（-1=无限等待；超时抛 `MotionTimeoutError`） |
| PositionSet / ActualPositionGet / EncoderPositionGet / TargetPositionGet / PositionErrorGet | ✅ |
| UserUnitsGet/Set、CountsToUserUnits | ✅（改单位后限位/速度容差随动） |
| SoftwarePos/NegLimitTriggerValue(Get/Set) | ✅（上位机强制，越限指令拒发） |
| NegativeLimitGet / PositiveLimitGet | ⚠️ 无硬件开关，用软限位状态代替（±10 counts 容差） |
| Home / HomeStateGet/Set | ⚠️ 无原点开关；`Home()` 存根，用 `PositionSet(0)` 手动回零 |
| FeedRateSet (0~2) / MotionDelaySet | ✅ 实际生效（速度倍率/运动前延时） |
| Default* 系列、Settling、UserLabel、MotionId、StopTime 等 | ✅ 存取（参数集保存在 API 对象） |
| MovePT/PVT/PVAJT/PTF/PVTF（流式点位） | ❌ 存根 |
| Gearing / Camming / FilterCoeff / Backlash | ❌ 存根（闭环在驱动器内部） |
| 专用/通用/模拟 IO | ❌ 存根（线缆未接 IO） |

### 3.3 MultiAxis / 其他

| 模块 | 本实现 |
|---|---|
| MultiAxis Map/Unmap/IsMapped | ✅ |
| MultiAxis 点位/速度/Stop/AmpEnable | ✅（各轴**同时启动但非插补**，各自独立完成） |
| 直线/圆弧插补、Robot、Gcode | ❌ |
| NetworkNode SDORead/SDOWrite + 便捷读取 | ✅（见 7.2） |

## 4. 安装与 30 秒上手

```bash
pip install rsi2soem          # 或 pip install . （仓库克隆）
```

```python
from rsi2soem import MotionController

AXES = {0: ("Z", (0.0, 91.5)), 1: ("X", (0.0, 110.0)), 2: ("Y", (0.0, 111.0))}

with MotionController.create("COM12", axes_config=AXES) as mc:
    print("轴数:", mc.AxisCountGet())
    z = mc.axis_get('Z')          # 或 mc.axis_get(0)
    z.PositionSet(0.0)            # 当前位置定义为零点(先确认轴在安全端点!)
    z.AmpEnableSet(True)          # 使能
    z.MoveTrapezoidal(2.0, 1.0)   # 走到 2 mm, 1 mm/s
    x = z.MotionDoneWait(15000)   # 等待到位(毫秒)
    print("到位: %.3f mm" % x)
    z.AmpEnableSet(False)
```

## 5. API 参考：MotionController

| 方法 | 说明 |
|---|---|
| `create(port=None, baud=115200, axes_config=None)` | 创建并连接；`port` 省略时自动找 CH340/USB Serial 串口；`axes_config` 注入轴命名与软限位 |
| `axis_get(axis=0)` / `AxisGet` | 取轴对象：`0/1/2` 或已配置的名称（大小写均可）；越界抛 `AxisNotPresentError` |
| `z_axis` / `x_axis` / `y_axis` 属性 | 按机台名取轴（需 axes_config 配置了对应名称） |
| `AxisCountGet()` | 链上逻辑轴数（参考机台=3） |
| `ChannelsPerSlaveGet()` | 每个 EtherCAT 节点内的 CiA402 通道数（自动探测；一拖四=3，级联单通道驱动器=1） |
| `multi_axis_get(axes=None)` / `MultiAxisGet` | 取多轴对象（默认全部轴） |
| `network_node_get(node=0)` / `NetworkNodeGet` | 取节点（SDO 通道），node=轴号 |
| `network_state_get()` / `NetworkStateGet` | `'OP'/'SAFEOP'/'PREOP'/'INIT'` |
| `network_wait_op(timeout=15)` / `NetworkStart` | 阻塞等 OP（复位/恢复后用），超时抛 `NetworkError` |
| `network_shutdown()` / `NetworkShutdown` | 全部轴去使能（链路保持） |
| `SampleRateGet()` | 1000（固件周期 Hz） |
| `FirmwareVersionGet()` / `VersionGet()` | 固件 / 本库版本 |
| `ErrorLogCountGet/Get/Clear` | 空实现（错误以异常抛出，不留日志） |
| `ThrowExceptions(True)` | 仅接受 True（本库始终抛异常） |
| `raw_command(line, timeout=3)` | 发一行底层协议命令（第 9 节），返回应答 |
| `close()` / `Delete()` | 关闭；支持 `with` |

存根（抛 `RsiNotImplementedError`）：`RecorderCountGet`、`UserLimitCountGet`、
`CompensatorCountGet`、`MathBlockCountGet`、`InterruptWait`、`MemoryGet`、`Trace`、
`SerialNumberGet`。

## 6. API 参考：Axis

所有位置/速度默认单位 mm / mm/s（改过 user units 后按新单位）。

### 6.1 运动

| 方法 | 说明 |
|---|---|
| `MoveTrapezoidal(x, vel=None)` | 绝对点位；`vel=None` 用 `DefaultVelocity`；越软限位抛 `SoftLimitError`（指令不下发） |
| `MoveRelative(dx, vel=None)` | 相对点位 |
| `MoveSCurve(x, vel=None)` | S 曲线（驱动器内部平滑） |
| `MoveVelocity(v)` / `MoveVelocitySCurve(v)` | 恒速 jog，`0`=停；朝限位外 jog 抛 `SoftLimitError` |
| `Stop()` | 减速停并闩锁；之后运动需先 `Resume()` |
| `Resume()` | 解除闩锁 |
| `Abort()` / `EStop()` / `EStopAbort()` / `EStopModify*` / `TriggeredModify()` | 等价 `Stop()` |
| `MotionDoneWait(ms=-1)` | 等到位，返回最终位置；`-1` 无限等；超时抛 `MotionTimeoutError` |
| `MotionDoneGet()` | 非阻塞到位查询；故障抛 `FaultError` |

### 6.2 使能与故障

| 方法 | 说明 |
|---|---|
| `AmpEnableSet(True/False)` | 使能/去使能；未生效抛 `RsiError` |
| `AmpEnableGet()` / `AmpFaultGet()` | 使能态/故障态查询 |
| `ClearFaults()` | 故障复位；固件自动重建链路回 OP（~5 s）；仍 fault 抛 `FaultError` |

### 6.3 状态与位置

| 方法 | 返回 |
|---|---|
| `StatusGet()` / `status_get()` | `AxisStatus` 快照：status_word/enabled/fault/pos/vel/err/al_state/wkc 等全字段 |
| `StateGet()` | `CiaState`：`OPERATION_ENABLED` 等（对应 RSIState） |
| `StatusWordGet()` / `StatusBitGet(mask)` / `StatusBitsGet()` | CiA402 状态字 / 位查询 |
| `SourceGet()` | `'AXIS'`（故障时）/ `'NONE'` |
| `ActualPositionGet()` | 实际位置（user units） |
| `PositionSet(x)` / `ActualPositionSet(x)` / `CommandPositionSet(x)` | 设当前坐标（零点标定） |
| `TargetPositionGet()` / `CommandPositionGet()` | 指令位置 |
| `PositionErrorGet()` | 跟随误差（user units） |
| `EncoderPositionGet()` | 编码器原始 counts |
| `ActualVelocityGet()` / `CommandVelocityGet()` | 流速（user units/s） |
| `error_code_get()` | 驱动器 0x603F 错误码（注意：驱动器锁存历史码，看故障要用 `StatusGet().fault`） |

### 6.4 配置

| 方法 | 说明 |
|---|---|
| `UserUnitsGet/Set(n)` | 1 user unit = n counts（默认 1024=1mm）；限位/容差随动 |
| `UserUnitsToCounts(x)` / `CountsToUserUnits(c)` | 单位换算 |
| `software_limits_set(lo, hi)` / `software_limits_get()` / `software_limits_clear()` | 软限位读写/禁用；未配置时**禁用**（无行程检查） |
| `SoftwarePos/NegLimitTriggerValueGet/Set` | 软限位的 RMP 命名读写 |
| `NegativeLimitGet()` / `PositiveLimitGet()` | 软限位触发状态（±10 counts 容差；限位禁用恒 False） |
| `max_velocity_set(v)` | 速度上限（默认 5 mm/s） |
| `FeedRateGet/Set(0~2)` | 进给倍率，实际生效（0.5=速度减半） |
| `MotionDelayGet/Set(s)` | 每次运动前延时，实际生效 |
| `DefaultVelocitySet` 等 `Default*` 存取 | 缺省运动参数（`vel=None` 时生效） |
| `SettlingTimeGet/Set`、`PositionTolerance*`、`VelocityTolerance*` | 到位判定参数 |
| `HomeStateGet/Set`、`HomeMethodGet/Set`、`HomeOffsetGet/Set`、`OriginPositionGet/Set` | 回零标记/原点偏置记录 |
| `UserLabelGet/Set`、`MotionIdGet/Set`、`StopTimeGet/Set`、`EStopTimeGet/Set`、`EStopDecelerationGet/Set` | 记录型配置 |
| `ErrorLimitTriggerValueGet` | 跟随误差阈值（固件固定 20000 counts） |

### 6.5 操作模式

`OperationModeGet()` → `RSIOperationMode.CSP`（本系统只支持 CSP）；
`OperationModeSet(其他)` 抛 `RsiNotImplementedError`。

### 6.6 存根（调用抛 `RsiNotImplementedError`）

`Home()`、`MovePT/PVT/PVAJT/PTF/PVTF`、`Gearing*`、`MoveCam*`、
`FilterCoeffGet/Set`、`BacklashWidthGet/Set`、`DedicatedIn/Out`、`DigitalIn/Out`、
`AnalogInGet`、`HomeLimitGet`、`HomeSwitchGet`。

> 存根是**显式拒绝**而非静默吞掉：移植 RSI 代码时一调就知道，不会误以为已生效。

## 7. API 参考：MultiAxis 与 NetworkNode

### 7.1 MultiAxis（多轴同时启动，非插补）

```python
ma = mc.multi_axis_get()          # 默认映射全部轴
ma.Map(); ma.IsMapped()
ma.AmpEnableSet(True)             # 全部轴一起使能
ma.MoveTrapezoidal([2.0, 5.0, 3.0], 1.0)   # 各轴同时启动, 各自独立到位
ma.MotionDoneWait(15000)
xs = ma.ActualPositionGet()       # [x_z, x_x, x_y]
ma.Stop()                         # 全部轴停
ma.Unmap()                        # 解映射后运动调用抛 RsiError
```

> 注意：这是"同时启动"，不是直线/圆弧插补。

### 7.2 NetworkNode（任意 SDO 读写）

```python
node = mc.network_node_get(0)     # 轴0 (Z, 通道 ch0)
sw   = node.SDORead(0x6041, 0, 2)       # 状态字 (raw 索引, 原样下发)
pos  = node.SDORead(0x6064, 0, 4)       # 实际位置
mode = node.SDORead(0x6061, 0, 1)       # 模式显示 (=8 CSP)
node.SDOWrite(0x6081, 0, 4, 10000)      # 写 profile velocity
# 便捷封装: node.status_word() / node.position_actual() / node.mode_display() ...
# 便捷封装已按节点号自动加通道偏移: network_node_get(1).status_word() 读 0x6841 (X)
```

> **一拖四寻址**：`SDORead/SDOWrite` 接受 **raw 索引**——读 X 通道用 `0x6841`、
> Y 通道用 `0x7041`（完整对照表见 2.1.1 节）。便捷方法（`status_word()` 等）
> 自动按节点号加偏移。`mc.ChannelsPerSlaveGet()` 返回探测到的每节点通道数。
>
> **SDO 使用注意**：固件把每次 SDO 事务限制在 ≤30 ms（写最多重试 3 次共 ≤90 ms），
> 以保护驱动器 100 ms 的 SM 看门狗。**不存在的对象**驱动器会直接沉默（返回
> `CommunicationError`），属正常现象；SDO 适合低频配置/诊断，不要放进实时循环。

## 8. 安全机制与机械顶死根因报告（必读）

### 8.1 五层保护

| 机制 | 位置 | 行为 |
|---|---|---|
| 上位机软限位 | rsi2soem 库 | 越限指令**根本不下发**，抛 `SoftLimitError` |
| 速度上限 | rsi2soem 库 | 默认 5 mm/s，超过拒绝 |
| 未使能保护 | rsi2soem 库 | 未使能发运动抛 `NotEnabledError` |
| 固件跟随误差看门狗 | STM32 固件 | 目标与实际差 >20000 counts 持续 0.5 s → 钳住目标流并停运动（顶墙/卡死的最后防线） |
| 故障自动恢复 | STM32 固件 | 驱动器 fault → 复位后自动重建链路；`REINIT` 命令可随时手动重建（MCU 复位，~5 s 回 OP） |

### 8.2 根因报告：曾被误诊为"驱动器退化"的机械顶死

2026-10-23 排查实录（重要教训，症状极易误判为通信故障）：

- **现象**：电机只能跟随目标流一小段就停（sw=0x1237、到位标志丢失、后续 AMOV "假死"）。
- **实测排除**：EtherCAT 循环 1000/s 稳定、wkc=3、PDO 目标流完全正确、SDO 全部正常、
  0x603F=0 无故障、MCU 复位与驱动器 ESC 软复位都无效。
- **真因**：**Z 轴滑台物理顶在上限位硬挡块上**。证据链：
  1. 实际电流 0x6078 顶死在 999‰（电机全力推，推不动）；
  2. 失能(DIS)瞬间滑台因重力**下滑 ~0.65 mm**（Z 是竖直轴）；
  3. 重新使能后向上爬到同一位置再次顶死；
  4. 向下慢爬 **4.6 mm 全程顺畅**，无任何卡顿。
- **结论**：固件/链路/API 全无问题。之前的"SDO 读失败导致退化""CW bit7 隐藏复位"等
  推断均为顶死的伴随假象（目标流跑远后驱动器按跟随误差窗口脱开，看起来像链路坏了）。

### 8.3 新轴接入后的标准做法（限位探测与标定）

```python
# 慢爬探测法: 用小速度爬, 位置连续几次采样不变 = 到端点(机械挡块)
ax.MoveVelocity(0.2)      # 朝一个方向 0.2 mm/s 慢爬
# 轮询 ActualPositionGet(), 停滞即端点; Stop() + Resume() 停下
# 反向再探另一端, 得到全程行程 L
ax.PositionSet(0.0)                       # 在选定端点标零
ax.software_limits_set(0.0, L - 0.4)      # 留 0.4 mm 余量
```

完整实现见 [examples/limit_homing.py](../examples/limit_homing.py)。

- 竖直轴（Z）**失能会下滑**，标定和断电前注意托住或先停到安全位。
- 任何"电机不动了"的现象，先看 `ActualPositionGet()` 是否顶在行程端点、
  `StatusGet().fault` 是否置位——不要盲目复位链路。

## 9. 底层串口协议

115200 8N1，ASCII 行协议（`\n` 结尾）。轴号缺省 = 0（兼容旧单轴写法）。
完整协议（含状态行字段、多通道寻址、SDO 预算）见 [protocol.md](protocol.md)，
核心命令速查：

| 命令 | 说明 |
|---|---|
| `AXES` | 返回 `AXES n=N`（逻辑轴数 = 从站数 × 通道数） |
| `EN [ax]` / `DIS [ax]` | 使能 / 去使能 |
| `RESET [ax]` | 故障复位（固件随后自动重建链路） |
| `VEL [ax] <counts/s>` | 恒速（0=停） |
| `AMOV [ax] <pos> <vel>` | 绝对点位（counts, counts/s） |
| `RMOV [ax] <delta> <vel>` | 相对点位 |
| `HALT [ax]` / `CONT [ax]` | 运动闩锁 / 解除 |
| `STS [ax]` | 状态行：`sw=0x.. pos=.. vel=.. err=0x.. mode=CSP al=OP wkc=3` |
| `MON <ms>` (0=停) | 周期输出所有轴状态（每轴一行） |
| `SDORD <ax> <idx> <sub>` | SDO 读（idx 为 raw 索引，通道对照见 2.1.1） |
| `SDOWR <ax> <idx> <sub> <size1-4> <val>` | SDO 写 |
| `ODLIST [ax]` | 增量导出对象字典（PDO 不断流） |
| `REINIT` | 全链路重建（MCU 复位方式，~5 s 回 OP） |
| `HELP` | 命令清单 |

事件推送：`EVT MOVE_DONE ax=N` / `EVT FAULT ax=N`（库已自动过滤，不会混进命令应答）。

## 10. 测试与硬件实测结果

两层测试（全部通过，2026-10-23/24）：

```bash
pytest                          # 43 项 mock 串口单元测试 (无需硬件, CI 可跑)
pytest --ecat-port COM12        # + 30 项真机硬件在环测试
```

| 层 | 覆盖 | 结果 |
|---|---|---|
| 单元测试 (43 项) | 控制器创建/通道探测/轴查找/单位换算/软限位拒绝/使能流程/全部运动命令的线上格式/Stop闩锁/超时/MultiAxis/SDO 解析与通道偏移/存根/异常/事件行过滤/无应答超时 | ✅ 43/43 |
| 硬件在环 (30 项) | MotionController 全表/Axis 使能/软限位/点位/相对/S曲线/恒速 jog/Stop/Resume/Abort/超时/FeedRate 计时/MotionDelay 计时/UserUnits 切换/配置存取/存根/SDO/通道偏移比对/MultiAxis/收尾停靠 | ✅ 30/30（17.6 s） |

三通道独立运动验证（固件级）：三轴 `EN` 全部 sw=0x1637（CSP/OP），
各轴 `RMOV ±200 counts` 独立运动、互不影响，位置回读精确归零。

三轴行程实测（轻柔停滞法 0.5 mm/s）：Z 92.1 mm / X 110.7 mm / Y 111.5 mm。

运动精度抽样：`MoveTrapezoidal(1.0mm)` 到位 1.007 mm；`MoveRelative ±1.0mm`
往返误差 ≤0.005 mm；`MoveVelocity(0.5mm/s)` 实测速度 0.500 mm/s。

## 11. 异常体系

```python
RsiError                      # 总基类
├─ CommunicationError         # 串口/SDO 通讯失败(含对象不存在)
├─ NetworkError               # 网络未进 OP 等
├─ SoftLimitError             # 软限位/速度上限拒绝
├─ FaultError                 # 驱动器故障, 带 .error_code (0x603F)
├─ MotionTimeoutError         # MotionDoneWait 超时
├─ NotEnabledError            # 未使能发运动
├─ AxisNotPresentError        # 轴号越界/未知名
└─ RsiNotImplementedError     # 存根功能(也是 NotImplementedError)
```

## 12. 常见问题 FAQ

**Q: 电机"不动"了，是不是链路坏了？**
A: 九成不是。按顺序查：① `ActualPositionGet()` 是否顶在行程端点（机械顶死，见 8.2）；
② `StateGet()` 是否使能、是否 Stop 闩锁（要 `Resume()`）；③ `StatusGet().fault`
置位 → `ClearFaults()`；④ 以上都正常仍不动 → `raw_command("REINIT")` 重建链路。

**Q: 断电后位置变了？**
A: 编码器是相对的，手推/旋钮/竖直轴下滑都会变。重新做"慢爬到端点 → `PositionSet(0)`"
标定（见 8.3），零点不贵，勤标定。

**Q: SDO 读某对象报 `CommunicationError`？**
A: 该对象在这台驱动器上不存在（它直接沉默不应答）。这是驱动器特性，不影响链路和
运动；固件已把单次 SDO 限制在 30 ms 内保护看门狗。

**Q: `error_code_get()` 读出非 0 但电机好好的？**
A: 0x603F 是驱动器锁存的"最近一次"错误码，故障清除后仍然可读（如 X/Y 曾报
0xFF36=电机未接）。判断当前故障用 `StatusGet().fault`。

**Q: 一拖四驱动器只认出一个轴？**
A: X/Y 电机没插或插了但驱动器没重启——通道故障（0xFF36）在电机缺接时锁存，
插上电机后重启驱动器即可；固件上电自动探测通道数并复位各通道故障。

**Q: 三轴怎么一起动？**
A: `mc.multi_axis_get()` 得全部轴组，`MoveTrapezoidal([z, x, y], v)` 同时启动
（各轴独立规划，非插补）。
