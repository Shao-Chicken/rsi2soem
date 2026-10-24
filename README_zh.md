# rsi2soem

**纯 Python 的 RSI RMP 风格运动控制库——通过 USB 串口 SOEM 主站控制 EtherCAT 机台。**

`rsi2soem`（RSI 风格 API，由 SOEM 主站驱动）让 PC 在**没有 EtherCAT 主站卡、
没有实时操作系统**的情况下控制 CiA402 伺服/步进驱动器：一块廉价的 STM32H743 开发板
运行开源 [SOEM](https://github.com/OpenEtherCATsociety/SOEM) 主站协议栈，
通过 CH340 USB 转串口暴露一套 ASCII 行协议；本库把这套协议包装成对 RSI RMP /
RapidCode API 的忠实复刻（`MotionController` / `Axis` / `MultiAxis` / `NetworkNode`），
为 RSI 硬件编写的代码几乎可以原样迁移。

[English README](README.md)

## 特点

- **无主站卡、无 RTOS、无厂商运行时**——一根 USB 线即可。
- **方法名与 RMP 一致**——RapidCode 程序快速移植。
- **支持"一拖四"多通道驱动器**——单个 EtherCAT 从站内多个 CiA402 通道
  （索引偏移 `+0x800×通道`）自动探测，各轴就是普通的 `AxisGet(0..2)`。
- **默认安全**——软限位、速度上限、使能检查、固件侧跟随误差看门狗。
- **不装懂**——固件做不到的功能（插补、PT/PVT 流式、齿轮凸轮……）以同名存根
  抛出 `RsiNotImplementedError`，绝不静默出错。

## 安装

```bash
pip install rsi2soem          # PyPI（发布后）
pip install .                # 本仓库克隆
pip install .[test]          # 含测试依赖
```

## 一分钟上手

```python
from rsi2soem import MotionController

AXES = {0: ("Z", (0.0, 91.5)),   # 轴名 + 软限位 (mm)
        1: ("X", (0.0, 110.0)),
        2: ("Y", (0.0, 111.0))}

with MotionController.create("COM12", axes_config=AXES) as mc:
    print(mc.AxisCountGet(), "轴, 网络:", mc.NetworkStateGet())

    z = mc.AxisGet("Z")
    z.PositionSet(0.0)            # 在当前位置标零
    z.AmpEnableSet(True)

    z.MoveTrapezoidal(10.0, 5.0)  # 绝对运动: 10 mm, 5 mm/s
    z.MotionDoneWait(15000)
    print("Z 位于", z.ActualPositionGet(), "mm")

    z.MoveRelative(-2.0)          # 相对运动 (默认速度)
    z.MoveVelocity(0.5)           # 恒速点动
    z.Stop(); z.Resume()          # 停止闩锁 / 解除

    z.AmpEnableSet(False)
```

## 测试

```bash
pytest                        # 43 项 mock 串口单元测试（无需硬件, CI 可跑）
pytest --ecat-port COM12      # + 30 项真机硬件在环测试
```

## 文档

- [完整中文 API 文档](docs/api-zh.md)
- [Full API reference (English)](docs/api-reference.md)
- [串口线协议](docs/protocol.md)
- [examples/](examples/)——快速上手、三轴机台配置、手动回零

## 许可证

MIT——见 [LICENSE](LICENSE)。
