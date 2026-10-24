# sub2rsi wire protocol / 串口线协议

The STM32 master speaks a plain ASCII line protocol over the CH340
USB-serial link: **115200 8N1, one command per line, `\n`-terminated**.
The axis argument defaults to 0 (legacy single-axis scripts keep working).

STM32 主站通过 CH340 USB 转串口提供 ASCII 行协议：**115200 8N1，
每行一条命令，`\n` 结尾**。轴号缺省为 0（兼容旧的单轴写法）。

## Replies / 应答

| Prefix | Meaning |
|---|---|
| `OK` | command accepted |
| `ERR ...` | rejected, human-readable reason follows |
| `STS ...` | status line (see below) |
| `SDO ...` | SDO read result |
| `AXES n=N` | axis count report |
| `EVT MOVE_DONE ax=N` / `EVT FAULT ax=N` | **asynchronous** events, pushed at any time - never a command reply |

## Commands / 命令

| Command | Description |
|---|---|
| `AXES` | returns `AXES n=N` (logical axes = slaves × channels) |
| `EN [ax]` / `DIS [ax]` | enable / disable the drive |
| `RESET [ax]` | CiA402 fault reset (the firmware then rebuilds the link) |
| `VEL [ax] <counts/s>` | constant velocity (0 = stop) |
| `AMOV [ax] <pos> <vel>` | absolute point-to-point (counts, counts/s) |
| `RMOV [ax] <delta> <vel>` | relative point-to-point |
| `HALT [ax]` / `CONT [ax]` | motion latch / release; while latched, motion commands are rejected with `ERR halt active` |
| `STS [ax]` | status line (below) |
| `MON <ms>` (0 = stop) | periodic status push, one line per axis |
| `SDORD <ax> <idx> <sub>` | SDO read → `SDO ax=.. idx=0x.. sub=.. val=0x........ size=N wkc=W` |
| `SDOWR <ax> <idx> <sub> <size 1-4> <val>` | SDO write |
| `ODLIST [ax]` | incremental object-dictionary dump (`OD n 0xXXXX subM name` per line, ends with `OD END`; PDO keeps flowing) |
| `REINIT` | full link rebuild (MCU reset, back in OP in ~5 s) |
| `HELP` | command list |

## Status line / 状态行

```text
STS ax=0 sw=0x1637 rdy=1 on=1 en=1 fault=0 warn=0 tgt=1 pos=-4600 vel=0 err=0x0000 mode=CSP al=OP wkc=3
```

| Field | Meaning |
|---|---|
| `sw` | CiA402 statusword (0x6041 via PDO) |
| `rdy/on/en/fault/warn/tgt` | decoded statusword bits: ready / switched-on / operation-enabled / fault / warning / target-reached |
| `pos` / `vel` | actual position (counts) / commanded stream velocity (counts/s) |
| `err` | CiA402 error code 0x603F (the drive latches the most recent code - a non-zero value with `fault=0` is history, not a live fault) |
| `mode` | operating mode display (CSP here) |
| `al` | EtherCAT AL state (`OP` expected) |
| `wkc` | working counter of the last cycle |

## Multi-channel drives / 一拖四多通道寻址

A one-to-four drive is **one EtherCAT slave** hosting several CiA402
channels. Channel ch lives at object index `base + 0x800×ch`:

| Channel | controlword | statusword | target pos | actual pos | error code | mode |
|---|---|---|---|---|---|---|
| ch0 | 0x6040 | 0x6041 | 0x607A | 0x6064 | 0x603F | 0x6060/0x6061 |
| ch1 | 0x6840 | 0x6841 | 0x687A | 0x6864 | 0x683F | 0x6860/0x6861 |
| ch2 | 0x7040 | 0x7041 | 0x707A | 0x7064 | 0x703F | 0x7060/0x7061 |
| ch3 | 0x7840 | 0x7841 | 0x787A | 0x7864 | 0x783F | 0x7860/0x7861 |

`SDORD`/`SDOWR` take **raw indices** - address channel 1 explicitly with
`0x6841` etc. The firmware maps logical axis `ax` to slave
`1 + ax/channels` automatically and probes the channel count (0x6841 /
0x7041 existence) in PRE-OP.

## SDO budget / SDO 预算

The drive's software ESC watchdogs its process-data mailbox at 100 ms.
The firmware bounds every SDO transaction to ≤ 30 ms (writes retry up to
3 times, ≤ 90 ms worst case). **A nonexistent object gets no reply at
all** (the drive stays silent; the read fails with a timeout error) -
that is normal. Keep SDO for low-rate configuration and diagnostics;
never poll it inside a real-time loop.
