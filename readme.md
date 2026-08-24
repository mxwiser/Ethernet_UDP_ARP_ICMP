# EP4CE10 + RTL8201F 100M 以太网 UDP 回环

本项目基于 Intel Cyclone IV E `EP4CE10F17C8` 和 RTL8201F，以 100BASE-TX MII 接口实现 IPv4、ARP、ICMP Echo 和 UDP 收发。当前 `dev` 分支的 Quartus 顶层为 `top_mii`，接收到并校验通过的 UDP 数据包会经过事务式缓存后回发给发送端。

## 当前工程配置

| 项目 | 配置 |
| --- | --- |
| FPGA | EP4CE10F17C8 |
| Quartus 工程 | `ep4ce10.qpf` / `ep4ce10.qsf` |
| 当前顶层 | `top_mii` |
| PHY | RTL8201F，Clause 22 PHY 地址为 1 |
| PHY 接口 | 100 Mbps MII，4 bit，TXC/RXC 为 25 MHz |
| 系统时钟 | `clk`，50 MHz |
| 板卡 MAC | `50:12:22:33:44:55` |
| 板卡 IPv4 地址 | `10.10.1.10` |
| 协商模式 | 仅通告 100BASE-TX 全双工 |

MII 时序约束位于 `ep4ce10.out.sdc`。其中系统时钟、MII TX 时钟和 MII RX 时钟按异步时钟域处理，跨时钟数据均通过异步 FIFO。

## 功能

- ARP 请求解析与应答。
- ICMP Echo Request 解析与 Echo Reply，支持 `ping`。
- UDP 接收、发送及数据回环。
- IPv4 首部校验和与以太网 FCS 校验。
- 完整帧校验通过后才提交到 UDP 回环缓存；坏包或溢出的当前数据包会整体回滚。
- TX 支持 IPv4 分片；单个未分片 UDP 负载上限为 1472 字节，超过后由 TX 拆成多个 IPv4 分片。
- RX 不支持 IPv4 分片重组：`fragment offset != 0` 或设置 `MF` 的数据包会被丢弃，不会交给 UDP 用户接口。
- 同时保留 MII 和 RMII PHY/AXIS 转换模块，但当前工程实际使用 MII。

## 数据通路

```text
RX:
RTL8201F MII
  -> phy_mii_axis
  -> rx_cdc_fifo_axis          (mii_rxc -> sys_clk)
  -> udp_axis_rx               (ARP / ICMP / UDP 解析与校验)
  -> udp_ring                  (完整 UDP 包事务式缓存)

TX:
udp_ring
  -> udp_axis_tx               (UDP / IPv4 / Ethernet 封装，可进行 TX 分片)
  -> tx_cdc_fifo_axis          (sys_clk -> mii_txc，ARP/ICMP 与 UDP 仲裁)
  -> phy_mii_axis
  -> RTL8201F MII
```

`udp_ring` 默认提供 2048 字节数据缓存和 64 项包元数据缓存。`rx_cdc_fifo_axis` 为 1024 项异步 FIFO；`tx_cdc_fifo_axis` 的系统应答通道和 UDP 通道各使用一个 1024 项异步 FIFO。

## AXI-Stream 帧边界约定

本项目的 `tlast` 不是标准 AXI-Stream 的“仅最后一个字节拉高”，而是帧级电平信号：

- `tlast = 1`：当前处于帧内。
- `tlast` 上升沿：帧开始。
- `tlast` 下降沿：帧结束。
- `tvalid`：当前周期的 `tdata` 是否为有效字节。

RX 物理接口不能反压，接收侧会忽略 `tready`。TX 支持 `tready` 反压；上游只有在 `tvalid && tready` 时才能推进数据。

## TX CDC 帧边界修复

`dev` 已包含 TX CDC 粘帧修复：

- `tx_cdc_fifo_axis.sv` 在异步 FIFO 读延迟产生的字节空拍期间继续保持帧级 `tlast`，只有读到显式帧尾标记才结束当前帧。
- `phy_mii_axis.sv` 和 `phy_rmii_axis.sv` 使用 `tx_end_pending` 锁存当前字节发送期间出现的帧结束信号。
- 当前字节尚未完全串行发送时禁止预取下一帧，避免下一帧首字节被并入上一帧。
- `tkeep` 和 `tstrb` 固定为有效，补全 TX AXIS 输出属性。

修复后的 CDC/PHY 仿真结果：

| 测试 | 结果 |
| --- | --- |
| MII 相位扫描 | 0～39 ns，共 40 组，每组连续 256 帧，全部通过 |
| RMII 相位扫描 | 0～19 ns，共 20 组，每组连续 256 帧，全部通过 |
| MII 轻微异频 | 半周期 19900/19999/20000/20001/20100 ps，全部通过 |
| RMII 轻微异频 | 半周期 9950/9999/10000/10001/10050 ps，全部通过 |
| 连续流压力测试 | MII 和 RMII 各连续 2048 帧，输入/CDC/PHY 帧数一致 |

以上测试均未发现数据错误、帧粘连或 TX abort。

## FIFO 满与持续流量

当上游产生数据的平均速率长期高于 PHY 实际发送速率时，TX FIFO 仍可能逐渐堆积并最终拉低 `tready`，这是正常的反压行为。只要上游遵守 `tready`，TX FIFO 不会覆盖尚未发送的数据。

PHY RX 无法暂停。如果持续接收速度长期超过回环发送和缓存释放速度，`udp_ring` 可能耗尽空间；此时当前未提交的数据包会被整包丢弃，不会发送半包。2048 帧回归验证了当前 CDC/帧边界修复在连续流量下不会造成额外粘帧或丢帧，但有限深度 FIFO 不能吸收无限时间的速率差。

## 主要文件

```text
top_mii.sv                     当前 MII 顶层
top.sv                         备用 RMII 顶层
udp_ring.sv                    UDP 完整包事务式回环缓存
ep4ce10.qsf                    Quartus 器件、顶层、引脚及源文件配置
ep4ce10.out.sdc                50 MHz 系统时钟和 MII 时序约束
eth_axis/udp.sv                UDP/ARP/ICMP 与 CDC 集成
eth_axis/udp_axis_rx.sv        UDP RX 解析、校验及分片拒收
eth_axis/udp_axis_tx.sv        UDP TX 封装及 IPv4 分片
eth_axis/eth_axis.sv           ARP 和 ICMP 处理
eth_axis/rx_cdc_fifo_axis.sv   RX 异步 CDC FIFO
eth_axis/tx_cdc_fifo_axis.sv   TX 异步 CDC FIFO 与通道仲裁
eth_axis/phy_mii_axis.sv       MII 与 AXIS 转换
eth_axis/phy_rmii_axis.sv      备用 RMII 与 AXIS 转换
eth_axis/user_dc_fifo.sv       Intel dcfifo 封装
eth_axis/axis.svh              AXIS 接口定义
eth_axis/pc_head.svh           UDP 对端地址和端口元数据接口
```

`ETH_old/` 是旧版参考代码，不参与当前 Quartus 工程编译。

## 编译与板上验证

1. 使用 Quartus 打开 `ep4ce10.qpf`。
2. 确认顶层实体为 `top_mii`，执行完整编译并下载生成的配置文件。
3. 将 PC 设置到 `10.10.1.0/24` 网段，例如 `10.10.1.2/24`。
4. 执行 `ping 10.10.1.10`，验证链路、ARP 和 ICMP。
5. 向 `10.10.1.10` 发送 UDP 数据，检查返回包的内容、长度和端口。

如需改用 RMII，需要同时切换顶层、PHY 引脚分配和时序约束，不能只替换 PHY 转换模块。

## 已知限制

- 仅支持 IPv4。
- RX 不进行 IPv4 分片重组，并主动丢弃所有分片包。
- RX 物理数据流不能反压，持续超负载时只能通过有限缓存和整包丢弃处理。
- SMI 控制器当前固定访问 PHY 地址 1，并仅通告 100 Mbps 全双工。
