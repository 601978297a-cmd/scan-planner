# M20 碰撞模型与线性 UDP 映射设计

日期：2026-07-23

## 目标

在不改变 SCAN-Planner 的 B 样条规划和跟踪逻辑的前提下：

1. 按 M20 实际外形修正碰撞模型。
2. 保留现有 UDP 桥的心跳、自动 ARM、TF/雷达新鲜度检查和超时停车。
3. 将转向映射改成连续线性映射，消除小角速度被直接放大到较大摇杆量造成的左右扭动。
4. 将控制发送频率提高到 20 Hz。

本次设计不启动真机 UDP，不发送 ARM 或运动指令。

## Git 隔离与回退点

- 原工作目录：`/home/nvidia/scan_interface_audit/SCAN-Planner`
- 原分支：`work/m20-scan-nav-safety-bridge`
- 独立 worktree：`/home/nvidia/scanplanner版本1/SCAN-Planner`
- 修改分支：`work/m20-scan-linear-udp-20260723`
- 修改前提交：`82a7512`
- 备份分支：`backup/m20-scan-before-linear-udp-20260723`

所有修改只在独立 worktree 的修改分支中进行。原工作目录不会随之切换分支，但同一 Git 仓库能够看到新增的提交和分支。

## 修改范围

预计只修改以下文件及相应测试：

- `src/m20_scan_bringup/config/m20_scan_planner.yaml`
- `src/m20_scan_bringup/config/m20_scan_udp_bridge.yaml`
- `src/m20_scan_bringup/m20_scan_bringup/m20_udp_protocol.py`
- `src/m20_scan_bringup/test/` 下与 UDP 映射和配置有关的测试

不修改 A*、Min-Snap、B 样条生成、轨迹计时和定位 TF。

## 碰撞模型

M20 标称外形约为长 `0.82 m`、宽 `0.43 m`。继续使用现有双圆柱模型，参数调整为：

```yaml
grid_map:
  double_cylinder_radius: 0.30
  double_cylinder_offset: 0.20

optimization:
  dist0: 0.25
```

两个圆心沿机身前后方向分别偏移 `0.20 m`，半径为 `0.30 m`。对应的包络约为长 `1.00 m`、宽 `0.60 m`，给腿部摆动、定位误差和点云延迟留出安全余量。第一轮不同时修改 `dist0`，避免无法区分碰撞体尺寸和优化安全距离各自造成的影响。

## UDP 线性映射

### 前进

只允许前进，不启用倒车和横移：

```text
X = clamp(max(vx, 0) × 3.333333, 0, 0.50)
Y = 0
```

因此：

- `vx = 0.00 m/s` → `X = 0.00`
- `vx = 0.075 m/s` → `X = 0.25`
- `vx = 0.15 m/s` → `X = 0.50`

`X = 0.50` 不超过当前 SCAN UDP 桥已有的最大前进输出，同时高于此前 Nav2 真机测试约 `X = 0.30` 的上限。

### 转向

删除“只要出现非零转向就跳到约 0.6”的非线性最小转向逻辑，改为：

```text
if abs(wz) < 0.04:
    Yaw = 0
else:
    Yaw = clamp(wz × 3.0, -0.60, 0.60)
```

因此：

- `wz = 0.04 rad/s` → `Yaw = 0.12`
- `wz = 0.10 rad/s` → `Yaw = 0.30`
- `wz = 0.20 rad/s` → `Yaw = 0.60`

保留现有输出斜率限制：

- `max_ax: 0.10`
- `max_awz: 0.50`
- `udp_yaw_slew_rate: 2.0`

这些限制负责平滑变化，不再制造最小转向跳变。

### 发送频率

- `command_rate: 20.0 Hz`
- 预览/诊断发布频率同步为 `20.0 Hz`

速度限制和安全检查仍由现有 UDP 桥执行。不得绕开健康状态、数据超时、ARM 状态或停车逻辑直接发送控制包。

## 验证方法

先完成纯离线验证：

1. 单元测试覆盖零输入、前进比例、前进限幅、禁止倒车、强制横移为零、转向死区、正负转向比例和转向限幅。
2. 配置测试确认双圆柱参数、20 Hz、最大输出和线性比例均从 YAML 正确加载。
3. 运行 `colcon build` 和相关 `pytest`/`colcon test`。
4. 启动 UDP 桥的禁发或 dry-run 模式，仅检查预览话题，确认不会产生网络控制包。
5. 检查原 worktree 状态，确认它没有出现本次文件修改。

真机验证不包含在本次自动执行中。之后只有在操作者明确确认、机器人架空或处于安全空旷区域、急停可用时，才允许启用真实 UDP。

## 验收标准

- 修改前状态可由备份分支和提交 `82a7512` 找回。
- 原 worktree 文件内容不受影响。
- `vx = 0.15` 时预览 `X = 0.50`，且任何输入都不超过 `0.50`。
- 小转向不再直接跳到约 `0.6`；`wz = 0.10` 时预览约为 `0.30`。
- 横移始终为零，负前进速度不产生倒车。
- TF、雷达或控制命令超时后仍输出停车状态。
- 所有相关测试通过，验证期间不向真机发送 UDP 控制。

## 回退

查看修改前版本：

```bash
git -C /home/nvidia/scanplanner版本1/SCAN-Planner diff \
  backup/m20-scan-before-linear-udp-20260723
```

如需让新 worktree 回到修改前版本，应先保存任何仍需保留的未提交内容，然后在操作者确认后执行：

```bash
git -C /home/nvidia/scanplanner版本1/SCAN-Planner switch \
  --detach backup/m20-scan-before-linear-udp-20260723
```

原 worktree 始终可以继续使用原分支：

```bash
git -C /home/nvidia/scan_interface_audit/SCAN-Planner status
```
