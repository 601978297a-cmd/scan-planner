# M20 碰撞模型与线性 UDP 映射实施计划

日期：2026-07-23

## 成功条件

- 双圆柱碰撞参数为半径 `0.30 m`、前后偏移 `0.20 m`。
- `vx=0.075` 映射为 `X=0.25`，`vx=0.15` 映射为 `X=0.50`。
- `|wz|<0.04` 映射为零，`wz=0.10` 映射为 `Yaw=0.30`，`wz=0.20` 映射为 `Yaw=0.60`。
- 倒车和横移继续被禁止。
- 命令与预览频率均为 `20 Hz`。
- 现有心跳、自动 ARM、输入新鲜度和立即停车测试继续通过。
- 全部验证使用 `enable_udp_output: false`，不连接真实 UDP。

## 任务 1：用测试锁定新映射

文件：

- 修改 `src/m20_scan_bringup/test/test_m20_udp_protocol.py`
- 修改 `src/m20_scan_bringup/test/test_super_lio_wiring.py`

步骤：

1. 将测试中的映射上限改为 `max_vx=0.15`、`max_wz=0.20`、`max_x=0.50`、`max_yaw=0.60`、`yaw_zero_epsilon=0.04`。
2. 保留非有限值、禁止倒车和强制横移为零的测试。
3. 将前进测试改为 `0.075→0.25`、`0.15→0.50` 和超限饱和。
4. 用线性转向测试替换旧的最小转向/迟滞测试。
5. 增加 YAML 断言，锁定碰撞参数、20 Hz 和线性映射参数，并确认旧的转向死区/迟滞参数已删除。
6. 单独运行这两个测试文件，确认新断言在实现前失败。

## 任务 2：最小修改 UDP 映射

文件：

- 修改 `src/m20_scan_bringup/m20_scan_bringup/m20_udp_protocol.py`
- 修改 `src/m20_scan_bringup/m20_scan_bringup/scan_m20_udp_safety_bridge.py`

步骤：

1. 将 `UdpMappingLimits` 精简为前进和转向线性映射所需字段。
2. 删除 mapper 内部的转向符号状态、启动阈值、停止阈值和最小转向输出。
3. 按比例计算并限幅 `X` 和 `Yaw`；非有限值返回零，`Y` 始终为零。
4. 节点仅声明 `udp_max_x`、`udp_max_yaw` 和 `yaw_zero_epsilon` 所需参数。
5. 自动 ARM 的“命令必须为零”检查使用统一的 `yaw_zero_epsilon`。
6. 不修改 `slew_command`、`slew_udp_axis`、状态机、心跳或停车逻辑。
7. 重新运行 UDP 协议测试，确认通过。

## 任务 3：修改配置

文件：

- 修改 `src/m20_scan_bringup/config/m20_scan_planner.yaml`
- 修改 `src/m20_scan_bringup/config/m20_scan_udp_bridge.yaml`

步骤：

1. 设置 `double_cylinder_radius: 0.30`。
2. 设置 `double_cylinder_offset: 0.20`。
3. 保持 `optimization.dist0: 0.25`。
4. 设置 `command_rate: 20.0` 和 `preview_rate: 20.0`。
5. 保持 `max_vx: 0.15`、`max_wz: 0.20`、`udp_max_x: 0.50`。
6. 设置 `udp_max_yaw: 0.60`，删除旧的 `udp_yaw_deadzone`、`yaw_start_threshold` 和 `yaw_stop_threshold`。
7. 保持 `enable_udp_output: false`。
8. 运行配置测试，确认通过。

## 任务 4：离线回归验证

步骤：

1. 运行 `python3 -m pytest src/m20_scan_bringup/test/test_m20_udp_protocol.py`。
2. 运行 `python3 -m pytest src/m20_scan_bringup/test/test_super_lio_wiring.py`。
3. 运行 `python3 -m pytest src/m20_scan_bringup/test`。
4. 使用现有 ROS 2 环境执行 `colcon build --packages-select m20_scan_bringup`。
5. 执行 `colcon test --packages-select m20_scan_bringup` 并检查测试结果。
6. 检查 YAML 中 `enable_udp_output` 仍为 `false`，确认验证期间未启动节点、未发送 UDP。
7. 执行 `git diff --check`，查看最终差异。

## 任务 5：提交与回退确认

步骤：

1. 只提交本计划列出的文件。
2. 提交信息使用 `Smooth M20 UDP steering and update collision model`。
3. 确认原 worktree `work/m20-scan-nav-safety-bridge` 没有工作区改动。
4. 保留 `backup/m20-scan-before-linear-udp-20260723` 指向修改前提交 `82a7512`。
