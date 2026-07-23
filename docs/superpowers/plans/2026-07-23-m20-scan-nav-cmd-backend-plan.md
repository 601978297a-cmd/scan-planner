# M20 SCAN `/NAV_CMD` 后端实施计划

日期：2026-07-23

## 成功条件

- SCAN 工作区自带可编译的 `drdds` 接口包。
- `/scan/cmd_vel_debug` 经安全桥输出为 `drdds/msg/NavCmd`。
- 前进只输出 `0` 或 `0.15 m/s`，横移始终为零。
- 转向只输出 `0` 或 `±0.35 rad/s`，换向前输出零。
- 只有 `/MOTION_INFO` 新鲜、状态 `17`、步态 `0x3002` 且其他安全条件满足时才允许 ARM。
- 一键启动选择 `nav_cmd`，不启动 UDP 后端。
- 默认、测试和自动验证不创建真实 `/NAV_CMD` 发布者。
- 回退分支 `backup/m20-before-nav-cmd-20260723` 保持指向 `27dfb33`。

## 任务 1：加入自包含 `drdds` 接口

新增：

- `src/drdds/CMakeLists.txt`
- `src/drdds/package.xml`
- `src/drdds/msg/MetaType.msg`
- `src/drdds/msg/NavCmdValue.msg`
- `src/drdds/msg/NavCmd.msg`
- `src/drdds/msg/MotionStateValue.msg`
- `src/drdds/msg/GaitValue.msg`
- `src/drdds/msg/MotionInfoValue.msg`
- `src/drdds/msg/MotionInfo.msg`

修改：

- `src/m20_scan_bringup/package.xml`

步骤：

1. 复制已经成功解码真实 `/MOTION_INFO` 的最小消息结构。
2. 将 `drdds` 声明为 `m20_scan_bringup` 的运行依赖。
3. 构建 `drdds`。
4. source 新 worktree 后运行 `ros2 interface show drdds/msg/NavCmd`。
5. 只读订阅一帧 `/MOTION_INFO`，验证 DDS 兼容；不创建命令发布者。

## 任务 2：测试先行锁定速度与机器人状态门控

修改：

- `src/m20_scan_bringup/test/test_safety_bridge_core.py`
- `src/m20_scan_bringup/test/test_safety_bridge_node.py`
- 新增或修改 `/NAV_CMD` 启动与配置测试

步骤：

1. 为 NAV 物理速度整形增加纯函数测试：
   - 负 X 和 Y 被拒绝。
   - 正 X 输出 `0.15`。
   - `|wz| <= 0.04` 输出零。
   - 非零转向输出 `±0.35`。
   - 非有限值输出零。
2. 增加状态测试：
   - 状态非 `17` 阻止 ARM。
   - 步态非 `0x3002` 阻止 ARM。
   - `/MOTION_INFO` 非有限值阻止 ARM。
3. 增加自动 ARM 稳定窗口和手动 DISARM 锁存测试。
4. 增加导航使能状态与停车零帧测试。
5. 增加单发布者冲突测试。
6. 先运行目标测试，确认旧实现不能满足新断言。

## 任务 3：完善 `scan_m20_safety_bridge`

修改：

- `src/m20_scan_bringup/m20_scan_bringup/safety_bridge_core.py`
- `src/m20_scan_bringup/m20_scan_bringup/scan_m20_safety_bridge.py`
- `src/m20_scan_bringup/config/m20_scan_safety_bridge.yaml`

步骤：

1. 在纯逻辑层加入 M20 NAV 有效速度整形。
2. 从 `/MOTION_INFO` 保存状态、步态和反馈速度有效性。
3. 使用最近完成的点云/传感器位姿时间戳对，避免新点云导致假超时。
4. 增加 `/scan/navigation_enabled` 的 Reliable/Transient Local 发布。
5. 增加 `2 s` 自动 ARM 稳定窗口和手动 DISARM 锁存。
6. ARM 前要求当前命令为零，且 `/NAV_CMD` 没有其他发布者。
7. ARM 后创建 `/NAV_CMD` 发布者，并以 `20 Hz` 发布物理速度。
8. 状态、步态、输入或发布者冲突异常时：
   - 立即撤销导航使能；
   - 清零内部命令；
   - 连续发布 `20` 帧零速度；
   - 销毁发布者。
9. 不添加自动起立、自动模式切换或自动步态切换。

## 任务 4：切换一键启动后端

修改：

- `src/m20_scan_bringup/launch/m20_scan_dry_run.launch.py`
- `src/m20_scan_bringup/launch/m20_localization_navigation.launch.py`
- 相关启动接线测试

步骤：

1. 新增 `enable_nav_cmd_output` launch 参数，默认 `false`。
2. 把该参数传给 `scan_m20_safety_bridge.enable_m20_output`。
3. 一键启动设置：
   - `control_backend=nav_cmd`
   - `enable_nav_cmd_output=true`
   - `enable_udp_output=false`
4. 测试确认同一次启动只出现一个控制后端。
5. RViz 脚本保持不变。

## 任务 5：离线验证

步骤：

1. 运行安全桥纯逻辑和节点目标测试。
2. 运行 `src/m20_scan_bringup/test` 全部测试。
3. 执行 `colcon build --packages-up-to m20_scan_bringup`。
4. 执行 `colcon test --packages-select drdds m20_scan_bringup`。
5. 执行 `colcon test-result --verbose`。
6. 检查所有配置默认值仍禁止真实输出。
7. 检查没有启动 launch、没有 `/NAV_CMD` 发布者、没有 UDP 控制进程。
8. 检查原 worktree 无修改。

## 任务 6：提交

步骤：

1. `git diff --check`。
2. 只提交计划中列出的文件。
3. 提交信息：
   `Use guarded M20 NAV_CMD backend for SCAN control`
4. 报告提交号、测试结果、启动前人工准备条件和回退方法。
