# M20 SCAN-Planner `/NAV_CMD` 控制后端设计

日期：2026-07-23

## 目标

将 `/home/nvidia/scanplanner_test/SCAN-Planner` 的真机控制后端从
basic_server UDP 摇杆轴指令切换为 M20 官方 DDS 速度话题
`/NAV_CMD`，使 SCAN 的速度以 `m/s` 和 `rad/s` 直接下发，同时保留
定位、雷达、命令超时、单发布者和停车保护。

本设计不改变定位 TF、A*、Min-Snap、B 样条或闭环轨迹跟踪结构。

## 已确认的官方接口约束

依据《山猫 M20 开发指南》的“运动控制（ROS2）”章节：

- 话题：`/NAV_CMD`
- 类型：`drdds/msg/NavCmd`
- QoS：Reliable、Volatile
- `x_vel`、`y_vel` 单位为 `m/s`
- `yaw_vel` 单位为 `rad/s`
- 仅在导航模式下生效
- 必须处于 RL 控制状态 `17`
- 超过 `500 ms` 没有速度指令时，底盘自动减速停止
- 文档表格建议 `10 Hz`，开发建议要求保持 `20 Hz`；本设计采用更安全的
  `20 Hz`
- 发布 `/NAV_CMD` 时不得同时运行原厂 planner，也不得处于自主充电任务

最初设计采用文档中的内部平地步态编码。2026-07-24 真机验证发现，
当前 M20 云控仅提供基础、楼梯和高台三种步态，平地导航时
`/MOTION_INFO` 将基础步态报告为 `0x1001`。因此安全桥按真机接口修正为只
允许基础步态 `0x1001`。

文档中用于初始保守速度选择的有效范围为：

- X：`[-2.0, -0.15] ∪ [0.15, 2.0] m/s`
- Y：`[-1.0, -0.25] ∪ [0.25, 1.0] m/s`
- Yaw：`[-1.5, -0.35] ∪ [0.35, 1.5] rad/s`

官方文档：

`https://alidocs.dingtalk.com/i/p/OlnXRl7ed542DGLp/docs/mExel2BLV54bA9pbHvlrl10XWgk9rpMq`

## Git 隔离和回退

- worktree：`/home/nvidia/scanplanner_test/SCAN-Planner`
- 当前分支：`work/m20-scan-linear-udp-20260723`
- 修改前提交：`27dfb33`
- 新回退分支：`backup/m20-before-nav-cmd-20260723`
- 更早的 UDP 回退分支：
  `backup/m20-scan-before-linear-udp-20260723`

实现只发生在上述独立 worktree。旧 UDP 源码保留，但启动时只允许选择一个
后端。

## 控制架构

```text
closed_loop_controller
  -> /scan/cmd_vel_debug (geometry_msgs/Twist)
  -> scan_m20_safety_bridge
  -> /NAV_CMD (drdds/NavCmd)
  -> M20 DDS subscriber
```

`scan_m20_safety_bridge` 负责：

1. 校验输入与机器人状态。
2. 将 SCAN 的速度限制到 M20 有效范围。
3. 以 `20 Hz` 发布 `/NAV_CMD`。
4. 发生异常时立即开始零速度停车序列。
5. 发布 `/scan/navigation_enabled`，统一门控 SCAN 规划与闭环控制。

不再经过 UDP `X/Y/Yaw ∈ [-1,1]` 比例映射。

## 自包含 `drdds` 接口

当前 174 上存在实验接口包：

`/home/nvidia/lzh/m20_drdds_tests/src/drdds`

该包不是厂商安装包，但已成功解析真实 `/MOTION_INFO`。实现时将经过验证的
消息定义放入 SCAN 工作区的 `src/drdds`，避免运行时依赖 `/home/nvidia/lzh`
目录。

需要包含：

- `MetaType`
- `NavCmdValue`
- `NavCmd`
- `MotionStateValue`
- `GaitValue`
- `MotionInfoValue`
- `MotionInfo`

当前运行环境实际可用的头部为：

```text
uint64 frame_id
builtin_interfaces/Time stamp
```

新文档展示为自定义 `Timestamp timestamp`，但两者序列化字段均为
`int32 sec + uint32 nsec`。实现采用已经通过真实 `/MOTION_INFO` 解码验证的
ROS 2 定义，并在连接 `/NAV_CMD` 前再次用只读订阅验证 DDS 兼容性。

## 状态和步态边界

第一版不自动执行以下动作：

- 切换常规/导航使用模式
- 起立
- 进入 RL 控制
- 切换步态

操作者负责先将机器人置于导航模式、RL 状态 `17`、基础步态
`0x1001`。

安全桥从 `/MOTION_INFO` 实时检查：

- 消息新鲜度不超过 `0.50 s`
- `motion_state.state == 17`
- `gait_state.gait == 0x1001`
- 反馈速度和字段均为有限值

状态或步态改变时，安全桥立即停车并撤销
`/scan/navigation_enabled`。

由于 `/MOTION_INFO` 不包含 `ControlUsageMode`，第一版不能从 ROS 话题自动
验证导航使用模式。启用真机输出前，操作者必须确认已切换到导航模式。

## 速度处理

第一版采用保守的前进和转向值：

```text
X:
  vx <= zero_epsilon -> 0.0
  vx > zero_epsilon  -> 0.15 m/s

Y:
  始终 0.0 m/s

Yaw:
  abs(wz) <= 0.04 -> 0.0
  wz > 0.04       -> +0.35 rad/s
  wz < -0.04      -> -0.35 rad/s
```

含义：

- 禁止倒车和横移。
- 非零输出保留已经选定的保守测试值；基础步态下仍需通过真机低速测试确认。
- 这些是物理速度，不是摇杆比例。
- 转向换向必须经过至少一帧零速度。
- 不在 `/NAV_CMD` 数据包之间生成小于官方最小有效速度的斜率中间值。
  第一版只发送上述离散有效值，实际加减速由底盘内部速度控制完成。

第一版不提高 SCAN 规划速度，避免同时引入轨迹计时误差。真机稳定后再根据
`/MOTION_INFO` 的实际速度反馈单独标定。

## 自动允许输出

安全桥默认不创建真实 `/NAV_CMD` 发布者。真机后端启用后，只有以下条件连续
稳定 `2 s` 才自动 ARM：

- 定位、传感器位姿、同步点云和命令均新鲜
- 点云与传感器位姿时间戳配对
- `/MOTION_INFO` 新鲜
- 状态为 `17`
- 步态为 `0x1001`
- `/NAV_CMD` 当前没有其他发布者
- 当前命令为零
- 未触发手动 DISARM 锁存

ARM 成功后才创建 `/NAV_CMD` 发布者，并将
`/scan/navigation_enabled=true`。手动 DISARM 后不会自动重新 ARM，必须由
操作者再次明确允许。

若 `/NAV_CMD` 消息构造或发布发生异常，安全桥立即销毁发布者、回到 DISARMED
并锁存自动 ARM；必须由操作者排除故障后手动重新 ARM。

## 停车和故障处理

以下任一情况触发停车：

- SCAN 命令超过 `0.20 s` 未更新
- 定位、TF、同步点云或 `/MOTION_INFO` 超时
- 状态不再是 `17`
- 步态不再是 `0x1001`
- 出现第二个 `/NAV_CMD` 发布者
- 发布异常或非有限速度
- 操作者手动 DISARM

停车顺序：

1. 立即发布 `/scan/navigation_enabled=false`。
2. 清空预览和内部速度状态。
3. 以 `20 Hz` 连续发布至少 `20` 帧零速度，覆盖 M20 的 `500 ms` 超时窗口。
4. 销毁 `/NAV_CMD` 发布者。
5. 保持 DISARM，等待安全条件重新建立。

关闭节点时执行相同的零速度序列。

## 启动集成

`m20_scan_dry_run.launch.py` 保留互斥后端：

```text
control_backend = nav_cmd | udp
```

新增独立的真机开关：

```text
enable_nav_cmd_output = false | true
```

一键启动脚本使用：

```text
control_backend=nav_cmd
enable_nav_cmd_output=true
enable_udp_output=false
```

因此：

- 启动 `scan_m20_safety_bridge`
- 不启动 `scan_m20_udp_safety_bridge`
- 不打开 UDP 运动 socket
- RViz 脚本无需修改

## 测试和验收

### 离线测试

- `drdds` 消息包可编译并被 `m20_scan_bringup` 导入
- Dry-run 不创建 `/NAV_CMD` 发布者
- 状态非 `17` 时拒绝 ARM
- 步态非 `0x1001` 时拒绝 ARM
- 其他 `/NAV_CMD` 发布者存在时拒绝 ARM
- 输入超时、状态变化和步态变化均进入停车
- 倒车和横移输出为零
- 前进输出为 `0` 或 `0.15 m/s`
- 转向输出为 `0` 或 `±0.35 rad/s`
- 转向反向前出现零速度
- 启动文件只选择一个控制后端
- 一键启动选择 `nav_cmd`，不选择 UDP
- 原有定位、点云同步和安全测试继续通过

### DDS 只读验证

- source 新 worktree 后可执行
  `ros2 interface show drdds/msg/NavCmd`
- 可正确读取一帧 `/MOTION_INFO`
- `/NAV_CMD` 的机器人订阅者为 Reliable/Volatile
- 真机测试前 `/NAV_CMD` 发布者数量为零

### 真机验证

真机运动验证不包含在自动实施中。只有在操作者再次明确确认后，才按以下顺序
执行：

1. 架空或放在安全空旷区域，急停可用。
2. 手动进入导航模式、RL 状态 `17`、步态 `0x1001`。
3. 先只观察 preview 和安全状态。
4. 零速度 ARM。
5. 短时测试 `0.15 m/s` 前进。
6. 分别测试 `±0.35 rad/s` 转向。
7. 测试命令超时和手动 DISARM 停车。

## 验收标准

- 默认和测试过程不发送真实 `/NAV_CMD`。
- 真机启动时 UDP 控制后端未运行。
- `/NAV_CMD` 输出使用物理速度单位。
- 机器人状态或传感器异常后最多一个控制周期开始发送零速度。
- 20 帧零速度完成后发布者被销毁。
- 原 worktree 和回退分支保持可用。
