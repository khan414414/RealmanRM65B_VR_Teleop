# RM65-B VR 遥操作技术方案

> 文档性质：当前实现说明与后续设计基线  
> 代码基线：`c255c154354e9a11ba82dd5286bf63e8a148d922`（2026-08-29）  
> 整理日期：2026-10-26  
> 适用入口：`python -m teleop.vr_teleop`

## 1. 文档范围与结论

本方案使用右手 VR/XR 控制器对 RealMan RM65-B 六轴机械臂进行相对式遥操作：手柄三维平移映射为机器人基坐标系下的 TCP 位置增量，由 J1～J5 完成位置逆解；手柄绕 Aim 轴的局部 twist 单独映射到 J6。系统不跟踪完整六维工具姿态，也不进行 VR 空间与机器人空间的绝对位姿注册。

当前实现的核心特征如下：

- Unity/OpenXR 采集右手控制器，每个 Unity `Update` 发送一帧 TCP 文本数据。
- Python 使用“最新值槽”而不是逐帧 FIFO，以固定 `100 Hz` 重采样并下发关节目标。
- Trigger 是软件离合器：按下建立相对锚点并跟随，松开停止生成新命令。
- XYZ 方向由 `3×3` 正交矩阵标定；标定本身不缩放距离，但控制层额外应用硬编码 `1.5×` 平移增益。
- J1～J5 使用局部、连续、位置任务逆解；J6 由手柄 twist 独立控制。
- 实际下发接口为 RealMan API2 `rm_movej_canfd()`，封装入口为 `servo_joint()`。
- 软件会做工作区投影、关节/笛卡尔限速、关节余量、奇异性、自碰撞、跟随误差和目标新鲜度检查，但不替代控制器安全功能和硬件急停。

## 2. 系统架构

```text
Meta Quest / OpenXR
        │ 右手位置、姿态、Trigger
        ▼
Unity Main 场景
VRControllerSender / SendVRControllerData.cs
        │ TCP 127.0.0.1:5555
        │ 11 字段换行文本，XR 原生帧率
        ▼
teleop/vr_teleop.py
        │ 协议解析、最新值缓存、100 Hz 调度、标定状态机
        ▼
teleop/vr_tracker.py
        │ 相对锚定、方向重映射、工作区/速度/安全检查
        ▼
teleop/rm65_decoupled_ik.py
        │ XYZ → J1..J5，局部 twist → J6
        ▼
Realman_Control/realman_control.py
        │ RealMan Python API2 / rm_movej_canfd()
        ▼
RM65-B 控制器 192.168.1.18:8080
```

### 2.1 模块职责

| 模块 | 职责 | 是否在主控制链路 |
|---|---|---:|
| `VR-Teleop/Assets/Scripts/SendVRControllerData.cs` | 读取 Unity 中的右手控制器 Transform 和 Trigger，转换坐标并发送 TCP 数据 | 是 |
| `teleop/vr_teleop.py` | 连接 Unity 和机器人、固定频率调度、标定交互、异常与新鲜度处理 | 是 |
| `teleop/vr_tracker.py` | 锚定、位置/腕转目标生成、限速、安全验证和关节命令发送 | 是 |
| `teleop/rm65_decoupled_ik.py` | RM65-B 正运动学、位置雅可比、J1～J5 数值逆解、连续 J6 腕转 | 是 |
| `Realman_Control/realman_control.py` | API2 单位转换、控制器限位读取、算法层校验、运动接口封装 | 是 |
| `teleop/vr_calibration.py` | VR 方向标定、质量检查和 JSON 持久化 | 标定时 |
| `teleop/gripper_request.py` | HTTP 夹爪请求 | 已实例化但主链路未触发 |
| `Realman_Control/rm65_state_logger.py` | 独立 UDP 状态记录 | 否，辅助工具 |
| `Realman_Control/replay_rm65_trajectory.py` | 日志校验、重采样和实机回放 | 否，辅助工具 |
| `UR10_RTDE/` | 原始 UR10 参考实现 | 否 |

## 3. 坐标系、定位与重映射

### 3.1 坐标系定义

| 符号 | 坐标系 | 来源/用途 |
|---|---|---|
| `{U}` | Unity/XR 世界坐标系 | OpenXR 驱动更新右手控制器 GameObject |
| `{S}` | Unity 发送坐标系 | Unity 发送前完成轴交换和手性处理 |
| `{B}` | RM65-B 基坐标系 | 位置目标、正运动学和位置雅可比的参考系 |
| `{6}` | 机器人第 6 连杆坐标系 | 内置 RM65-B 运动学链末端 |
| `{T}` | 当前活动 TCP/工具坐标系 | 从示教器活动工具框读取的 `{6}T_T` |

当前所谓“定位”是基于 Trigger 锚点的相对定位，不是绝对定位：

- 不求 `{B}T_U` 的完整六维外参；
- 不标定 VR 原点到机器人基座原点的平移；
- 每次 Trigger 从松开变为按下时，手柄当前位置和机器人当前 TCP 位置分别成为新的零点；
- XR Origin 重定位、Guardian 重置、基站移动或 Unity 坐标配置变化后，方向标定需要重做。

### 3.2 Unity 到发送坐标系的转换

Unity 发送器对位置执行：

```text
p_S = [ z_U, -x_U, y_U ]ᵀ
```

对应代码为：

```csharp
position = new Vector3(position.z, -position.x, position.y);
```

四元数按 `(x, y, z, w)` 排列并转换为：

```text
q_S = [ qz_U, -qx_U, qy_U, -qw_U ]
```

该转换处理 Unity/OpenXR 与 Python 侧约定之间的轴基和手性差异。Python 会再次归一化四元数；近零范数四元数会被拒绝。

场景中 `controller` 引用 `Right Controller`。它的 Action Based Controller 位置动作优先绑定右手 `devicePosition`，旋转动作优先绑定右手 `pointerRotation`，因此：

- 平移主要来自握持中心，降低纯腕转造成的伪平移；
- 腕转主要来自 Aim 朝向，适合提取“拧螺丝刀”式 twist。

### 3.3 VR 到机器人基坐标的方向标定

令标定后的方向映射为 `R_SB ∈ SO(3)`。它满足：

```text
Δp_B_direction = R_SB · Δp_S
R_SBᵀ R_SB = I, det(R_SB) = +1
```

程序采集同一原点 `c` 以及操作者认为对应机器人基坐标 `+X/+Y/+Z` 的三个手柄点 `p_x/p_y/p_z`，构造单位方向矩阵：

```text
A = [u_x u_y u_z]
u_i = (p_i - c) / ||p_i - c||
```

然后通过正交 Procrustes/SVD 求解：

```text
R_SB = argmin ||R · A - I||F,  R ∈ SO(3)
```

当前标定质量门限为：

| 项目 | 门限 |
|---|---:|
| 每个静止点采样数 | `40` 帧 |
| 静止采样 RMS 抖动 | `≤ 3 mm` |
| 静止采样最大偏差 | `≤ 8 mm` |
| 每根轴相对原点位移 | `8～25 cm` |
| 任意两轴夹角 | `75～105 deg` |
| 右手性三重积 | `≥ 0.75` |
| 方向矩阵条件数 | `≤ 1.5` |
| 单轴最大拟合误差 | `≤ 12 deg` |
| 三轴 RMS 拟合误差 | `≤ 8 deg` |

标定结果原子写入：

```text
config/vr_position_calibration.json
```

文件 schema 为 `1`，输入帧标识为 `unity_sender_z_minus_x_y_v1`，输出帧为 `rm65_base`。加载时严格检查 schema、坐标系、单位、`distance_scale=1.0`、矩阵正交性、右手性和质量字段。

当前仓库没有提交该标定文件，因此首次运行会使用回退矩阵：

```text
R_SB_fallback = diag(-1, -1, 1)
```

与 Unity 发送转换合并后，回退模式的位置方向关系为：

```text
Δp_B ∝ [ -Δz_U, +Δx_U, +Δy_U ]ᵀ
```

### 3.4 Trigger 相对锚定与位置目标

Trigger 按下的第一个有效控制周期会读取：

- 手柄锚点位置与姿态：`p_S0, q_S0`；
- 机器人六关节反馈：`q_R0`；
- 当前活动工具框 `{6}T_T`；
- 由 RM65-B 模型正解得到的 TCP 锚点位置：`p_B0`。

保持 Trigger 按住时，当前位置请求为：

```text
Δp_raw = 1.5 · R_SB · (p_S - p_S0)
Δp_cmd = sphere_clip(Δp_raw, radius=0.25 m)
p_B_requested = p_B0 + Δp_cmd
```

这意味着：

- 当前端到端平移增益是 `1.5×`；
- 手柄约移动 `16.7 cm` 即达到 `0.25 m` 的相对工作区半径；
- 超出工作区时不是丢弃目标，而是沿原方向投影到球面；
- 松开再按下 Trigger 会重新建立 `p_S0/p_B0`，类似离合器或增量控制“归零”。

### 3.5 腕转重映射

手柄姿态不进入完整姿态逆解。程序只提取相对锚点姿态中绕发送坐标系局部 `+X` 轴的 swing-twist 分量：

```text
q_rel = inverse(q_S0) ⊗ q_S
θ_twist = unwrap(project_twist(q_rel, axis=[1,0,0]))
q6_requested = q6_anchor + wrist_roll_sign · wrist_roll_gain · θ_twist
```

当前参数：

| 参数 | 值 |
|---|---:|
| `wrist_roll_axis` | `(1, 0, 0)` |
| `wrist_roll_sign` | `+1` |
| `wrist_roll_gain` | `1.0` |
| 单次输入 twist 吸收上限 | `45 deg` |
| J6 关节安全余量 | 上下限各 `15 deg` |

四元数符号翻转 `q ↔ -q` 会被连续化处理；跨越 `±180°` 的 twist 会展开为连续角。单帧大于 `45°` 的变化会在后续周期继续吸收，而不是永久拒绝。接近“绕正交轴旋转 180°”时 twist 数学上不确定，该输入会被拒绝并保持上一个安全目标。

手柄其余 pitch/yaw 不直接控制机器人姿态。由于 J1～J5 只完成三维位置任务，TCP 姿态会随逆解构型变化。若活动 TCP 相对 J6 轴存在横向偏置，J6 转动也会改变 TCP 位置，J1～J5 会对此进行补偿。

## 4. 交互接口

### 4.1 Unity → Python TCP 协议

| 属性 | 值 |
|---|---|
| 角色 | Unity 为 TCP 服务端，Python 为客户端 |
| 地址 | `127.0.0.1:5555` |
| 编码 | ASCII/UTF-8 兼容文本，十进制使用 invariant culture |
| 分帧 | 每条记录以 `\n` 结束；不能依赖 TCP 包边界 |
| 延迟设置 | 两端均启用 `TCP_NODELAY`/`NoDelay` |
| 发布频率 | 每个 Unity `Update` 一帧，取决于 XR Runtime 帧率 |

当前 11 字段协议为：

```text
px,py,pz,qx,qy,qz,qw,trigger,reserved_grip,sequence_id,unity_time_s\n
```

| 序号 | 字段 | 单位/范围 | 说明 |
|---:|---|---|---|
| 1～3 | `px,py,pz` | m | `{S}` 中右手控制器位置 |
| 4～7 | `qx,qy,qz,qw` | 无量纲 | `{S}` 中四元数 |
| 8 | `trigger` | `0/1` | 跟随离合器状态 |
| 9 | `reserved_grip` | 固定 `0` | 保留兼容，Python 忽略 |
| 10 | `sequence_id` | 非负整数 | Unity 帧序号，用于丢帧统计 |
| 11 | `unity_time_s` | s | Unity 单调时间，用于输入频率统计 |

Python 同时兼容旧的 9 字段协议。Unity 时间戳目前不参与新鲜度判定；控制器样本年龄使用 Python 收包时的 `time.monotonic()` 计算。格式错误、非有限值或非法时间字段会被记录并忽略。

### 4.2 人机交互

| 输入 | 行为 |
|---|---|
| 右手 Trigger 按住 | 建立锚点后持续跟随 |
| 右手 Trigger 松开 | 清除待跟随目标，停止生成新关节命令 |
| 右手 Grip | 已禁用；协议字段保留为 `0` |
| Python 终端 `Q` | 请求进入方向标定；必要时等待 Trigger 释放 |
| 标定中 `Enter` | 采集当前标定点 |
| 标定结果页 `Y` | 保存并立即应用映射 |
| 标定中 `R` | 清除本轮样本并重做 |
| 标定中 `Esc` | 取消，保留原映射 |
| `Ctrl+C` | 退出主程序并断开 API2 连接 |

Trigger 松开、进入标定和主程序正常退出都不会调用 `rm_set_arm_slow_stop()`。它们不是安全急停。CAN-FD 命令流停止后的保持、减速或停机策略取决于 RM65 控制器和示教器配置。

### 4.3 Python → RM65-B 控制接口

`RealmanControl` 对上层暴露统一单位：

- 关节：弧度；
- 位置：米；
- 姿态：旋转向量，弧度；
- API2 边界内部转换为关节角度制和固定轴 XYZ 欧拉角。

主遥操作链路使用：

```text
VRTracker.track()
  → RealmanControl.servo_joint(joints_rad, time=0.01)
  → rm_movej_canfd(joints_deg, high_follow=False,
                   expand=0.0, trajectory_mode=1, radio=50)
```

`speed`、`acceleration`、`lookahead_time` 和 `gain` 参数为兼容旧接口而保留，API2 的该透传调用不会消费这些参数。当前使用低跟随模式；`high_follow=False`。

其他封装接口包括：

| 封装方法 | API2/行为 | 主遥操作是否使用 |
|---|---|---:|
| `get_joint_values()` | `rm_get_joint_degree()`，角度转弧度 | 是 |
| `get_joint_limits()` | 读取示教器最小/最大关节位置并缓存 | 启动时 |
| `get_active_tool_frame_pose_euler()` | 读取活动工具框 | 启动时 |
| `validate_joint_target()` | 厂商奇异性与自碰撞算法，只读 | 每个候选目标 |
| `servo_joint()` | `rm_movej_canfd()` | 是 |
| `servo_tool()` | `rm_movep_canfd()` | 否 |
| `move_joint()/move_tool()` | 规划运动 | 否 |
| `stop()` | `rm_set_arm_slow_stop()` | 主遥操作不调用 |
| `close()` | 删除 API2 机械臂连接 | 退出时 |

### 4.4 夹爪接口

旧夹爪接口为：

```text
POST http://130.215.216.42:8005/open
POST http://130.215.216.42:8005/close
```

当前仍创建 `GripperRequest` 对象，但 Unity 将 `reserved_grip` 固定为 `0`，Python 主循环不调用 `trigger_gripper()`，所以主遥操作中不会产生夹爪 HTTP 请求。

### 4.5 独立状态记录接口

`python -m Realman_Control.rm65_state_logger` 会暂时把控制器实时上报配置指向本机：

| 项目 | 默认值 |
|---|---:|
| RM65 API2 | `192.168.1.18:8080` |
| UDP 接收端口 | `8089` |
| 上报周期 | `20 ms`（`50 Hz`） |
| 输出目录 | `C:\RM65Logs` |
| 内存队列 | `10000` 条 |
| 首帧超时 | `5 s` |

记录字段包括接收单调时间、系统时间、估算设备时间、SDK 错误码、机械臂状态、六关节角、基坐标 TCP 位置与四元数。当前 API2 回调未提供原生控制器时间戳，因此 `device_time_ns` 是按上报周期估算的，不能用于高精度端到端时延测量。

记录器会保存并在退出时尝试恢复原实时上报配置；它不发送运动命令，也不会停止另一个控制进程。

## 5. 控制周期与状态机

### 5.1 频率同步

- Unity 以 XR/渲染原生帧率发布，常见为 `72/80/90/120 Hz`。
- Python 只保留最新完整样本，不积压旧动作。
- 机器人命令调度器使用绝对截止时间，以固定 `100 Hz` 运行。
- 调度器落后时跳过已错过周期，不突发补发。
- 最新样本年龄超过 `50 ms` 时，不推进 VR 目标，只重复最后安全关节命令。
- 每 `3 s` 输出一次 Unity 输入频率、机器人命令频率、样本年龄、Unity 序号丢帧数和机器人跳周期数。

### 5.2 状态转换

```text
启动
  ▼
[暂停 Paused] ── Trigger 按下且锚点校验通过 ──► [跟随 Tracking]
      ▲                                           │
      └────────────── Trigger 松开 ───────────────┘
      │
      ├── Q ──► [等待 Trigger 释放] ──► [标定 Calibration]
      │                                      │
      └──────────── 保存/取消并再次释放 ◄─────┘

[跟随] -- 样本陈旧/安全检查失败/跟随误差过大 --> 重发最后安全命令
[任意] -- TCP 断开或 Ctrl+C --> 断开 API2；不主动慢停
```

进入跟随前，`resume()` 会验证当前反馈：

1. 六关节反馈形状和数值有效；
2. 当前关节位于扣除限位余量后的安全区间内；
3. J1～J5 位置雅可比不过度病态；
4. 厂商奇异性和自碰撞检查通过；
5. 正运动学和活动 TCP 可建立一致锚点。

## 6. 逆解与命令生成

### 6.1 J1～J5 位置逆解

`RM65DecoupledIK` 使用嵌入代码的 RM65-B 运动学参数构造正运动学和雅可比。当前仓库未包含其注释所引用的原始 `RM65-B.urdf`，因此代码中的关节原点、轴和限位常量是当前可审查的模型基线。

每个控制周期以“上一次已接受命令”为 seed，只用 J1～J5 求解三维位置：

```text
e = p_target - FK(q)[0:3]
Δq_task = Jᵀ (J Jᵀ + λ² I)⁻¹ e
```

其中：

- `J` 是 `3×5` 的 J1～J5 位置雅可比；
- 阻尼 `λ` 随最小奇异值降低而增大；
- 零空间目标优先保持 Trigger 按下时的 J1～J5 构型；
- 叠加接近关节限位时的回避方向；
- 每次迭代限制最大关节步，并使用回溯线搜索保证位置误差下降；
- 解被限制在 seed 附近的局部 trust region，避免跳到另一逆解支路；
- J6 不进入位置伪逆，但其值参与完整正运动学。

运行时位置容差为 `0.2 mm`。逆解未完全收敛并不必然丢帧：只要候选解确实同时接近本周期子目标和保留的最终目标，并通过全部限速与安全检查，就可作为有界的局部推进量下发。

### 6.2 单周期命令流水线

每个新鲜控制周期按以下顺序处理：

1. 校验手柄位置和四元数，更新“最新意图”；
2. 应用 `R_SB`、`1.5×` 平移增益和 `0.25 m` 工作区投影；
3. 提取连续腕转，计算并裁剪 J6 请求；
4. 检查反馈与最近命令包络的 following error；
5. 以最大笛卡尔步长生成本周期位置子目标；
6. 用 J1～J5 位置逆解并独立推进 J6；
7. 将候选裁剪到带安全余量的关节区间；
8. 对六关节增量统一缩放，保持关节运动方向；
9. 用实际 FK 位移限制笛卡尔步长；
10. 检查候选确实向最终目标推进；
11. 检查关节余量、本地位置雅可比、厂商奇异性和自碰撞；
12. 若完整步只因安全阈值失败，尝试正常关节步长的 `2.5%`；
13. 通过后调用 `servo_joint()`；否则保留目标并重发最后安全命令。

### 6.3 当前运行参数

| 参数 | 当前值 | 单周期等效值（100 Hz） |
|---|---:|---:|
| 控制周期 | `0.01 s` | — |
| 平滑系数 | `1.0` | 不做指数平滑 |
| 平移增益 | `1.5×` | — |
| 最大相对工作区半径 | `0.25 m` | — |
| 最大笛卡尔目标速度 | `0.6 m/s` | `6 mm/cycle` |
| 最大关节目标速度 | `180 deg/s` | `1.8 deg/cycle` |
| J1～J5 限位余量 | `5 deg` | — |
| J6 限位余量 | `15 deg` | — |
| J1～J5 最小位置雅可比奇异值 | `0.01` | — |
| following error 阈值 | `10 deg` | 阻塞恢复阈值 `8 deg` |
| following error 检查周期 | `0.1 s` | 每 10 个控制周期 |
| 低跟随命令包络 | `0～150 ms` | 最近约 15 个周期 |
| 安全边界尝试比例 | `2.5%` | 相对正常关节步长 |
| 新鲜样本最大年龄 | `50 ms` | 约 5 个控制周期 |

关节硬限位优先从控制器/示教器读取。只有上层对象不提供 `get_joint_limits()` 时，才回退到代码内的模型限位。带余量的区间才是 VR 命令允许进入的区间。

## 7. 安全语义与故障处理

### 7.1 软件能够处理的情况

| 情况 | 行为 |
|---|---|
| 目标超出相对工作区 | 投影到 `0.25 m` 球面并逐周期逼近 |
| 输入帧陈旧超过 `50 ms` | 不推进目标，重发最后安全命令 |
| 调度器错过周期 | 跳过旧周期，不突发补发 |
| 逆解单周期能力不足 | 保留目标，按限速逐周期逼近 |
| 接近关节限位 | 使用余量区间裁剪并检查 |
| 完整步接近奇异/自碰撞 | 尝试经验证的 `2.5%` 小步 |
| 小步也不安全 | 保留最新意图，重发上一安全关节目标 |
| following error 过大 | 冻结命令推进，保留最新意图，等待反馈追上 |
| malformed TCP 记录 | 忽略该记录，继续处理后续完整记录 |
| Unity TCP 正常断开 | 退出循环并关闭 API2 连接 |

### 7.2 不能误解的安全语义

- Trigger 是软件离合器，不是急停。
- “重复最后目标”是 CAN-FD 连续流保持策略，不等于安全停机。
- 主程序在松开 Trigger、标定、TCP 断开、`Ctrl+C` 时均不主动慢停。
- 软件奇异性/自碰撞检查只验证候选关节端点，不替代物理围栏、速度限制、工具/负载配置和控制器安全功能。
- 控制器通信超时后的真实行为必须在示教器和 RM65 控制器上确认。
- 首次联调应使用低速、安全姿态、小幅手柄动作，并确保硬件急停可立即触达。

## 8. 部署与操作流程

### 8.1 依赖

| 类别 | 当前要求 |
|---|---|
| 操作系统 | Windows 10/11 |
| Python | Conda 环境，项目说明使用 Python 3.12 |
| Python 包 | `numpy scipy requests Robotic_Arm` |
| Unity | `6000.5.6f1` |
| XR 包 | OpenXR `1.17.1`、XR Management `4.5.4`、XR Interaction Toolkit `3.5.1` |
| VR | 支持 OpenXR 的头显/控制器，例如 Meta Quest Link |
| 机器人网络 | PC 与 `192.168.1.18:8080` 可达 |

### 8.2 启动顺序

1. 清空工作区，确认工具、负载、TCP、关节限位和控制器安全配置正确。
2. 给 RM65-B 上电、清警并使能；将机械臂置于非奇异、安全起始姿态。
3. 在 Unity 打开 `VR-Teleop/Assets/Scenes/Main.unity`，进入 Play。
4. 确认 Unity Console 显示 `Server started at 127.0.0.1:5555` 和 `Waiting for connection...`。
5. 在仓库根目录启动：

   ```powershell
   conda activate VRControl
   $env:PYTHONNOUSERSITE="1"
   python -m teleop.vr_teleop
   ```

6. 检查程序打印的关节限位来源；首次或 XR 环境变化后执行方向标定。
7. 将右手控制器放在舒适位置，小幅按住 Trigger 验证 XYZ 和 J6 正反方向。

### 8.3 标定流程

1. 松开 Trigger，在 Python 终端按 `Q`。
2. 原点保持静止，按 `Enter`，等待 40 帧采样完成。
3. 从同一原点沿机器人基坐标 `+X` 移动约 `10～15 cm`，按 `Enter`。
4. 回到同一原点，再分别完成 `+Y` 和 `+Z` 采样。
5. 检查输出的三维点、轴长度、夹角、拟合误差和矩阵。
6. 按 `Y` 保存；若质量或方向可疑，按 `R` 重做；按 `Esc` 取消。
7. 标定结束后先完全释放 Trigger，再重新按住建立新锚点。

### 8.4 退出顺序

1. 松开 Trigger。
2. 在示教器上停止/确认机械臂状态。
3. 在 Python 终端按 `Ctrl+C`。
4. 停止 Unity Play。

## 9. 验证现状

当前自动化测试覆盖：

- TCP 9/11 字段解析和非法记录拒绝；
- 100 Hz 调度跳周期而不突发补发；
- Unity 序号丢帧和频率统计；
- Trigger 锚定、暂停、keepalive 和传输异常传播；
- 方向标定求解、右手性、距离、抖动、质量门限和 JSON 往返；
- RM65-B 正运动学、解析雅可比与数值差分一致性；
- J1～J5 位置逆解、J6 解耦、连续展开和大跳变分段吸收；
- 控制器关节限位读取、单位转换和命令校验。

在 `VRControl` 环境执行：

```powershell
C:\Users\khan\.conda\envs\VRControl\python.exe -m unittest discover -s tests -v
```

本次基线实测为 **57 项中 48 项通过、9 项失败**。失败均位于 `tests/test_vr_tracker.py`，主要原因是测试仍按 `1.0×` 平移比例和 `90 deg/s` 关节步长构造期望，而当前实现为 `1.5×` 和 `180 deg/s`；若干“保留目标/实际笛卡尔步长”用例也因同一个 `1.5×` 因子连带失败。因此当前测试集不是全绿状态，必须先确认期望设计参数，再选择修改实现或更新测试，不能直接把失败断言批量改成当前数值。

尚未由仓库内测试覆盖的部分：

- Unity PlayMode 到 Python 的真实网络联调；
- Quest/OpenXR 丢跟踪与重定位场景；
- RealMan API2 实机/硬件在环测试；
- 控制器 CAN-FD 超时和断流后的真实安全行为；
- 工具偏置、负载和场景障碍物下的碰撞验证；
- 端到端运动时延和抖动统计。

## 10. 已知缺口与改进建议

### P0：联调前应确认

1. **先确认参数设计契约。** 需要由项目负责人明确平移增益应为 `1.0×` 还是 `1.5×`、关节限速应为 `90` 还是 `180 deg/s`、笛卡尔限速应为 `0.15` 还是 `0.6 m/s`，随后让代码、测试、README 和本文保持一致。当前值只是运行代码事实，不应视作已经安全验收。
2. **参数来源不集中。** 机器人 IP、Unity 端口、夹爪地址、平移增益和安全阈值散落并部分硬编码。建议引入版本化 YAML/JSON 配置，并在启动时完整打印有效配置。
3. **平移增益语义曾漂移。** 标定 schema 声明 `distance_scale=1.0`，控制层另乘 `1.5`。建议把 `position_gain` 设为显式配置项，并在标定/启动日志中同时显示“方向标定尺度”和“控制增益”。
4. **Trigger 不具备安全停机语义。** 必须在控制器侧验证 CAN-FD 看门狗、断流保持/减速/停机行为；不得用 Trigger 代替硬件急停。
5. **未显式传输 tracking-valid。** Unity 发送器只读 Transform 和 Trigger，没有把 OpenXR `isTracked/trackingState` 发给 Python。建议协议增加跟踪有效位；丢跟踪时冻结并要求 Trigger 释放后重新锚定。

### P1：建议优先完善

1. **时间同步不足。** Unity 时间只用于频率统计，状态日志设备时间又是估算值。建议记录 Unity 采样时间、Python 收包/下发时间、命令序号和机器人反馈序号，建立可分析的端到端时延链。
2. **缺少绝对外参。** 当前只标定旋转方向，适合相对离合控制，不适合需要 VR 场景与机器人工作空间对齐的任务。若要显示数字孪生或做绝对定位，应使用至少 3 个非共线对应点求 `{B}T_U`，并独立验证尺度。
3. **J6 符号依赖人工确认。** `wrist_roll_sign=+1` 未进入标定文件。建议将 J6 轴、符号和增益纳入设备/工具配置，并保留低速确认流程。
4. **模型来源不可追溯。** 运动学注释引用的 RM65-B URDF 未包含在当前仓库。建议提交许可允许的模型文件或记录版本、校验值和参数来源。
5. **缺少机器人状态门禁。** 启动和运行中应明确校验上电、使能、急停、报警、控制模式等状态，而不只依赖 API 连接和关节反馈。

### P2：工程化改进

1. 把遥操作、状态记录和回放的配置与日志格式统一起来。
2. 增加 Unity PlayMode、协议回放和 RealMan SDK mock 的端到端测试。
3. 为安全告警增加结构化事件码，而不仅是终端文本。
4. 在 VR 内显示连接、Trigger、样本年龄、限速、工作区边界和安全阻塞状态。
5. 对外部障碍物增加基于场景模型的碰撞检查；当前只检查机器人自碰撞。

## 11. 关键代码索引

| 主题 | 文件/位置 |
|---|---|
| Unity 坐标转换和 TCP 协议 | `VR-Teleop/Assets/Scripts/SendVRControllerData.cs` |
| 主循环、100 Hz 调度、样本新鲜度 | `teleop/vr_teleop.py` |
| 位置增益、锚定、限速、安全流水线 | `teleop/vr_tracker.py` |
| 标定求解与 JSON schema | `teleop/vr_calibration.py` |
| RM65-B 模型、FK/Jacobian、解耦 IK | `teleop/rm65_decoupled_ik.py` |
| RealMan API2 封装 | `Realman_Control/realman_control.py` |
| UDP 状态记录 | `Realman_Control/rm65_state_logger.py` |
| 轨迹回放 | `Realman_Control/replay_rm65_trajectory.py` |
| 单元测试 | `tests/` |

