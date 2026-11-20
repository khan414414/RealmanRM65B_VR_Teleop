# RM65-BI VR Teleoperation (firmware 1.4.x)

`RM65BI` 分支以 `main` 的 `c255c15` 为基础，适配无需升级固件的 RM65-BI 1.4.x 控制器。控制连接使用 **API 4.2.8** 的旧版版本查询协议；仓库包含经 SHA-256 校验的 Windows 64 位 DLL，来源见 [vendor/README.md](Realman_Control/vendor/README.md)。

现有 `RM65B` 类名仍作为兼容别名。`Robotic_Arm==1.1.6` 的离线 `Algo` 用于官方 IK/FK、奇异点和自碰撞预测；控制器连接、状态读取、规划运动和 CAN-FD 透传全部由 API 4.2.8 完成。

使用 VR/XR 手柄遥操作睿尔曼 RealMan RM65-B 六轴机械臂，并记录实机关节状态和末端位姿。

本项目改造自原始 `UR10_Teleop`：Unity 端继续负责读取手柄数据，Python 端通过 RealMan API 4.2.8 控制 RM65-BI。仓库中的 `UR10_RTDE` 目录仅作为原始接口参考，当前遥操作入口不会使用 UR10 RTDE 控制机械臂。

## 安全警告

本项目会向真实机械臂发送 CAN-FD 关节位置透传命令。首次运行前请务必做到：

- 清空机械臂工作空间，并确认没有人员位于运动范围内。
- 保持硬件急停按钮随时可用。
- 在机械臂安全配置和低速条件下测试。
- 首次只移动手柄 1～2 cm，确认坐标方向后再扩大动作。
- 不要把松开手柄 Trigger 后的“暂停跟随”当作安全急停。

当前遥操作层会检查相对工作范围、关节步长、关节余量、奇异位形、自碰撞和逆解状态。超出软阈值时保留最新目标并按速度上限逐周期逼近；大于 `45°` 的单帧手柄腕转也会分段吸收，不再永久拒绝。完整一步触发奇异或碰撞检查时，只尝试正常关节步长 `2.5%` 的已验证小步。若连小步也不安全，则保持最后一个安全目标，但不会调用机械臂慢停接口。奇异、自碰撞和示教器关节限位仍是不可穿越的硬边界，这些软件检查也不能替代机械臂自身的安全限制。

## 系统组成

数据流如下：

```text
右手 VR 控制器
    ↓ 位置、四元数、按键
Unity: SendVRControllerData.cs
    ↓ 每个 Meta/Unity XR Update 一帧，附序号和单调时间戳
    ↓ TCP 127.0.0.1:5555
Python: teleop/vr_teleop.py
    ↓ 最新值缓冲，固定 100 Hz 绝对时钟重采样
    ↓ 相对 XYZ、局部 wrist twist
VRTracker / OfficialPositionIK / 官方 Algo IK+FK
    ↓ J1～J5 位置逆解，J6 独立腕转
    ↓ servo_joint()
RM65BI / RealMan API 4.2.8
    ↓ Movej_CANFD()
RM65-B 控制器与实机
```

主要目录：

```text
VR-Teleop/                         Unity VR 输入与 TCP 服务端
teleop/                            遥操作数据解析、映射和夹爪请求
Realman_Control/realman_control.py RM65-BI 旧版 SDK 控制封装
Realman_Control/legacy_sdk.py     RealMan API 4.2.8 的类型化 C 接口绑定
Realman_Control/rm65_state_logger.py
                                   实机状态 CSV/NPZ 记录器
UR10_RTDE/                         原始 UR10 参考代码，当前入口不使用
```

## 默认配置

| 项目 | 默认值 | 位置 |
|---|---:|---|
| RM65-B 控制器 | `192.168.1.18:8080` | `teleop/vr_teleop.py` |
| Unity TCP 服务端 | `127.0.0.1:5555` | Unity 场景和 `teleop/vr_teleop.py` |
| Meta/Unity 发布频率 | XR Runtime 的实际帧率（常见 `72/80/90/120 Hz`） | `SendVRControllerData.cs` |
| 机械臂命令频率 | 固定 `100 Hz`，错过周期时跳帧而不突发补发 | `teleop/vr_teleop.py` |
| Python 控制周期 | `0.01 s` | `teleop/vr_teleop.py` |
| VR→基坐标映射 | 已保存的 `3x3` 标定矩阵；缺失时回退 `diag(-1,-1,1)` | `teleop/vr_teleop.py` |
| 位置距离比例 | `1.5:1`（标定矩阵只改变方向） | `teleop/vr_tracker.py` |
| 平移/腕转平滑系数 | `0.15` / `0.15` | `teleop/vr_teleop.py` |
| VR 标定文件 | `config/vr_position_calibration.json` | `teleop/vr_calibration.py` |
| 最大相对位置范围 | `0.25 m` | `teleop/vr_tracker.py` |
| 最大笛卡尔目标速度 | `0.15 m/s` | `teleop/vr_tracker.py` |
| 最大关节目标速度 | `90 deg/s` | `teleop/vr_tracker.py` |
| 奇异/碰撞阈值附近的尝试步长 | 正常关节步长的 `2.5%` | `teleop/vr_tracker.py` |
| 外部夹爪服务（Grip 控制已禁用） | `130.215.216.42:8005` | `teleop/vr_teleop.py` |
| 状态记录 UDP 端口 | `8089` | `rm65_state_logger.py` |
| 状态日志目录 | `C:\RM65Logs` | `rm65_state_logger.py` |

## 环境要求

### 硬件

- RealMan RM65-B 机械臂及控制器
- Windows PC，与机械臂控制器网络互通
- 支持 OpenXR 的 VR/XR 设备，例如 Meta Quest
- 可选：提供 HTTP `open`/`close` 接口的外部夹爪服务（当前 Grip 控制已禁用）

### 软件

- Windows 10/11
- Conda 环境 `VRControl`，Python 3.12
- 控制 SDK：仓库自带的 RealMan API 4.2.8（Windows 64 位 DLL）
- 离线运动学和安全算法：`Robotic_Arm==1.1.6`，仅使用 `Algo`，不建立 API2 连接
- NumPy、SciPy、Requests
- Unity `6000.5.6f1`，或能够兼容打开该工程的 Unity 版本
- Meta Quest Link 和 OpenXR Runtime（使用 Quest 时）

## Python 安装

创建并激活环境：

```powershell
conda create -n VRControl python=3.12
conda activate VRControl
```

安装 Python 依赖：

```powershell
python -m pip install -r requirements-rm65bi.txt
```

确认所有依赖来自当前 Conda 环境：

```powershell
python -c "import numpy, scipy, requests, urllib3; from Robotic_Arm.rm_robot_interface import Algo; print(requests.__file__)"
```

路径应位于：

```text
C:\Users\<用户名>\.conda\envs\VRControl\Lib\site-packages
```

如果 Python 错误地加载了 `AppData\Roaming\Python` 中的包，可在当前 PowerShell 会话中禁止用户级包：

```powershell
$env:PYTHONNOUSERSITE="1"
```

## 网络与机械臂准备

1. 将电脑有线网卡设置到与机械臂相同的网段，例如 `192.168.1.100/24`。
2. 确认电脑 IP 不能与机械臂的 `192.168.1.18` 重复。
3. 给机械臂上电，释放急停，在示教器中清除报警并使能机械臂。
4. 将机械臂移动到远离奇异点、桌面和其他障碍物的安全初始姿态。一个经 RM65-B URDF 和厂商正解验证的候选为 `[0°, -75°, 60°, 0°, 60°, 0°]`，但仍须按现场工具和障碍物确认路径。
5. 检查网络：

```powershell
ping 192.168.1.18
Test-NetConnection 192.168.1.18 -Port 8080
```

端口测试中的 `TcpTestSucceeded` 应为 `True`。本项目不会自动完成机械臂上电、清错或使能。

## RM65-BI 连接自检

安装依赖后，先执行只读检查，无需启动 Unity：

```powershell
python -m Realman_Control.check_connection
```

自检读取型号、版本、关节角、示教器限位、TCP 和工具坐标系，并初始化离线安全算法。它只断开连接，不发送运动、慢停、清错、使能或固件升级命令。确认成功后按原流程启动 Unity，再执行：

```powershell
python -m teleop.vr_teleop
```

旧 SDK 不提供新版 `trajectory_mode`/`radio` 透传滤波参数，二者只接受 `0`；现有遥操作仍由本地控制器按固定周期生成目标。`speed_tool()` 笛卡尔速度透传和非零 `blend_percent` 会明确报不支持。

## Unity 与 Quest 设置

1. 使用 Unity Hub 添加仓库中的 `VR-Teleop` 文件夹。
2. 打开 `Assets/Scenes/Main.unity`。
3. 使用 Quest 时，在 Meta Quest Link 中启用 OpenXR Runtime，并通过 Link 或 Air Link 连接头显。
4. 点击 Unity 的 **Play**。
5. 确认右手控制器在场景中能够正确跟踪。
6. Unity Console 应显示：

```text
Server started at 127.0.0.1:5555
Waiting for connection...
```

Unity 必须先进入 Play 模式，因为 Unity 是 TCP 服务端，而 Python 是主动连接它的客户端。

手柄发送逻辑位于：

```text
VR-Teleop/Assets/Scripts/SendVRControllerData.cs
```

## 启动遥操作

在仓库根目录打开 PowerShell：

```powershell
conda activate VRControl
$env:PYTHONNOUSERSITE="1"
python -m teleop.vr_teleop
```

正常启动时会看到：

```text
Connected to Unity server
```

程序没有单独打印“RM65 connected”。如果 RM65-B 连接失败，`RM65B` 初始化会直接抛出异常；没有异常且程序持续运行，表示已经进入等待手柄输入的状态。

## 手柄控制

当前场景读取右手控制器：

| 手柄输入 | Unity Input Action | 功能 |
|---|---|---|
| 右手食指扳机 | `Activate / TriggerButton` | 按住时跟随，松开时暂停 |
| 右手侧面 Grip | `Select / GripButton` | 已禁用，不触发夹爪动作 |

### VR 基坐标方向标定

Trigger 建立的相对锚点只负责“重新归零”，不属于坐标方向标定。当前程序提供独立的 VR→RM65 基坐标标定，操作键均在运行 Python 的终端中输入：

1. 松开 Trigger，并通过示教器确认机械臂处于所需状态。
2. 在 Python 终端按 `Q`。程序会暂停生成 VR 跟随命令，但不会调用 `rm_set_arm_slow_stop`；如果 Trigger 仍按住，会先等待释放。
3. 将手柄放在舒适的标定原点，保持静止，按 `Enter`；程序采集 `40` 帧并取均值。
4. 从同一原点沿示教器显示的机器人基坐标 `+X` 移动约 `10～15 cm`，保持静止后按 `Enter`。
5. 回到同一原点，沿基坐标 `+Y` 移动约 `10～15 cm`，保持静止后按 `Enter`。
6. 再回到原点，沿基坐标 `+Z` 移动约 `10～15 cm`，保持静止后按 `Enter`。
7. 程序检查手柄抖动、移动距离、三轴夹角、右手性和拟合误差，然后显示 `3x3` 映射矩阵。
8. 按 `Y` 保存并立即启用；按 `R` 清除本轮样本并重做；按 `Esc` 取消并保留原映射。
9. 标定完成或取消后，先确认 Trigger 已释放，再重新按住 Trigger 建立新的跟随锚点。

三轴正方向必须以机器人基坐标为准，满足右手规则 `+X × +Y = +Z`。例如把 `+X` 定义为向前、`+Z` 定义为向上时，`+Y` 应向左，不能直接将“前、右、上”当作 `+X/+Y/+Z`。`sampled axes are not a clear right-handed` 表示三轴组合未通过右手性检查；程序会显示归一化三重积。负值表示方向组合构成左手系，常见原因是某轴正负方向取反或采样顺序交换。应结合报错后的四个采样位置及实际移动方向核对，而不是直接翻转某一轴来通过检查。

标定矩阵是正交旋转矩阵，因此不会改变 `1:1` 距离比例。结果会原子写入 `config/vr_position_calibration.json`，下次启动自动校验并加载；文件缺失或损坏时使用旧的固定映射 `diag(-1,-1,1)`。重置 XR Guardian/边界、重新定向 XR Origin、移动 VR 基站或修改 Unity 坐标转换后，应重新标定。

该流程只标定 XYZ 平移方向，不标定机器人关节零位、TCP、负载或 J6 腕转符号；这些仍分别以示教器配置和 `wrist_roll_sign` 为准。标定期间程序不发送新的 `servo_joint` 目标，也不发送慢停命令，物理保持、减速或停机由 RM65 控制器与示教器决定。

### 开始跟随

程序启动后默认处于暂停状态。将右手控制器放在舒适位置，然后按住食指扳机。刚按下时终端显示：

```text
Resuming teleoperation...
```

此时程序将当前手柄姿态、当前六关节角和由 RM65-B 模型计算的 TCP 位置设置为锚点。保持按住期间，后续手柄运动以这个锚点为基准，不会直接使用 Unity 的绝对位置。

位置距离比例为 `1:1`：手柄移动 10 cm，机械臂目标也移动 10 cm。

完成标定后，手柄平移会按照机器人基坐标映射：

- 手柄前后运动 → 机械臂基/工作坐标系 X 方向
- 手柄左右运动 → 机械臂基坐标系 Y 方向
- 手柄上下运动 → 机械臂基/工作坐标系 Z 方向
- 手柄 XYZ 平移 → 只由 J1～J5 的位置逆解跟随
- 手柄绕 Aim 指向轴的 twist（像拧螺丝刀）→ 只控制 J6
- 手柄其余 pitch/yaw 旋转 → 不进入机械臂逆解

右手位置优先读取 `devicePosition`（Grip/握持中心），腕转仍从 `pointerRotation`（Aim 朝向）提取，避免仅转手腕时 Pointer 原点绕握持中心产生伪平移。Unity 到 J6 的校准符号为 `+1`；首次必须低速确认正反方向，若相反则修改 `wrist_roll_sign`。

该模式只约束 TCP 的 XYZ，不维持完整工具姿态。J1～J5 在位置任务的零空间中会优先保持 Trigger 按下时的关节构型，并受到连续性、限位和单周期步长约束。

### 松开暂停和重新建立锚点

松开食指扳机后，终端显示：

```text
Pausing command generation...
```

暂停后可以重新调整手柄位置，再次按住扳机即可恢复；每次重新按下时会建立一次新锚点，作用类似遥操作离合器。持续按住不会重复建立锚点。

暂停逻辑只是不再生成新的跟随目标，不会调用 `rm_set_arm_slow_stop`，也不等同于机械臂硬件急停。CAN-FD 数据中断后的保持、减速或停机行为由 RM65 控制器和示教器配置决定；出现非预期运动时必须使用硬件急停。

### 外部夹爪（已禁用）

右手侧面 Grip 已从 Unity 发送逻辑中解除绑定。为保持 TCP 数据格式兼容，第九个字段仍然保留但始终发送 `0`；Python 端也会忽略旧版 Unity 客户端发送的该字段。因此按下 Grip 不会再向外部 HTTP 服务发送夹爪打开或关闭请求。

## 结束遥操作

建议按以下顺序退出：

1. 松开食指扳机暂停跟随。
2. 通过示教器停止机械臂并确认其状态。
3. 在 Python 终端按 `Ctrl+C`。
4. Python 只断开 API 4.2.8 连接，不发送慢停命令。
5. 停止 Unity Play 模式。

## 记录 RM65-B 实机状态

独立状态记录器会保存与参考 `RM65_state_*.csv/.npz` 相同结构的数据，包括系统时间戳、单调时间戳、6 轴关节角、TCP 位置、TCP 四元数、SDK 错误码和机械臂状态。

持续记录，按 `Ctrl+C` 停止并保存：

```powershell
conda activate VRControl
python -m Realman_Control.rm65_state_logger
```

记录固定 60 秒：

```powershell
python -m Realman_Control.rm65_state_logger --duration 60
```

如果自动检测的电脑网卡 IP 不正确，手动指定机械臂网段中的电脑 IP：

```powershell
python -m Realman_Control.rm65_state_logger `
  --local-ip 192.168.1.100 `
  --duration 60
```

默认输出目录：

```text
C:\RM65Logs
```

输出文件名：

```text
RM65_state_YYYYMMDD_HHMMSS_nnnnnnnnn.csv
RM65_state_YYYYMMDD_HHMMSS_nnnnnnnnn.npz
```

CSV 在采集过程中持续写入；NPZ 在正常结束、达到指定时长或按下 `Ctrl+C` 后生成。记录器不发送运动命令，也不会在退出时停止另一个进程正在控制的机械臂。它会临时设置控制器的 UDP 实时上报目标，并在退出时恢复原有配置。

NPZ 的主要字段包括：

```text
device_time_ns
device_time_is_estimated
device_sample_index
estimated_missing_cycles
mono_ns
system_time_ns
sdk_error_code
arm_current_status
joint_rad
tcp_position_base_m
tcp_quaternion_base_xyzw
device_push_cycle_ns
device_time_source
native_controller_timestamp_available
queue_overflow_count
```

当前 RealMan 实时回调结构没有提供原生控制器时间戳，因此 `device_time_ns` 根据实时上报周期估算，`device_time_is_estimated` 为 `1`。

## 常见问题

### `ModuleNotFoundError: No module named 'urllib3'`

当前 Conda 环境中缺少完整的 Requests 依赖，而 Python 加载了用户目录中的残缺包。执行：

```powershell
conda activate VRControl
conda install -n VRControl requests
$env:PYTHONNOUSERSITE="1"
```

然后重新运行遥操作程序。

### Python 无法连接 `127.0.0.1:5555`

确认 Unity 已打开 `Main` 场景并进入 Play 模式，而且 Console 中显示 `Waiting for connection...`。检查是否有其他程序占用了 TCP 5555 端口。

### 无法连接 RM65-B

检查：

- 机械臂 IP 是否确实为 `192.168.1.18`
- 电脑和机械臂是否处于同一网段
- TCP 8080 是否可达
- 急停、报警、上电和使能状态是否正常
- 是否使用 Windows 64 位 Python，以及仓库内的 API 4.2.8 DLL 是否完整
- `Robotic_Arm==1.1.6` 是否可用于离线安全计算；不要用其 `RoboticArm` 类连接 1.4.x 控制器

### `joint target is near a shoulder singularity`

这表示准备发送的目标姿态进入了厂商算法判定的肩部奇异区，不是关节限位，也不是碰撞距离。消息中的 `shoulder distance` 是腕部中心点到肩部奇异平面的距离；默认解析法阈值为 `0.05 m`，数值越接近零越接近肩部奇异。例如 `0.0245 m` 表示约 `2.45 cm`，已经位于默认肩部奇异范围内。程序会保留最新 VR 目标；完整关节步长不安全时，再验证一个正常步长 `2.5%` 的小步并在通过后发送。若小步仍进入奇异区，就以 `100 Hz` 重发最后一个安全目标，不调用慢停，也不会慢速穿过奇异边界。松开 Trigger，用示教器将腕部中心移离基座/肩部奇异平面，并从非奇异姿态重新建立锚点。

### 状态记录器收不到数据

手动传入正确的 `--local-ip`，并允许 Python 通过 Windows 防火墙接收 UDP 8089。`--local-ip` 必须是电脑网卡地址，不能填写机械臂 IP。

旧版 UDP 回调没有 `arm_current_status`，状态记录器将该列写为 `-1`（未知），不伪装成空闲状态。`--cycle-ms` 仍表示毫秒，内部转换为控制器使用的 5 ms 周期计数；例如 `20 ms` 对应 `cycle=4`。记录器读取并保存原始 UDP 配置，退出时恢复。

## 控制接口说明

`Realman_Control/realman_control.py` 对外保持与原 UR10 RTDE 封装相近的单位和方法：

- 关节角：弧度
- TCP 位置：米
- TCP 姿态：旋转向量，弧度
- `move_joint()`：规划关节运动
- `move_tool()`：规划直线末端运动
- `servo_joint()`：CAN-FD 关节位置透传
- `servo_tool()`：CAN-FD 末端位姿透传，保留给其他调用者
- `stop()`：机械臂慢停
- `close()`：断开 API 4.2.8 连接

RealMan API 4.2.8 内部使用关节角度制和欧拉角，封装会在接口边界完成转换。

当前 VR 遥操作使用 `OfficialPositionIK` 调用官方离线 IK/FK，再通过 `servo_joint()` 发送关节角。J1～J5 距离关节上下限各保留 `5°`，J6 各保留 `15°`；工作区、关节速度及实际 TCP 步长继续受原有配置约束。每周期只使用最新手柄意图。非收敛候选必须确实接近本周期子目标和最终目标，并通过步长、限位、奇异性及自碰撞检查。完整候选不安全时只尝试经检查的 `2.5%` 小步；若仍不安全，则保持最后安全目标并丢弃被拒绝的偏差，便于手柄立即向安全方向退让。跟随误差仍与最近 `0～150 ms` 命令包络比较，超阈值时保持并周期复查。最终保护停机、超时保持和急停行为由 RM65 控制器及示教器决定。

## 官方 IK 与奇异区恢复

当前遥操作默认使用 `teleop/official_position_ik.py` 中的 `OfficialPositionIK`，由官方离线 `Algo.rm_algo_inverse_kinematics()` 生成关节候选，并以官方 FK 计算 TCP 位置、实际步长和进展。官方接口接收完整位姿，没有直接的“XYZ + 固定 J6”标志：适配层在当前朝向无法继续推进 XYZ 或影响连续性时，搜索邻近朝向，再连续优化三个朝向参数。优化器只选择朝向，关节解仍全部由官方 IK 产生。朝向、关节连续性、参考姿态和远离肘部奇异区仅为软偏好，不是固定姿态任务；J6 始终按独立腕转子目标设置，随后用官方 FK 重新核对 XYZ，偏轴工具也不例外。单周期最多调用官方 IK 16 次，生产控制循环另有周期 60% 的软计算预算（100 Hz 时为 6 ms）；预算耗尽后使用已验证的部分步长或保持，下一周期继续。旧 `RM65DecoupledIK` 仅保留供显式注入的离线测试使用，无本地关节逆解回退。

奇异性或碰撞检查拒绝候选后，程序保持最后安全关节目标，并丢弃该次被拒绝的平移/腕转偏差。这样，后续向安全方向的小幅手柄移动可以立即从最后安全位置开始，无需先抵消不可达偏差或重新按 Trigger。原始 Trigger 锚点的 0.25 m 工作区、关节限位及步长、J6 范围、奇异点和自碰撞检查均保留。

以下情况需分开判断：

- `... rejected offset discarded ...`：运动候选受限，保持安全位置，手柄向安全方向退让即可；Trigger 状态不会因此锁住。
- `tracking target retained while command is held: J... following error ...`：实际关节没有跟上已发送目标，按原阈值周期复查并保持；不会通过自动重置反馈来绕过真实跟随误差。
- `VR input rejected ...`：XR 输入源错误或丢失跟踪。这里仍要求恢复实体右手柄跟踪后松开 Trigger 再重新按下，避免恢复时姿态跳变。这不是机器人奇异点检查触发的锁定。

姿态自由度回归覆盖了实际法兰位置的已知反例：保持原朝向时官方完整位姿 IK 失败，但同 J6 的位置存在可行解。修复后的模拟跟随能将该 12.5 cm 目标的最终误差降至 0.02 mm 内。另有 8 个同 J6、工作区内且已知关节路径采样通过检查的目标，均在离线模拟中到达 0.02 mm 内。普通/偏轴工具、释放姿态后的连续往返、J6 独立性和原有安全检查均有测试；这些结果不保证全局可达，也不代替实机运动验证。

快速运动的连续性处理：朝向搜索的邻域随 XYZ 步长缩小，候选按关节位移和前一周期速度连续性筛选，再做连续朝向优化，避免在接近终点时仍用固定角度跳换姿态。Trigger 重新建立锚点时会重置连续性历史。平移和腕转均采用 `0.15` 的指数平滑，在 100 Hz 下增加约 60 ms 的跟随缓冲；位置求解容差为 `0.02 mm`。官方 FK 使用单精度，笛卡尔步长缩放会在硬阈值内保留至多 `1 μm` 余量，避免浮点量化误拒绝；数值求解暂时失败时保留最新目标重试，仅明确的关节/奇异性/碰撞拒绝才清除被拒绝的偏差。所有最终下发候选仍经原有安全检查；这不是严格的加速度或加加速度轨迹规划。

## 手柄位移比例排查

若实际平移 12 cm，但标定只测到约 3 cm，可使用独立的分层诊断。该程序只连接 Unity，不导入机器人 SDK，也不发送机器人命令。

1. 退出 `teleop.vr_teleop`，在当前仓库的 `VR-Teleop` 项目中重新进入 Play，确保新版发送脚本已经编译。
2. 运行：

```powershell
python -m teleop.vr_pose_diagnostics
```

3. 手柄放在尺子的起点，在诊断终端按 Enter 或 R 归零，再平移 12 cm，保持静止约 2 秒。

程序对比 XR 原始位置、Unity 局部位置、Unity 世界位置、Unity 发送位置和 Python 接收位置，同时显示父级累计缩放、跟踪状态和实际 Unity 项目路径。原始运动记录仍保持 11 字段；新增 `#VR_DIAG` 行每秒最多约 4 条，遥操作接收端会跳过这些诊断行。

诊断还会记录 Position、Rotation 等输入动作实际使用的设备和右手设备列表；`XRNode.RightHand` 返回的设备不一定就是驱动位置输入的设备，不能只根据该接口的设备名称判断输入来源。只有原始位置有效、`isTracked=true` 且跟踪状态包含位置有效位时，才计算原始位移比例。

发送端仅接受右手实体 Touch 系列手柄的位置和旋转，要求两者来自同一设备、`isTracked=true`，并同时具有位置和旋转有效位。手部跟踪、丢失跟踪或设备不一致时，发送 `#VR_INPUT_INVALID` 状态行，停止发送运动采样。Python 会清除待跟随目标、暂停遥操作并取消本轮标定，保留已保存的标定；恢复有效跟踪并松开 Trigger 后才可再次跟随。需要重新按 Q 开始标定。该检查保留原来的坐标映射和距离比例。

- XR 原始位移约 12 cm、世界位移约 3 cm：重点检查父级 Transform 或运行时位置处理。
- XR 原始位移也约 3 cm：继续检查 XR 跟踪源、设备选择和采样时机。
- Unity 发送位移约 12 cm、Python 接收位移约 3 cm：检查接收的是哪份 Unity 项目、帧序号与日志。

诊断原始记录默认保存在 `.log_analysis/vr_pose_diagnostics_*.jsonl`。

## RM65BI 分支验证

2026-11-18 在当前 RM65-BI 实机上通过 API 4.2.8 完成只读连接检查：版本、关节角、关节限位、TCP、工具坐标系均可正常读取；离线安全算法可正常初始化。

SDK 相关测试共 30 项通过，包含真实 DLL 与本地模拟控制器之间的旧版握手、关节/位姿透传单位、UDP 周期及异常清理测试。实际机械臂上未执行运动测试。

全量测试为 115 项，其中 106 项通过，9 项 VR 跟踪测试失败；在未修改的 `main`（`c255c15`）副本上复现了相同的 9 项失败，无新增失败或导入错误。新增测试覆盖偏轴 TCP、独立 J6、官方失败处理、不松 Trigger 退让、72 Hz 输入下的快速/慢速连续轨迹、单精度 FK 步长边界、原朝向不可解的位置目标、释放姿态后的往返连续性、安全候选筛选及软时间预算。关节限位、速度上限和工作区半径保持原值。此前 Unity 只读实测已确认：正常跟踪的 Touch 数据可以通过；丢失跟踪或切换到手部跟踪时，发送无效状态并拦截运动采样。

```powershell
python -m unittest discover -s tests -p test_realman_control.py
python -m unittest discover -s tests -p test_legacy_sdk.py
python -m unittest discover -s tests -p test_rm65_state_logger.py
```

## 参考资料

- [RealMan Robotics 开发者文档](https://develop.realman-robotics.com/robot/summarize/)
- [RealMan API 4.2.8 官方示例与 SDK 附件](https://bbs.realman-robotics.cn/question/316.html)
- [API2 离线算法参考](https://develop.realman-robotics.com/robot/apipython/classes/algo/)
- [Unity 官方网站](https://unity.com/)
- [Meta Quest Link 设置](https://www.meta.com/help/quest/articles/headsets-and-accessories/oculus-link/connect-with-air-link/)

## License

本项目沿用仓库根目录中的 [MIT License](LICENSE)。
