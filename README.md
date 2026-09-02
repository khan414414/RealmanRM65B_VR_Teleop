# AUBO i7 VR Teleoperation

使用 VR/XR 手柄遥操作 AUBO i7 六轴机械臂，在 Unity 中同步显示六轴数字孪生，并记录实机关节状态和末端位姿。

本项目改造自原始 `UR10_Teleop`：Unity 端继续负责读取手柄数据，Python 端已切换为通过 AUBO ARCS 官方 Python SDK（`pyaubo_sdk`）的 RPC 服务控制 i7。仓库中的 `UR10_RTDE` 目录仅作为原始接口参考，当前遥操作入口不会使用 UR10 RTDE 控制机械臂。

## 安全警告

本项目会向真实机械臂发送 `servoJoint` 关节位置流式命令。首次运行前请务必做到：

- 清空机械臂工作空间，并确认没有人员位于运动范围内。
- 保持硬件急停按钮随时可用。
- 在机械臂安全配置和低速条件下测试。
- 首次只移动手柄 1～2 cm，确认坐标方向后再扩大动作。
- 不要把松开手柄 Trigger 后的“暂停跟随”当作安全急停。

当前遥操作层会检查关节步长、关节余量、本地奇异位形、实机跟随误差、逆解状态、胶囊近似自碰撞以及场景 Plane。超出软阈值时保留最新目标并按速度上限逐周期逼近；大于 `45°` 的单帧手柄腕转也会分段吸收。完整一步触发安全检查时，只尝试正常关节步长 `2.5%` 的已验证小步。若连小步也不安全，则保持最后一个安全目标，但不会调用机械臂停机接口。胶囊模型不是完整 CAD/环境碰撞模型；现场夹具、线缆、人员和 Plane 之外的物体不在模型内，最终关节限位、碰撞、停机和急停仍以控制器与示教器为准。

## 系统组成

数据流如下：

```text
右手 VR 控制器
    ↓ 位置、四元数、按键
Unity: VRControllerSender.cs
    ↓ 每个 Meta/Unity XR Update 一帧，附序号和单调时间戳
    ↓ TCP 127.0.0.1:5555（全双工）
Python: teleop/vr_teleop.py
    ↓ 最新值缓冲，固定 100 Hz 绝对时钟重采样
    ↓ 相对 XYZ、局部 wrist twist
VRTracker / AuboI7DecoupledIK
    ↓ J1～J5 位置逆解，J6 独立腕转
    ↓ 通过安全检查并被 servoJoint 接受的同一组 q1～q6
    ├→ AuboI7 / AUBO ARCS Python SDK → i7 控制器与实机
    └→ 同一 TCP 反向回传 → Unity: AuboI7Visualizer.cs
                              → 场景中的 AUBO i7 Digital Twin
```

主要目录：

```text
VR-Teleop/                              Unity VR 输入、数字孪生与 TCP 服务端
VR-Teleop/Assets/Scripts/VRControllerSender.cs
                                        Quest 输入发送与关节命令接收
VR-Teleop/Assets/Scripts/AuboI7Visualizer.cs
                                        i7_30 运动学层级与简化外观
teleop/                                 遥操作数据解析、映射和夹爪请求
teleop/aubo_i7_decoupled_ik.py          i7 离线运动学与 J1～J5 位置逆解
Aubo_Control/aubo_control.py            AUBO ARCS RPC 控制封装
Aubo_Control/aubo_i7_state_logger.py    实机状态 CSV/NPZ 记录器
Aubo_Control/replay_aubo_i7_trajectory.py
                                        状态日志校验与安全回放
UR10_RTDE/                              原始 UR10 参考代码，当前入口不使用
```

## 默认配置

| 项目 | 默认值 | 位置 |
|---|---:|---|
| AUBO i7 控制器 | `192.168.127.128:30004` | `teleop/vr_teleop.py` |
| ARCS 登录凭据 | `aubo` / `123456` | `Aubo_Control/aubo_control.py` |
| Unity TCP 服务端 | `127.0.0.1:5555` | Unity 场景和 `teleop/vr_teleop.py` |
| Meta/Unity 发布频率 | XR Runtime 的实际帧率（常见 `72/80/90/120 Hz`） | `VRControllerSender.cs` |
| 机械臂命令频率 | 固定 `100 Hz`，错过周期时跳帧而不突发补发 | `teleop/vr_teleop.py` |
| Unity 关节镜像 | 容量为 1 的最新值缓冲；每个 Unity 帧应用最新 q1～q6 | 双向 TCP `J1` 协议 |
| Python 控制周期 | `0.01 s` | `teleop/vr_teleop.py` |
| VR→基坐标映射 | 前后与左右分别接到对应运动通道，并单独反转左右方向；缺失标定时有效映射为 `[[0,-1,0],[1,0,0],[0,0,1]]` | `teleop/vr_teleop.py` |
| 位置距离比例 | `1.5:1`（手柄 1 cm → TCP 目标 1.5 cm） | `teleop/vr_teleop.py` |
| 平滑系数 | `1.0`（不做指数平滑） | `teleop/vr_teleop.py` |
| VR 标定文件 | `config/aubo_i7_vr_position_calibration.json` | `teleop/vr_calibration.py` |
| 最大相对位置范围 | 不设置额外球形上限；由关节、速度、奇异和碰撞检查约束 | `teleop/vr_teleop.py` |
| 最大笛卡尔目标速度 | `0.50 m/s` | `teleop/vr_teleop.py` |
| 最大关节目标速度 | `180 deg/s` | `teleop/vr_teleop.py` |
| Plane 碰撞高度 | 机器人基坐标 `Z=-0.05 m`，另留 `5 mm` 间隙（对应场景 `Plane` 的 Unity `Y=-0.05 m`） | `teleop/aubo_i7_collision.py` |
| 运动学安全阈值附近的尝试步长 | 正常关节步长的 `2.5%` | `teleop/vr_tracker.py` |
| 外部夹爪服务（Grip 控制已禁用） | `130.215.216.42:8005` | `teleop/vr_teleop.py` |
| 状态日志目录 | `C:\AUBOLogs` | `Aubo_Control/aubo_i7_state_logger.py` |

> 关节限位优先读取控制器（示教器）配置；无法读取时回退到官方 `aubo_i7_30` URDF 的 `±360°` 模型限位。i7_10/i7_11 等旧硬件限位更窄，务必以控制器返回值为准。

## 环境要求

### 硬件

- AUBO i7 机械臂及 ARCS 控制器
- Windows PC，与机械臂控制器网络互通
- 支持 OpenXR 的 VR/XR 设备，例如 Meta Quest
- 可选：提供 HTTP `open`/`close` 接口的外部夹爪服务（当前 Grip 控制已禁用）

### 软件

- Windows 10/11
- Conda 环境 `VRControl`，Python 3.12
- AUBO ARCS Python SDK：`pyaubo_sdk`（随 ARCS 控制器发行，或向 AUBO 支持获取与控制器版本匹配的安装包）
- NumPy、SciPy、Requests
- Unity `6000.5.6f1`，或能够兼容打开该工程的 Unity 版本
- Meta Quest Link 和 OpenXR Runtime（使用 Quest 时）

## Python 安装

创建并激活环境：

```powershell
conda create -n VRControl python=3.12
conda activate VRControl
```

安装 Python 依赖（`pyaubo_sdk` 按现场提供的 wheel/目录安装）：

```powershell
python -m pip install numpy scipy requests
python -m pip install <现场的 pyaubo_sdk 安装包>
```

确认所有依赖来自当前 Conda 环境：

```powershell
python -c "import numpy, scipy, requests; import pyaubo_sdk; print('ok')"
```

路径应位于：

```text
C:\Users\<用户名>\.conda\envs\VRControl\Lib\site-packages
```

如果 Python 错误地加载了 `AppData\Roaming\Python` 中的包，可在当前 PowerShell 会话中禁止用户级包：

```powershell
$env:PYTHONNOUSERSITE="1"
```

> `pyaubo_sdk` 是可选运行时依赖：未安装时仍可运行单元测试和离线运动学，只有在实例化 `AuboI7` 连接实机时才需要它。

## 网络与机械臂准备

1. 将电脑有线网卡设置到与机械臂控制器相同的网段，例如 `192.168.127.100/24`。
2. 确认电脑 IP 不能与控制器的 `192.168.127.128` 重复。
3. 给机械臂上电，释放急停，在示教器中清除报警并使能机械臂。
4. 将机械臂移动到远离奇异点、桌面和其他障碍物的安全初始姿态。一个通过本地胶囊自碰撞、场景 Plane（机器人基坐标 `Z=-0.05 m`）和 AUBO i7 URDF 奇异性检查的候选为 `[0°, -60°, -75°, 0°, -75°, 0°]`，但仍须按现场工具和障碍物确认路径。
5. 检查网络：

```powershell
ping 192.168.127.128
Test-NetConnection 192.168.127.128 -Port 30004
```

端口测试中的 `TcpTestSucceeded` 应为 `True`。本项目不会自动完成机械臂上电、清错或使能。

## Unity 与 Quest 设置

1. 使用 Unity Hub 添加仓库中的 `VR-Teleop` 文件夹。
2. 打开 `Assets/Scenes/Main.unity`。
3. 使用 Quest 时，在 Meta Quest Link 中启用 OpenXR Runtime，并通过 Link 或 Air Link 连接头显。
   Standalone OpenXR 配置已同时启用 Oculus Touch 与 Meta Quest Touch Plus Controller Profile。
4. 点击 Unity 的 **Play**。
5. 确认 Hierarchy 中有独立于 `XR Origin` 的 **AUBO i7 Digital Twin**；进入 Play 后会按 i7_30 官方关节原点生成六轴简化模型。
6. 确认右手控制器在场景中能够正确跟踪。
7. Unity Console 应显示：

```text
Server started at 127.0.0.1:5555
Waiting for connection...
```

Unity 必须先进入 Play 模式，因为 Unity 是 TCP 服务端，而 Python 是主动连接它的客户端。

手柄发送和关节镜像接收逻辑位于：

```text
VR-Teleop/Assets/Scripts/VRControllerSender.cs
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
SIMULATION MODE: no real AUBO connection has been opened.
Press L in this terminal to connect the real AUBO i7; release the Trigger before switching.
```

程序默认只连接 Unity，并使用本地关节级仿真后端，不会访问 `192.168.127.128:30004`。此时 Quest 3 输入经过与实机相同的 IK、限速和安全检查后驱动 Unity 中的 AUBO i7。

只有在运行 Python 的终端获得键盘焦点并按下 `L`（大小写均可）后，程序才会连接真机、启用 servo 模式并切换到真实关节后端。切换前先松开 Trigger；切换成功后还必须保持松开至少一帧，再重新按下 Trigger 才会开始驱动。成功时会看到：

```text
REAL ROBOT MODE: AUBO servo mode enabled. Release the Trigger, then press it again to begin tracking.
```

如果 RPC 连接、登录、servo 模式或真机初始化失败，程序会打印 `AUBO CONNECTION FAILED`，关闭本次不完整连接并继续仿真，不会退出。再次按 `L` 可以重试。启动时终端会打印实际使用的关节限位来源；仿真使用 i7_30 模型回退，切换真机后使用控制器/示教器限位，请现场核对。

### Unity 虚实同步

Unity 不会从原始手柄位姿另算一套 IK。在仿真模式下，Python 把仿真后端接受的 q1～q6 发回 Unity；在真机模式下，Python 只有在 `servoJoint()` 成功接受某个六关节目标后，才把完全相同的 q1～q6 发回 Unity。正常跟踪、输入陈旧时的 keepalive、以及安全保持三条路径都会镜像，因此两种模式使用同一套限速、标定和安全处理逻辑。

回传使用换行分帧的 `J1` 协议：

```text
J1,command_seq,source_unity_seq,source_unity_time,python_time,mode,q1,q2,q3,q4,q5,q6
```

关节单位是弧度，顺序为 J1～J6。Python 的发送线程使用容量为 `1` 的最新值缓冲，Unity 暂停渲染或 TCP 背压不会阻塞 `100 Hz` 实机控制循环；Unity 恢复后直接采用最新姿态，不补播过时帧。

场景中的模型采用官方 `aubo_i7_30.urdf` 的六个关节原点、固定 RPY 和局部 `+Z` 转轴，外观由 Unity primitives 简化生成。Python 使用与这些 primitives 尺寸对应的球体/胶囊做保守的自碰撞与 Plane 预检查，但它仍不是 CAD 网格碰撞。当前镜像的是实机收到的**关节命令**，并非编码器实测反馈；若需要观察机械响应滞后或示教器移动，应另接 `get_joint_values()` 反馈流。

## 手柄控制

当前场景读取右手控制器：

| 手柄输入 | Unity Input Action | 功能 |
|---|---|---|
| 右手食指扳机 | `Activate / TriggerButton` | 按住时跟随，松开时暂停 |
| 右手侧面 Grip | `Select / GripButton` | 已禁用，不触发夹爪动作 |

### VR 基坐标方向标定

Trigger 建立的相对锚点只负责“重新归零”，不属于坐标方向标定。当前程序提供独立的 VR→AUBO 基坐标标定，操作键均在运行 Python 的终端中输入：

1. 松开 Trigger，并通过示教器确认机械臂处于所需状态。
2. 在 Python 终端按 `Q`。程序会暂停生成 VR 跟随命令；如果 Trigger 仍按住，会先等待释放。
3. 将手柄放在舒适的标定原点，保持静止，按 `Enter`；程序采集 `40` 帧并取均值。
4. 从同一原点沿示教器显示的机器人基坐标 `+X` 移动约 `10～15 cm`，保持静止后按 `Enter`。
5. 回到同一原点，沿基坐标 `+Y` 移动约 `10～15 cm`，保持静止后按 `Enter`。
6. 再回到原点，沿基坐标 `+Z` 移动约 `10～15 cm`，保持静止后按 `Enter`。
7. 程序检查手柄抖动、移动距离、三轴夹角、右手性和拟合误差，然后显示 `3x3` 映射矩阵。
8. 按 `Y` 保存并立即启用；按 `R` 清除本轮样本并重做；按 `Esc` 取消并保留原映射。
9. 标定完成或取消后，先确认 Trigger 已释放，再重新按住 Trigger 建立新的跟随锚点。

标定文件保存的是正交旋转矩阵；运行时在方向处理后统一乘以 `1.5` 的位移比例，并交换前后/左右两个输出通道、单独反转左右输出方向，上下方向保持不变。结果会原子写入 `config/aubo_i7_vr_position_calibration.json`，下次启动自动校验并加载；文件缺失或损坏时，有效固定映射为 `1.5 * [[0,-1,0],[1,0,0],[0,0,1]]`。重置 XR Guardian/边界、重新定向 XR Origin、移动 VR 基站或修改 Unity 坐标转换后，应重新标定。

该流程只标定 XYZ 平移方向，不标定机器人关节零位、TCP、负载或 J6 腕转符号；这些仍分别以示教器配置和 `wrist_roll_sign` 为准。标定期间程序不发送新的 `servoJoint` 目标，也不发送停机命令，物理保持、减速或停机由 AUBO 控制器与示教器决定。

### 开始跟随

程序启动后默认处于暂停状态。将右手控制器放在舒适位置，然后按住食指扳机。刚按下时终端显示：

```text
Resuming teleoperation...
```

此时程序将当前手柄姿态、当前六关节角和由 i7 模型计算的 TCP 位置设置为锚点。保持按住期间，后续手柄运动以这个锚点为基准，不会直接使用 Unity 的绝对位置。

位置距离比例为 `1.5:1`：手柄移动 10 cm，机械臂 TCP 目标移动 15 cm。

完成标定后，手柄平移会按照机器人基坐标映射：

- 手柄前后运动 → 机械臂基/工作坐标系 X 方向
- 手柄左右运动 → 机械臂基坐标系 Y 方向
- 手柄上下运动 → 机械臂基/工作坐标系 Z 方向
- 手柄 XYZ 平移 → 只由 J1～J5 的位置逆解跟随
- 手柄绕 Aim 指向轴的 twist（像拧螺丝刀）→ 只控制 J6
- 手柄其余 pitch/yaw 旋转 → 不进入机械臂逆解

右手位置优先读取 `devicePosition`（Grip/握持中心），腕转仍从 `pointerRotation`（Aim 朝向）提取，避免仅转手腕时 Pointer 原点绕握持中心产生伪平移。Unity 到 J6 的校准符号当前为 `+1`；首次必须低速确认正反方向，若相反则修改 `wrist_roll_sign`。

该模式只约束 TCP 的 XYZ，不维持完整工具姿态。J1～J5 在位置任务的零空间中会优先保持 Trigger 按下时的关节构型，并受到连续性、限位和单周期步长约束。若配置了偏轴工具偏移，J6 会影响 TCP 位置，此时 J1～J5 会对所选 J6 角度进行补偿。

### 松开暂停和重新建立锚点

松开食指扳机后，终端显示：

```text
Pausing command generation...
```

暂停后可以重新调整手柄位置，再次按住扳机即可恢复；每次重新按下时会建立一次新锚点，作用类似遥操作离合器。持续按住不会重复建立锚点。

暂停逻辑只是不再生成新的跟随目标，不等同于机械臂硬件急停。`servoJoint` 流中断后的保持、减速或停机行为由 AUBO 控制器和示教器配置决定；出现非预期运动时必须使用硬件急停。

### 外部夹爪（已禁用）

右手侧面 Grip 已从 Unity 发送逻辑中解除绑定。为保持 TCP 数据格式兼容，第九个字段仍然保留但始终发送 `0`；Python 端也会忽略旧版 Unity 客户端发送的该字段。因此按下 Grip 不会再向外部 HTTP 服务发送夹爪打开或关闭请求。

## 结束遥操作

建议按以下顺序退出：

1. 松开食指扳机暂停跟随。
2. 通过示教器停止机械臂并确认其状态。
3. 在 Python 终端按 `Ctrl+C`。
4. Python 会退出 servo 模式、登出并断开 RPC 连接，不发送额外的停机命令。
5. 停止 Unity Play 模式。

## 记录 i7 实机状态

独立状态记录器按固定周期轮询 ARCS RPC 状态，保存系统时间戳、单调时间戳、6 轴关节角、关节速度、TCP 位置、TCP 四元数和轮询错误标记。

持续记录，按 `Ctrl+C` 停止并保存：

```powershell
conda activate VRControl
python -m Aubo_Control.aubo_i7_state_logger
```

记录固定 60 秒：

```powershell
python -m Aubo_Control.aubo_i7_state_logger --duration 60
```

指定控制器地址与轮询周期：

```powershell
python -m Aubo_Control.aubo_i7_state_logger `
  --robot-ip 192.168.127.128 `
  --cycle-ms 20 `
  --duration 60
```

默认输出目录：

```text
C:\AUBOLogs
```

输出文件名：

```text
AUBO_i7_state_YYYYMMDD_HHMMSS_nnnnnnnnn.csv
AUBO_i7_state_YYYYMMDD_HHMMSS_nnnnnnnnn.npz
```

CSV 在采集过程中持续写入；NPZ 在正常结束、达到指定时长或按下 `Ctrl+C` 后生成。记录器不发送运动命令，也不会在退出时停止另一个进程正在控制的机械臂。

NPZ 的主要字段包括：

```text
device_time_ns
device_time_is_estimated
device_sample_index
estimated_missing_cycles
mono_ns
system_time_ns
poll_error
joint_rad
joint_speed_rad_s
tcp_position_base_m
tcp_quaternion_base_xyzw
device_push_cycle_ns
device_time_source
native_controller_timestamp_available
poll_error_count
```

当前 ARCS RPC 轮询接口没有提供原生控制器时间戳，因此 `device_time_ns` 根据轮询周期估算，`device_time_is_estimated` 为 `1`；含 `poll_error=1` 的样本表示该次 RPC 状态轮询失败，此时关节数据沿用上一有效值。

## 回放状态日志

先用干跑模式校验日志（默认行为），检查时间戳、关节限位、速度与单步上限：

```powershell
python -m Aubo_Control.replay_aubo_i7_trajectory C:\AUBOLogs\AUBO_i7_state_xxx.npz
```

确认安全后再实机执行（需按提示输入 `REPLAY` 口令确认）：

```powershell
python -m Aubo_Control.replay_aubo_i7_trajectory `
  C:\AUBOLogs\AUBO_i7_state_xxx.npz --execute --move-to-start
```

干跑使用 i7_30 URDF 模型限位；执行前会连接控制器，用其实时返回的关节限位再次校验，超出即拒绝回放。

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

### 无法连接 AUBO i7

检查：

- 控制器 IP 是否确实为 `192.168.127.128`，RPC 端口 `30004` 是否可达
- 电脑和控制器是否处于同一网段
- 急停、报警、上电和使能状态是否正常
- ARCS 账号密码是否为默认 `aubo` / `123456`（可在 `AuboI7` 构造参数中覆盖）
- `pyaubo_sdk` 是否安装在当前 Conda 环境，且版本与控制器 ARCS 版本匹配

### `AUBO servo mode is disabled; call start_servo_mode() first`

`servo_joint()` 要求先显式进入 servo 模式。遥操作主程序会在开始流式命令前自动处理；自行编写脚本时需先调用 `start_servo_mode()`，结束后调用 `stop_servo_mode()`。

### `joint target is near a singularity` / 奇异告警

这表示准备发送的目标构型进入了本地奇异性检查范围，不是关节限位，也不是碰撞距离。程序会保留最新 VR 目标；完整关节步长不安全时，再验证一个正常步长 `2.5%` 的小步并在通过后发送。若小步仍不安全，就以 `100 Hz` 重发最后一个安全目标，不调用停机，也不会慢速穿过奇异边界。松开 Trigger，用示教器将腕部中心移离奇异区域，并从非奇异姿态重新建立锚点。

### `self-collision risk` / `too close to the plane`

Python 会把六个关节壳体、五段连杆、法兰以及非零 TCP 工具偏移近似为球体/胶囊。每个准备发送的候选都必须与非相邻连杆保持 `5 mm` 间隙，并位于场景 `Plane` 上方至少 `5 mm`；否则保持上一安全目标。应用配置把当前 `Plane` 的 Unity `Y=-0.05 m` 换算为机器人基坐标 `Z=-0.05 m`。如果在 Unity 中移动 Plane 或 AUBO 根对象，必须同步修改 `UNITY_PLANE_ROBOT_Z_M`，否则虚拟高度和 Python 碰撞高度会不一致。

### J1～J5 到达限位时 J6 不转

旧逻辑把六轴候选作为整体，并要求候选继续缩小 TCP 位置误差；J1～J5 到达边界后无法继续改善位置，所以即便 J6 有新目标，整组候选也会因“无位置进展”被丢弃。现在主候选失败后会尝试 J6-only 降级：固定 J1～J5，仅按速度上限推进 J6。该降级仍需通过 J6 限位、实际 TCP 步长、奇异性、控制器安全、自碰撞和 Plane 检查；如果 J6 本身到达限位或转动会碰撞，它仍会被正确冻结。

### 状态记录器报 `poll_error` 增长

RPC 轮询失败通常意味着网络抖动或控制器忙。检查网线/网卡、确认没有其他程序同时通过 30004 端口控制同一台机械臂；少量失败样本会沿用上一有效值并计数，大量失败时应先停止记录再排查网络。

## 控制接口说明

`Aubo_Control/aubo_control.py` 对外保持与原 UR10 RTDE 封装相近的单位和方法：

- 关节角：弧度
- TCP 位置：米
- TCP 姿态：旋转向量，弧度（对应 ARCS `getTcpPose`/`getTcpOffset` 约定）
- `move_joint()`：`moveJoint` 规划关节运动（SI 单位）
- `servo_joint()`：`servoJoint` 流式关节控制（需先 `start_servo_mode()`；队列满返回码 `2` 时有界重试）
- `start_servo_mode()` / `stop_servo_mode()`：进入/退出 servo 模式
- `stop()`：`stopJoint` 受控停机
- `stop_script()`：受控停机并退出 servo 模式
- `get_joint_values()` / `get_joint_speed()` / `get_tool_pose()` / `get_tcp_offset()`：状态读取
- `get_joint_limits()`：读取并缓存控制器关节限位（优先于模型 URDF）
- `close()`：退出 servo、登出并断开 RPC

ARCS SDK 内部使用关节角度制和欧拉角时，封装会在接口边界完成转换。

当前 VR 遥操作使用 `AuboI7DecoupledIK` 后调用 `servo_joint()`，不做完整六维位姿逆解。六关节目标会在发送前逐项裁剪到示教器可动范围内的安全区间：J1～J5 距离上下限各保留 `5°`，J6 各保留 `15°`。程序只保留最新 VR 意图，不建立逐帧 FIFO，因此新鲜的新帧会替换旧目标，松开 Trigger 会清除待跟随目标。当前不设置额外的球形工作区上限；目标由关节余量、`180 deg/s` 关节步长、`0.50 m/s` 实际 FK 笛卡尔步长、奇异性、胶囊自碰撞和 Plane 共同约束。完整候选不安全时再尝试一个 `2.5%` 小步，仍不安全则保持上一目标。若 J1～J5 因限位或无位置进展被冻结，但存在新的腕转目标，程序会构造 J1～J5 不变的 J6-only 候选；它只有在仍通过 J6 限位、速度、奇异性、控制器、本地自碰撞和 Plane 检查后才会发送。following error 会把反馈与最近 `0～150 ms` 命令包络比较；真实误差超阈值时冻结全部命令，让机械臂追上当前目标。胶囊模型不包括完整 CAD、线缆和 Plane 之外的环境，最终碰撞保护、停机、超时保持和急停行为由 AUBO 控制器及示教器决定。

## 实机确认项

以下内容依赖现场控制器版本与配置，软件无法离线核实：

- 控制器实际 IP、RPC 端口（当前代码为 `192.168.127.128:30004`）与 ARCS 账号密码
- `pyaubo_sdk` 与控制器 ARCS 版本的兼容性
- 控制器上报的关节限位是否与该批次 i7 硬件一致（i7_10/i7_11/i7_30 限位不同）
- `servoJoint` 的 lookahead/gain 参数在 100 Hz 流下的实际跟踪表现
- `wrist_roll_sign` 的正反方向（首次低速确认）
- TCP 偏移配置与现场工具的一致性

## 参考资料

- [AUBO Robotics 官网](https://www.auborobotics.com/)
- AUBO ARCS 示教器与 `pyaubo_sdk` 随控制器发行的开发文档
- AUBO 官方 `aubo_description` 仓库中的 `aubo_i7` URDF 运动学参考
- [Unity 官方网站](https://unity.com/)
- [Meta Quest Link 设置](https://www.meta.com/help/quest/articles/headsets-and-accessories/oculus-link/connect-with-air-link/)

## License

本项目沿用仓库根目录中的 [MIT License](LICENSE)。
