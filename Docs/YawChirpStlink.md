# Yaw 单轴扫频辨识（ST-Link 采样）

固件分支：`feat/yaw-chirp-stlink`。上电前先用 DM 工具将 DM8009P 的绝对值编码器零点设在飞镖 yaw 的机械中位，并确认驱动器处于 MIT 模式。这里不重新设零点，直接将电机位置反馈 `0 rad` 当作 yaw 中位。

遥控器左开关拨到上、右开关拨到下时，电机任务在当前位置不超过零点 ±5° 且电机已使能的条件下开始一次辨识。保持拨档时只运行一次；松开任一拨档会停止激励，先以力矩阻尼减速，待停止后恢复常规 yaw 控制。辨识期间 SysCtrl 向发射机构发送 `DART_RELAX`。

激励是 MIT 力矩前馈的线性变频正弦，幅值 `2 N·m`，频率从 `0.2 Hz` 线性增加至 `3 Hz`，时长 `20 s`。MIT 帧中的位置、速度、Kp、Kd 均按现有库设为零。达到电机零点 ±20° 时提前终止扫频并进行阻尼制动；±30° 是本次辨识的目标活动范围。软件只根据反馈提前制动，不能在机械惯量、编码器零点错误或通信中断时保证绝对不会越过 ±30°，实机应先以低幅值确认方向和停止距离。

用 ST-Link 监视全局变量 `yaw_ident_debug`，每个 MotorThread 周期更新一次。建议至少记录：

| 字段 | 含义 |
|---|---|
| `timestamp_us` | DWT 微秒时间戳，32 位，回绕时按无符号差分处理 |
| `sample_seq` | 电机任务采样序号，可检查 ST-Link 是否漏采 |
| `motor_rx_seq` | 电机 CAN 反馈计数，可识别反馈是否更新 |
| `state` | 0 空闲、1 扫频、2 制动、3 已结束 |
| `torque_command_nm` | 本周期下发的 MIT 力矩命令，N·m |
| `torque_feedback_nm` | 电机反馈帧解码得到的力矩，N·m |
| `gyro_yaw_rad_s` | BMI088 的 `ins.gyro_y`，rad/s；这是 yaw 输出角速度 |
| `motor_speed_rad_s` | DM8009P 反馈速度，rad/s；当前库按 MIT 协议原值解码，属于电机侧 |
| `position_rad` | 电机编码器反馈位置，rad，用来检查活动范围 |
| `elapsed_s`, `frequency_hz` | 扫频时刻和瞬时频率 |

MATLAB 辨识时可用 `torque_command_nm` 或 `torque_feedback_nm` 作输入，用 `gyro_yaw_rad_s` 作输出，按 `timestamp_us` 重建时间轴。注意电机反馈力矩是驱动器估计值，电机侧速度与 yaw 输出轴速度不宜直接当作同一量。

当前 DM8009P 库的映射范围为 `P_MIN/P_MAX = -12.5664/+12.5664 rad`、`V_MIN/V_MAX = -15/+15 rad/s`、`T_MIN/T_MAX = -20/+20 N·m`，定义位于 `User/Module/DMMotor/Src/DM8009P.cpp`。这些是 MIT 编解码范围，不是本实验的力矩幅值。

所给 GitHub 仓库在本环境中无法获取，因此上述线性变频参数没有从该仓库逐项复制。参数集中放在 `User/Task/Src/TaskMotor.cpp` 的 `yaw_ident_*` 常量处，便于按实机响应调整。
