# Yaw 单轴辨识与 UART10 采集

本分支 `feat/yaw-chirp-stlink` 使用 DM8009P 的绝对编码器零点作为 yaw 零点。遥控器左上、右下时，电机已使能、反馈有效且起始位置在零点 ±45° 内，就启动一次 MIT 力矩扫频。保持拨档时只运行一次；松开拨档会停止扫频并制动。

当前扫频配置在 `User/Task/Src/TaskMotor.cpp`：力矩幅值 8 N·m，频率从 0.4 Hz 线性升至 4 Hz，最长 30 s。到零点 ±46° 或反馈异常时进入制动；±52° 是制动结束判断的角度条件。这些数值是当前工程配置，实际运动范围仍应由机械条件确认。

## 接线与采集

USART10 配置为 **115200、8N1、3.3 V TTL**；**PE3 为 TX**，接 USB 转串口的 RX，另接公共 GND。UART7 仍供 G4 力传感器使用。此分支不启动原本占用 USART10 的 HostComm 线程，因此 HostComm 的串口功能暂不可用。

电脑安装 `pyserial` 后，可运行：

```powershell
python -m pip install pyserial
python Tools/yaw_uart_capture.py --port COM5 --output captures/yaw_uart.csv
```

把 `COM5` 改为实际串口；脚本只创建新文件，不覆盖已有记录。建议先开始电脑采集，再拨动遥控器。按 Ctrl+C 结束，CSV 可以用 MATLAB `readtable` 读取。串口约每 10 ms 发送一行，包括空闲及扫频后的状态；`state` 为 0 空闲、1 扫频、2 制动、3 结束。只用 `state == 1` 的行做扫频辨识，并用 `sample_seq` 或 `timestamp_us` 检查丢样。

CSV 字段的整数缩放单位如下：

| 字段 | 含义 |
| --- | --- |
| `timestamp_us` | DWT 微秒时间戳，32 位回绕 |
| `sample_seq` | 1 ms 电机任务样本序号 |
| `state` | 辨识状态，0/1/2/3 |
| `position_mrad` | 电机编码器位置，毫弧度 |
| `gyro_yaw_mrad_s` | BMI088 yaw 角速度，毫弧度每秒 |
| `motor_speed_mrad_s` | DM8009P 电机侧反馈速度，毫弧度每秒 |
| `torque_command_mNm` | MIT 命令力矩，毫牛米 |
| `torque_feedback_mNm` | 电机反馈力矩，毫牛米 |
| `frequency_mHz` | 瞬时扫频频率，毫赫兹 |
| `imu_status` | IMU 状态位：bit0/bit1 为 ID 错误，bit2 为尚无有效陀螺仪读数 |
| `motor_rx_seq` | 电机 CAN 反馈计数 |
| `imu_sample_seq` | IMU 有效样本计数 |

换算到 SI 单位时，把 `*_mrad`、`*_mrad_s`、`*_mNm`、`*_mHz` 除以 1000。若 `imu_status` 仍为 7、`imu_sample_seq` 始终为 0，BMI088 的 yaw 输出无效，可先检查 IMU，或仅分析电机侧速度。`torque_feedback_mNm` 是驱动器反馈值。

ST-Link 仍可直接监视每个电机任务周期更新的全局结构体 `yaw_ident_debug`，其中还保留了三个 gyro 轴的浮点数据以及扫频 elapsed time。`yaw_ident_uart_tx_fail_count` 记录 UART10 发送失败次数。

当前 DM8009P 库的 MIT 映射范围为 `P_MIN/P_MAX = -12.5664/+12.5664 rad`、`V_MIN/V_MAX = -15/+15 rad/s`、`T_MIN/T_MAX = -20/+20 N·m`，定义见 `User/Module/DMMotor/Src/DM8009P.cpp`。这些是协议编解码范围。
