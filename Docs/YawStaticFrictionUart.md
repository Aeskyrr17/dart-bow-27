# Yaw 静摩擦测试（UART10）

分支：`feat/yaw-static-friction-uart10`。保持遥控器左上、右下，启动一次自动测试；松开任一拨档立即把测试力矩置零。起动位置必须在 DM8009P 绝对编码器零点的 ±20° 内，电机反馈有效且当前速度接近零。测试时 SysCtrl 对发射机构发送 `DART_RELAX`。

每次测试先以 **MIT 零力矩**等待位置和电机速度连续稳定 0.5 s，再以 `0.5 N·m/s` 线性爬升，最大 `8 N·m`。正、负方向交替，各测 3 次，分别对应 trial 1/3/5 和 2/4/6。起动判据为：沿力矩方向的位置变化超过 `0.003 rad`，电机反馈速度超过 `0.05 rad/s`，且连续保持 20 ms。判到起动后立即把命令力矩清零，等再次稳定 0.5 s 后进入下一次。整个测试过程不调用 yaw 位置环或速度环；DM8009P 当前 MIT 打包代码把位置、速度、Kp、Kd 均设置为零。

爬坡前的零力矩反馈保存在 `baseline_feedback_mNm`。`pre_onset_*` 是满足起动判据之前最后一个静止样本；`onset_*` 是连续变化达到 20 ms 时的样本。它们会在下一次试验开始前保留，便于串口以 50 Hz 发送。驱动器反馈力矩是电机侧估计值；若线缆拉力或预紧存在，应分别比较正负方向及不同起始位置的阈值。

测试从 ±20° 内开始；编码器位置到 ±45° 时将命令力矩清零并标记中止，为用户提出的 ±50° 范围留出余量。清零不能消除机械惯性，实机仍需确认停止距离。达到 `8 N·m` 仍未起动也会中止。结束或中止后保持零 MIT 力矩直到松开拨档。

## 串口采集

USART10 为 `115200 8N1`、3.3 V TTL。PE3 (TX) 接 USB 转串口的 RX，并接公共 GND。UART7 仍用于 G4 力传感器；USART10 上原 HostComm 线程在此分支不启动。

先在电脑运行（将 COM5 改成实际端口）：

```powershell
python -m pip install pyserial
python Tools/yaw_uart_capture.py --port COM5 --output captures/yaw_friction.csv
```

脚本只创建新文件，不会覆盖已有文件。采集开始后拨动遥控器。按 Ctrl+C 结束，MATLAB 用 `readtable` 读取。若要换几个 yaw 位置，可完成一组后松开拨档，手动调整至零点 ±20° 内，再触发新的一组，并保存另一份 CSV。

在 MATLAB 中提取每次确认起动后的首行，例如：

```matlab
T = readtable('captures/yaw_friction.csv');
E = T(T.onset_seq > 0 & [true; diff(T.onset_seq) ~= 0], :);
E(:, {'trial','direction','trial_start_mrad', ...
       'pre_onset_feedback_mNm','onset_feedback_mNm'})
```

分别对正、负方向的反馈力矩阈值求均值和离散程度，并保留每次起始位置；有外载偏置时不要把两个方向直接合并。

串口持续输出约 50 行/s。CSV 中位置用 `mrad`、速度用 `mrad/s`、力矩用 `mNm`；除以 1000 即为 rad、rad/s、N·m。`state`：0 空闲，1 零力矩静止确认，2 爬坡，3 起动后等待静止，4 六次完成，5 中止。`direction` 为 +1 或 -1；`onset_seq` 为本组已测起动次数。`abort_reason`：0 无，1 达到角度边界，2 到最大力矩仍未起动，3 电机反馈异常，4 下一次起始位置超出 ±20°。`timestamp_us` 和 `onset_timestamp_us` 为 32 位微秒时间戳，约 71.6 分钟回绕；`sample_seq` 是 1 ms 电机任务计数，用于检查 UART 抽样间隔。

`gyro_yaw_mrad_s` 与 `imu_status` 也一并记录。若 `imu_status=7`、`imu_sample_seq=0`，BMI088 数据仍无效，判起动使用的是 DM8009P 电机侧反馈速度。可用 ST-Link 查看 `yaw_ident_debug` 的全部浮点字段，以及 `yaw_ident_uart_tx_fail_count`。
