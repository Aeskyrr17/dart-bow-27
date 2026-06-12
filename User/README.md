# 代码交接说明

这份 `User` 目录是本工程真正的业务代码区。CubeMX/HAL 生成的外设初始化在 `Core`、`USBX` 等目录里，电控通用架构和第三方库也有一部分已经固定下来；后续主要维护时，优先从 `User` 目录入手。

建议先按“数据从哪里来、被谁处理、最后发到哪里去”的思路读代码，不要一开始钻进每个驱动函数。

## 1. 总体架构

代码大致分为五层：

| 层级 | 目录 | 作用 |
| --- | --- | --- |
| BSP | `User/BSP` | 对 HAL 外设做一层薄封装，例如 CAN、UART、PWM、DWT |
| Module | `User/Module` | 具体设备或功能模块，例如电机、IMU、裁判系统 UI、ZDT 步进电机原厂协议 |
| Service | `User/Service` | 偏“服务型”的数据处理，例如遥控器、IMU、裁判系统、发射参数辅助 |
| Task | `User/Task` | ThreadX 线程任务，负责整车/飞镖业务逻辑 |
| Utils | `User/Utils` | 通用工具，例如 PID、数学函数、CRC、消息结构 |

核心运行方式：

1. `bsp_Init()` 初始化 UART、DWT、CAN。
2. 各个 ThreadX 任务启动，任务之间主要用 OneMessage 的 topic 发布/订阅数据。
3. 外设中断或 DMA 回调只做轻量处理，把数据放入缓存、更新反馈或释放信号量。
4. Task 层周期读取最新消息，计算控制量，再通过 Module/BSP 发给电机或上位机。

可以把它理解成：

```text
外设数据 -> BSP 回调 -> Module/Service 解包 -> OneMessage topic -> Task 业务逻辑 -> 电机/视觉/裁判系统输出
```

## 2. 主要数据链

### 遥控器/视觉/裁判系统到发射指令

`TaskSysCtrl.cpp` 是上层决策入口。

它订阅：

- `remoter`：遥控器数据
- `sensor`：传感器状态和左右副弦力
- `lch2sys`：发射流程反馈
- `referee`：裁判系统数据
- `visionrx`：视觉返回数据

它发布：

- `cmd`：发射动作命令，给 `TaskLauncher`
- `visiontx`：发给视觉的数据

主要逻辑是根据遥控器拨杆、视觉状态、裁判系统比赛状态、当前飞镖编号，决定当前动作：

- `DART_RELAX`：放松/急停
- `DART_PREPARE`：准备发射
- `DART_SYN_ADJUST`：调同步带
- `DART_STRING_ADJUST`：手动调副弦
- `DART_FIRE`：发射
- `DART_YAW_ADJUST`、`DART_PRE_TENSION` 等辅助动作

### 发射指令到电机控制

`TaskLauncher` 负责把 `cmd` 变成更底层的 `motorctrl`，`TaskMotor.cpp` 负责真正下发电机控制。

`TaskMotor.cpp` 订阅：

- `motorctrl`：目标速度、目标张力、扳机释放、同步带模式等
- `sensor`：左右副弦力反馈

它发布：

- `motorfdb`：龙门、同步带等反馈

副弦控制有两种方式：

- `string_able == true`：用力传感器闭环，PID 根据目标张力和实际张力算速度。
- `string_able == false`：直接用遥控器或上层给的速度开环控制。

最终左右副弦速度会通过 `ZDTStepper::X_V2_Vel_LC_Control()` 走 CAN3 发给 ZDT 步进电机。

### 传感器到闭环控制

`TaskSensor.cpp` 负责采集传感器并发布 `sensor` topic。

当前重点是 G4 力传感器：

```text
UART7 DMA 接收 -> HAL_UART_RxCpltCallback -> G4ForceGot 信号量
-> TaskSensor 解码 L/R 力值 -> 发布 sensor.string_L_force / sensor.string_R_force
-> TaskMotor 副弦 PID 使用
```

如果不用 G4，也保留了 USART2/USART3 Modbus 力传感器的旧逻辑。

## 3. BSP CAN

相关文件：

- `User/BSP/Inc/bsp_can.hpp`
- `User/BSP/Src/bsp_can.cpp`

### 初始化

`CAN_Init()` 初始化 `hfdcan1`、`hfdcan2`、`hfdcan3`：

- CAN1/CAN2 使用标准帧，主要给 DJI、达妙等电机。
- CAN3 使用扩展帧，当前挂 ZDT 步进电机。
- 三路 CAN 都打开 FIFO0 新消息中断和错误中断。

### 接收

所有 CAN 接收主要在 `HAL_FDCAN_RxFifo0Callback()` 里按 ID 分发：

- `hfdcan3`：先按扩展 ID 解析 ZDT 步进电机，取 `(Identifier >> 8) & 0xFF` 作为电机 ID，再更新左右副弦反馈。
- `0x201 ~ 0x208`：DJI 电机反馈，交给 `DJIMotorHandler`。
- `0x05 ~ 0x08`：达妙电机反馈，交给 `DMMotorHandler`。

CAN 回调里不建议加入太重的业务逻辑。现在它已经承担了 ID 分发，后续如果反馈类型继续变多，最好只缓存数据或释放信号量，再在线程里处理。

### 发送

普通 CAN 标准帧使用：

```cpp
CAN_Transmit(hcan, id, data, len);
```

ZDT 步进电机使用：

```cpp
can_SendCmd(hfdcan, cmd, len);
```

`can_SendCmd()` 会把一条 ZDT 命令拆成经典 CAN 扩展帧：

- 扩展 ID：`(cmd[0] << 8) | packNum`
- 数据第 0 字节：功能码 `cmd[1]`
- 后续字节：命令参数，每帧最多放 7 字节

所以 ZDT 的“设备 ID”和“功能码”不在普通 CAN 标准 ID 里，而是通过扩展帧 ID 和数据区一起表达。

## 4. BSP UART

相关文件：

- `User/BSP/Inc/bsp_usart.hpp`
- `User/BSP/Src/bsp_usart.cpp`

`USART_Init()` 主要配置 DMA 接收：

| 串口 | 当前用途 | 接收方式 |
| --- | --- | --- |
| USART1 | 裁判系统 | `HAL_UARTEx_ReceiveToIdle_DMA` |
| UART5 | DR16 遥控器 | `HAL_UARTEx_ReceiveToIdle_DMA` |
| UART7 | G4 力传感器 | `HAL_UART_Receive_DMA` |
| USART2/USART3 | 旧力传感器预留 | 当前主要逻辑在 `TaskSensor` |

接收链路：

- UART5 收到遥控器数据后，清 DCache，释放 `RemoterGot` 信号量，再重新开启 DMA。
- USART1 收到裁判系统数据后，清 DCache，推入 `referee_fifo` 环形缓冲区。
- UART7 收到 G4 力传感器数据后，清 DCache，释放 `G4ForceGot` 信号量，再重新开启 DMA。

注意 STM32H7 的 DMA 缓冲区要放在 DMA 能访问的 RAM 区域，并且涉及 Cache 时要做 `SCB_InvalidateDCache_by_Addr()`，否则会出现“中断来了但数据没变”的假象。

## 5. ZDT 步进电机

相关文件：

- `User/Module/ZDTStepper/X_V2.hpp`
- `User/Module/ZDTStepper/X_V2.cpp`
- `User/Task/Inc/config_motor.hpp`

这里有一个容易踩坑的点：

`User/Module/ZDTStepper/X_V2.cpp` 目前基本是注释掉的原厂例程，用来参考协议格式。当前工程真正使用的是 `config_motor.hpp` 里的 `ZDTStepper` 类。

### 当前工程里的 ZDTStepper

`TaskMotors::MotorsInit()` 里初始化了三个步进电机：

```cpp
stringMotorL.Init(&hfdcan3, 2, 0);
stringMotorR.Init(&hfdcan3, 1, 1);
yawMotor.Init(&hfdcan2, 1, 0);
```

其中：

- `id`：驱动器设备 ID。
- `positive_dir`：人为定义的正方向，用来统一上层正负速度和驱动板 0/1 方向。
- `hcan`：该电机使用哪一路 FDCAN。

`ParseSpeed()` 负责把带符号速度拆成：

```text
signed_speed -> dir + abs_speed
```

上层只需要关心速度正负，不需要每次手动判断驱动器方向位。

### 常用控制

当前主要使用速度限流模式：

```cpp
X_V2_Vel_LC_Control(id, dir, acc, vel, snF, maxCur);
```

参数含义：

- `id`：电机 ID
- `dir`：方向
- `acc`：加速度
- `vel`：速度，发送前会放大 10 倍
- `snF`：多机同步标志
- `maxCur`：最大电流

发送时命令格式大致是：

```text
[设备ID, 功能码, 方向, 加速度高/低, 速度高/低, 同步标志, 电流高/低, 0x6B]
```

然后交给 `can_SendCmd()` 拆成 CAN 扩展帧发送。

### 反馈解析

ZDT 反馈在 CAN3 接收回调里先按电机 ID 分给左右副弦：

```text
CAN3 RX -> motor_id = (Identifier >> 8) & 0xFF
-> stringMotorL/stringMotorR.updateFeedback()
```

`updateFeedback()` 再根据功能码更新：

- `0x27`：相电流/力矩相关反馈
- `0x35`：实时速度
- `0x36`：实时位置

目前 `TaskMotor` 调试区主要使用了左右副弦速度反馈。

## 6. 修改建议

后续接手时，建议按需求类型找入口：

| 想改什么 | 优先看哪里 |
| --- | --- |
| 改遥控器逻辑、自动瞄准触发条件 | `TaskSysCtrl.cpp` |
| 改发射状态机 | `TaskLauncher.cpp` |
| 改电机控制、PID、限幅 | `TaskMotor.cpp`、`config_motor.hpp` |
| 改力传感器 | `TaskSensor.cpp`、`bsp_usart.cpp` |
| 改 CAN ID 或电机反馈分发 | `bsp_can.cpp` |
| 改消息结构 | `Utils/Inc/magicmsgs.hpp` |
| 改 ZDT 步进电机命令 | `config_motor.hpp` 中的 `ZDTStepper` |

建议保持现有习惯：

- 中断回调里只做“收数据、分发、释放信号量”。
- 复杂逻辑放在线程里。
- 任务之间用 OneMessage topic 传结构体，不直接互相调用。
- 新增数据链时，先在 `magicmsgs.hpp` 里定义消息，再由生产者发布、消费者订阅。
- DMA 接收缓冲区注意 RAM 区域和 Cache。

## 7. 推荐阅读顺序

如果第一次接手这份代码，建议按这个顺序读：

1. `User/README.md`：先建立地图。
2. `User/Utils/Inc/magicmsgs.hpp`：看所有任务之间传什么数据。
3. `User/Task/Src/TaskSysCtrl.cpp`：看上层动作怎么决定。
4. `User/Task/Src/TaskLauncher.cpp`：看发射流程怎么展开。
5. `User/Task/Src/TaskMotor.cpp`：看电机控制怎么落地。
6. `User/BSP/Src/bsp_can.cpp` 和 `User/BSP/Src/bsp_usart.cpp`：看外设数据怎么进出。
7. `User/Task/Inc/config_motor.hpp`：看电机对象和 ZDTStepper 封装。

先读主线，再读细节，会轻松很多。
