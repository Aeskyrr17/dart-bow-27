#ifndef DISPLACEMENT_SENSOR_HPP
#define DISPLACEMENT_SENSOR_HPP

#include <stdint.h>
#include "fdcan.h"

/* ===================================================================================
 * BRT 拉绳位移传感器（CANbus 版）应用层协议
 *
 * 报文格式（标准帧，CAN ID = 节点地址，出厂 0x01，出厂波特率 500kbps）：
 *     [LEN][DEVICE_ID][FUNC][DATA 0~4 字节]
 *
 * 四条最容易踩的坑，写代码前先看一眼：
 *   1. **LEN ≠ DLC**。LEN 是数据域首字节，表示“应用层长度”（含它自己）；DLC 是帧的
 *      实际字节数。本协议下两者通常相等（4/5/7），但校验时必须查 LEN，不能只看 DLC。
 *   2. **传感器不主动广播**（模式=0x00 查询时）。必须主机周期性下发读指令，
 *      不发就一帧都没有 —— 这是“接上了但没数据”最常见的原因。
 *   3. **多字节默认低字节在前**，但 0x22 / 0x0B / 0x0D 三条是大端（见下表注释）。
 *   4. **应答要按 FUNC 分流解析**：读位置(0x01)/读速度(0x0A) 的 LEN=07；
 *      设置类(0x02/0x03/0x06/0x07...) 的应答 LEN=04，第 4 字节是状态码（0=成功）。
 *      统一按 LEN=07 校验会把设置类应答全部当错误帧丢掉。
 * =================================================================================== */

/**
 * @brief 指令码（FUNC）。本工程只保留常用指令，其余在注释里留档。
 * @note  每条指令的请求/应答长度、端序都不同，见各条注释。
 */
enum class InstructionType : uint8_t
{
    /* ---------------- 常用：读写位置/速度 ---------------- */
    ReadPosition = 0x01,  ///< 读位置。请求 04 01 01 00；应答 LEN=7，DATA 为 4 字节**小端** u32
    ReadVelocity = 0x0A,  ///< 读速度。请求 04 01 0A 00；应答 LEN=7，DATA 为 4 字节**小端** i32

    /* ---------------- 常用：标定与回传方式 ---------------- */
    SetMode = 0x04,           ///< 设查询/回传模式。DATA：00=查询；AA=自动回传位置；02=自动回传速度
    SetReportPeriod = 0x05,   ///< 设自动回传周期。DATA 为**小端** u16，单位 us（50~65535）
    SetZero = 0x06,           ///< 当前位置设为零。DATA=00；应答 LEN=4，状态 0=成功
    SetDirection = 0x07,      ///< 设递增方向。DATA：00=顺时针；01=逆时针
    SetBaudRate = 0x03,       ///< 设波特率。DATA：00=500K 01=1M 02=250K 03=125K 04=100K；改完主机也要跟着改
    SetVelocitySampleTime = 0x0B, ///< 设速度采样时间。DATA 为**大端** u16，单位 ms（1~65535）
    SetCurrentNum = 0x0D,        ///< 设当前位置计数。DATA 为**大端** u32，

    /* ---------------- 以下本工程暂不使用，留档备查 ----------------
     *   0x02 Set standard node ID; the acknowledgement uses the new address.
     *        Commit the new address only after receiving the acknowledgement.
     *   0x22 设扩展地址（ID > 255）。请求 LEN=07、地址**大端** u32；应答为**扩展帧**，
     *        32 位状态**大端**，LEN=07。（说明书这一条的长度/端序写错过，以此为准）
     *   0x0C 当前位置设为中点。DATA=01
     *   0x0F 当前位置设为五圈值。DATA=01
     */
};

/** @brief 回传模式取值（配合 InstructionType::SetMode 使用） */
enum class SensorMode : uint8_t
{
    Query = 0x00,        ///< 查询模式（默认）：必须主机轮询
    AutoPosition = 0xAA, ///< 自动回传位置
    AutoVelocity = 0x02, ///< 自动回传速度
};

/** @brief 波特率指令的取值（配合 InstructionType::SetBaudRate 使用） */
enum class SensorBaudCode : uint8_t
{
    Kbps500 = 0x00, ///< 出厂默认
    Mbps1 = 0x01,
    Kbps250 = 0x02,
    Kbps125 = 0x03,
    Kbps100 = 0x04,
};

/** @brief 递增方向取值（配合 InstructionType::SetDirection 使用） */
enum class SensorDirection : uint8_t
{
    Clockwise = 0x00,
    CounterClockwise = 0x01,
};

/**
 * @class DisplacementSensor
 * @brief 拉绳位移传感器模块：协议解析 + 轮询节拍 + 在线状态维护。
 *
 * 分工：
 *   - OnFrame() 由 bsp 的 HAL_CAN_RxFifo0MsgPendingCallback（中断上下文）调用，只做校验、存原始值和打时间戳；
 *   - Update() 由线程每拍调用，做浮点换算（工程量）和在线判定；
 *   - Poll() 由线程按 poll_period_ms 调用，下发读位置指令。
 * 这样中断里不碰浮点，线程里读到的 position_mm 永远是“已经换算好”的。
 *
 */
class DisplacementSensor
{
public:
    /* ---------------- 帧布局与长度常量（避免 .cpp 里散落魔数） ---------------- */
    static constexpr uint8_t kIdxLen = 0;   ///< 数据域第 0 字节：应用层长度
    static constexpr uint8_t kIdxAddr = 1;  ///< 数据域第 1 字节：节点地址
    static constexpr uint8_t kIdxFunc = 2;  ///< 数据域第 2 字节：指令码
    static constexpr uint8_t kIdxData = 3;  ///< 数据域第 3 字节：DATA 起点（低字节在前）

    static constexpr uint8_t kReqLen = 4;      ///< 请求帧的 LEN 与 DLC（读位置/读速度/设置类都是 4）  注意：设置采样/回传时间的帧长度为5
    static constexpr uint8_t kRespLenPos = 7;  ///< 0x01 / 0x0A 应答：LEN + addr + func + 4B 数据
    static constexpr uint8_t kRespLenAck = 4;  ///< 设置类应答：LEN + addr + func + status
    static constexpr uint16_t kMaxDlc = 8;     ///< 经典 CAN 单帧上限

    /** @brief 在线状态 */
    enum SensorStateTypeDef
    {
        SENSOR_OFFLINE = 0,
        SENSOR_ONLINE = 1
    };

    /** @brief 静态配置：上电时给一次，运行期不再改 */
    struct Config
    {
        FDCAN_HandleTypeDef *hcan = &hfdcan2;  ///< 挂在哪条 CAN 上（换 CAN2 时只改这里）
        uint8_t node_id = 1;               ///< 节点地址：CAN ID 与 data[1] 都用它
        uint16_t poll_period_ms = 30;      ///< 轮询周期：查询模式下必须自己发读指令（20~50ms）
        uint16_t timeout_ms = 100;         ///< 超过这么久没应答判离线（≈3 个周期）
        float resolution_mm_per_count = 100.0/4096.0f;  ///< mm/count，见上方说明
        int8_t direction = 1;                    ///< 拉出为 +1；装反了改 -1，不用动代码
        uint32_t zero_count = 1000U;             ///< 零点基准计数
    };

    /* ---------------- 配置 ---------------- */
    Config cfg;

    /* ---------------- 通信状态（中断写、线程读 → 全部 volatile） ---------------- */
    volatile uint32_t raw_count = 0U;     ///< 最近一帧的原始位置计数（32 位多圈绝对值，**正整数**）
    volatile int32_t raw_velocity = 0U;   ///< 最近一帧的原始速度（0x0A 指令才有意义，带符号）
    volatile uint32_t stamp_ms = 0U;      ///< 最近一帧**有效**应答的 tx_time_get()（1 tick = 1ms）
    volatile uint32_t rx_count = 0U;      ///< 累计有效帧数
    volatile uint32_t pos_rx_count = 0U;  ///< 累计收到的位置帧的数量
    volatile uint32_t vel_rx_count = 0U; ///< 累计收到的速度帧的数量
    volatile uint32_t err_count = 0U;     ///< 帧格式/地址/指令码不符而被丢弃的帧数（诊断用）
    volatile uint32_t miss_count = 0U;    ///< 连续“没等到新应答”的拍数（在线判定去抖）
    volatile uint8_t last_status = 0xFFU; ///< 设置类指令的应答状态码；0=成功，0xFF=还没收到过
    volatile uint8_t last_ack_func = 0xFFU; ///< 设置类指令的应答指令码；0xFF=还没收到过
    volatile SensorStateTypeDef state = SENSOR_OFFLINE; ///< 在线状态（由 AliveCheck 维护）
    volatile uint16_t velocity_sample_time = 1000U; ///< 0x0B 速度采样时间，单位 ms（1~65535，默认 1000ms）

    /* ---------------- 工程量（只在线程里算，不需要 volatile） ---------------- */
    float position_mm = 0.0f;    ///< 位移，mm。由 raw_count 每拍重算（不做累加，避免漂移）
    float velocity_mm_s = 0.0f;  ///< 速度，mm/s。使用 0x0A 原始速度
    float velocity_diff_mm_s = 0.0f;  ///< 速度，mm/s。由相邻帧差分算出

    // CAN node ID 1 or 2 selects an independent sensor instance.
    static DisplacementSensor *Instance(uint8_t node_id);

    /**
     * @brief 记录配置并重置全部状态（不碰总线，可在 bsp_Init 之后调用）。
     * @note  实现时必须把 raw_count / raw_velocity / stamp_ms / rx_count / err_count /
     *        miss_count / last_status / state / position_mm / velocity_mm_s 以及
     *        prev_* 差分量全部清零，否则上电后 Position() 会拿脏值算出天文数字。
     * @retval true 配置合法（node_id!=0、分辨率>0、timeout_ms>=poll_period_ms）
     */
    bool Init(const Config &config);

    /**
     * @brief 同上，等价于 Init(Config{})（全用默认值：CAN1 / 地址 1 / 30ms / 0.098mm）。
     * @note  不能直接写成 `Init(const Config &config = Config())`：
     *        Config 的默认成员初值要等外围类闭合后才生效，类内默认实参会编译报错。
     */
    bool Init();

    /**
     * @brief 下发一次“读位置”指令（04 id 01 00）。由线程按 poll_period_ms 调用。
     * @note  发送失败（邮箱满/总线错误）会计入 miss_count，不会阻塞。
     *        组帧请用 kReqLen / kIdxFunc / static_cast<uint8_t>(InstructionType::ReadPosition)，
     *        不要硬编码 0x04 / 0x01。
     */
    void Poll();

    /**
     * @brief 下发一次“读速度”指令（04 id 0A 00）。
     * @warning 与 Poll() 共用同一条应答通道，**不要每拍都发**：建议 4~5 拍位置夹杂 1 拍速度，
     *          或把 poll_period_ms 拉长到 100ms 再交替。发太快会把位置刷新率拖垮。
     */
    void PollVelocity();

    /**
     * @brief 通用指令发送通道：组一帧 [len][addr][func][param...] 发出去。
     * @param func      指令码
     * @param param     DATA 内容（可为 nullptr，表示只发 1 字节 0x00 占位）
     * @param param_len DATA 字节数（0~4；总长不得超过 8）
     * @retval true 已成功装入发送邮箱（**不代表传感器已执行**，执行结果看 last_status）
     * @note  设置类指令（0x02/0x03/0x06/0x07）建议都走这里，下一行就能加一条新指令，
     *        不必为十几条指令各写一个方法。
     */
    bool SendInstruction(InstructionType func, const uint8_t *param = nullptr, uint8_t param_len = 0);

    /**
     * @brief 设置回传模式。切到 AutoPosition/AutoVelocity 后**必须停止 Poll()**，
     *        否则轮询与自动回传会撞车。
     * @retval true 已发出（切换结果看 last_status）
     */
    bool SetMode(SensorMode mode);

    /**
     * @brief 设置递增方向。注意 0x07 是**传感器侧**方向，配置里的 cfg.direction 是**软件侧**
     *        符号，两者任选其一即可，不要同时取反（会互相抵消）。
     */
    bool SetDirection(SensorDirection dir);

    /**
     * @brief 修改编码器自动回传的时间
     * @param new_time 新的自动回传时间（单位us）
     * @return true 已发出（切换结果看 last_status）
     */
    bool SetReportPeriod(uint16_t new_time);

    /**
     * @brief 改速度采样时间（0x0B 指令）。
     * @param new_time 1~65535 ms，在Config 中默认设置成 1000ms（可修改）。**注意单位是 ms，不是 s**。
     * @retval true 已发出（切换结果看 last_status）
     */
    bool SetVelocitySampleTime(uint16_t new_time);

    /**
     * @brief 改波特率。
     * @warning 改完必须同步改 CubeMX 的 CAN 位时序（Prescaler/BS1/BS2）并重新生成，
     *          否则当场失联。**建议最后再做，且先准备好恢复流程。**
     */
    bool SetBaudRate(SensorBaudCode code);

    /**
     * @brief 改当前位置计数（0x0D 指令）。
     * @param new_count 新的计数值（**大端** u32）
     * @retval true 已发出（切换结果看 last_status）
     */
    bool SetCurrentNum(uint32_t new_count);

    /**
     * @brief 处理一帧收到的报文。**在中断上下文调用**，只做校验与存值。
     * @param data 报文数据（指向中断栈上的临时缓冲，只在回调内有效）
     * @param len  DLC（0~8）
     * @note  校验顺序：DLC → LEN → 地址 → FUNC，任何一步不符计入 err_count 后返回。
     *        按 FUNC 分流：0x01/0x0A 期望 len>=7，设置类期望 len>=4（状态码写 last_status）。
     *        帧有效时更新 raw_count/raw_velocity、stamp_ms=tx_time_get()、rx_count++、miss_count=0。
     *        **不要在中断里做浮点运算**，也不要 printf / 加会阻塞的锁。
     */
    void OnFrame(const uint8_t *data, uint16_t len);

    /**
     * @brief 每拍调用（线程）：更新 position_mm / velocity_mm_s / velocity_diff_mm_s，并维护在线状态。
     * @note  先用 rx_count 是否变化判断“这一拍有没有新帧”，没新帧就别算差分
     *        （否则速度会被算成 0 来回抖）。差分用 prev_raw_count_/prev_stamp_ms_。
     */
    void Update();

    /**
     * @brief 当前在线状态（会先按 timeout_ms 判定，不会用到过期的 state）。
     * @note  `stamp_ms == 0` 表示从未收到过有效帧，应直接判离线，
     *        否则上电头 100ms 会误报在线。时间基准用 tx_time_get()（1 tick = 1ms）。
     */
    SensorStateTypeDef AliveCheck();

    /**
     * @brief 当前位移（mm）。直接由 raw_count 现算，不依赖 Update() 是否跑过。
     * @warning 必须走**有符号**差值：
     *          `(int32_t)raw_count - (int32_t)cfg.zero_count`。
     *          直接对两个 uint32 相减，当 raw_count < zero_count（上电默认 0 < 1000）时会下溢成
     *          0xFFFFFC18 ≈ 42.9 亿，转成 float 是个巨大的**正**数，而不是负位移。
     *          同时别忘了乘 cfg.direction。
     */
    float Position() const;

    /**
     *  @brief 硬件层面上把当前位置当成零点，断电仍保持
     *  @note
     */
    void SetZeroHard();

    /**
     * @brief 把当前位置记为软零点：zero_count = raw_count（只写 RAM，掉电即失效）。
     * @note  软零点不动传感器内部计数，所以多圈绝对值不会丢；需要掉电保持就把
     *        zero_count 存 Flash，或改用 0x06 指令做硬件置零（后者要同步把本地
     *        zero_count/raw_count 一起清 0，否则位置会跳变）。
     */
    void SetZeroSoft();

private:
    DisplacementSensor() = default;

    /* ---------------- 差分用（只在 Update() 里读写，线程上下文） ---------------- */
    uint32_t prev_raw_count_ = 0U;   ///< 上一帧原始计数
    uint32_t prev_stamp_ms_ = 0U;    ///< 上一帧时间戳
    uint32_t prev_rx_count_ = 0U;     ///< 上一帧有效帧计数
    uint32_t prev_pos_rx_count_ = 0U; ///< 上一帧位置帧计数
    uint32_t prev_vel_rx_count_ = 0U; ///< 上一帧速度帧计数
    float prev_position_mm_ = 0.0f;  ///< 上一帧位移
};

#endif // DISPLACEMENT_SENSOR_HPP
