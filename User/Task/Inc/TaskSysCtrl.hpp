
/**
 * @brief 储存飞镖参数
 * 
 */
struct Dart_Context_t
{
    int number;//飞镖编号
    float yaw_offset[10];//左右yaw轴的偏移offset
    float tension_toruqe;//需要的力

    int fired_count_this_open;  // 当前开门周期已发射的飞镖数
    bool shoot_enabled;         // 是否允许发射

};

/**
 * @brief 飞镖参数设置
 * 
 */
struct Dart_Config_t
{
    float yaw_offset;
    float tension_torque;
};

/**
 * @brief 飞镖相关信息
 * 可通过检索load_sequence查表查到相对的offset&tension_torque
 */
struct SysCtrl_Context_t
{
    int fired_count_this_open;  // 当前开门周期已发射的飞镖数
    int current_dart_index;//当前打到第几发飞镖
    int load_sequence[4];
    bool last_fire_status; //true表示上一次为fire状态

};