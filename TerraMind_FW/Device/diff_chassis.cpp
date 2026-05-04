#include "diff_chassis.h"

#include "../BSP/can_bsp.h"

namespace
{
// 内部使用的圆周率与极小正数（用于防止除零）
static const float kPi = 3.14159265358979323846f;
static const float kMinPositive = 1.0e-6f;
}

// 默认机械参数初始化
diff_chassis::MechanicalConfig::MechanicalConfig()
    : wheel_track_m(0.30f),
      wheel_diameter_m(0.10f),
      max_linear_speed_mps(0.0f),
      max_angular_speed_rad(0.0f),
      left_wheel_scale(1.0f),
      right_wheel_scale(1.0f),
      left_reversed(false),
      right_reversed(false)
{
}

// 目标转速结构体默认初始化
diff_chassis::WheelTarget::WheelTarget() : left_rpm(0.0f), right_rpm(0.0f)
{
}

// 构造函数 1：外部传入实例
diff_chassis::diff_chassis(M3508 *left_motor, M3508 *right_motor, const MechanicalConfig &cfg)
    : cfg_(cfg),
      left_motor_(left_motor),
      right_motor_(right_motor),
      own_motors_(false) // 外部实例，不负责销毁
{
    // 将电机控制模式强制设为 Component，由底盘类接管 CAN 发送
    set_motors_component_mode();
}

// 构造函数 2：内部动态分配电机
diff_chassis::diff_chassis(uint8_t left_motor_id, uint8_t right_motor_id, const MechanicalConfig &cfg)
    : cfg_(cfg),
      left_motor_(0),
      right_motor_(0),
      own_motors_(true) // 标记为内部实例，析构时需要 delete
{
    // 创建时直接指定为 Component 模式
    left_motor_ = new M3508(left_motor_id, M3508::ControlMode::Component);
    right_motor_ = new M3508(right_motor_id, M3508::ControlMode::Component);
    set_motors_component_mode();
}

// 析构函数
diff_chassis::~diff_chassis()
{
    // 如果是内部创建的电机，需要释放内存
    if (!own_motors_)
    {
        return;
    }
    if (left_motor_ != 0)
    {
        delete left_motor_;
        left_motor_ = 0;
    }
    if (right_motor_ != 0)
    {
        delete right_motor_;
        right_motor_ = 0;
    }
}

// 核心控制接口：输入线速度和角速度，计算并发送电机指令
bool diff_chassis::set_cmd_vel(float linear_speed_mps, float angular_speed_rad)
{
    if (left_motor_ == 0 || right_motor_ == 0)
    {
        return false;
    }

    // 1. 运动学解算：获得左右轮目标 RPM
    const WheelTarget target = calc_wheel_target_rpm(linear_speed_mps, angular_speed_rad);
    
    // 2. 更新电机目标转速
    left_motor_->set_target_rpm(target.left_rpm);
    right_motor_->set_target_rpm(target.right_rpm);

    // 3. 执行 PID 迭代，计算当前控制周期的电流指令（控制量）
    const int16_t left_command = left_motor_->speed_control_step();
    const int16_t right_command = right_motor_->speed_control_step();
    
    // 4. 将左右电机指令打包在同一个/组 CAN 帧中发送
    return send_dual_motor_command(left_command, right_command);
}

// 停止底盘运动
void diff_chassis::stop()
{
    (void)set_cmd_vel(0.0f, 0.0f);
}

// 运动学逆解：由底盘 v/w 解算左右轮目标转速
diff_chassis::WheelTarget diff_chassis::calc_wheel_target_rpm(float linear_speed_mps, float angular_speed_rad) const
{
    WheelTarget out;

    // 对整体线速度和角速度进行限幅
    const float v = clamp_if_enabled(linear_speed_mps, cfg_.max_linear_speed_mps);
    const float w = clamp_if_enabled(angular_speed_rad, cfg_.max_angular_speed_rad);
    
    // 防止除零或非法的极小轮距
    const float half_track = 0.5f * ((cfg_.wheel_track_m > kMinPositive) ? cfg_.wheel_track_m : kMinPositive);

    // 差速模型：
    // 左轮线速度 = v - w * (轮距 / 2)
    // 右轮线速度 = v + w * (轮距 / 2)
    float left_linear = v - half_track * w;
    float right_linear = v + half_track * w;

    // 乘以标定系数（应对两轮实际直径不一致导致的跑偏）
    left_linear *= cfg_.left_wheel_scale;
    right_linear *= cfg_.right_wheel_scale;

    // 如果电机安装反向，则反转控制方向
    if (cfg_.left_reversed)
    {
        left_linear = -left_linear;
    }
    if (cfg_.right_reversed)
    {
        right_linear = -right_linear;
    }

    // 获取合法轮径，防止除零
    const float wheel_diameter = (cfg_.wheel_diameter_m > kMinPositive) ? cfg_.wheel_diameter_m : kMinPositive;
    
    // 将轮子线速度转换为 RPM
    out.left_rpm = linear_speed_to_rpm(left_linear, wheel_diameter);
    out.right_rpm = linear_speed_to_rpm(right_linear, wheel_diameter);

    return out;
}

// 设置机械配置
void diff_chassis::set_mechanical_config(const MechanicalConfig &cfg)
{
    cfg_ = cfg;
}

// 获取当前机械配置
const diff_chassis::MechanicalConfig &diff_chassis::get_mechanical_config() const
{
    return cfg_;
}

// 获取左电机实例指针
M3508 *diff_chassis::left_motor()
{
    return left_motor_;
}

// 获取右电机实例指针
M3508 *diff_chassis::right_motor()
{
    return right_motor_;
}

const M3508 *diff_chassis::left_motor() const
{
    return left_motor_;
}

const M3508 *diff_chassis::right_motor() const
{
    return right_motor_;
}

// 辅助函数：限制绝对值大小
float diff_chassis::clamp_if_enabled(float value, float max_abs_limit) const
{
    // 如果限幅值 <= 0，认为不限幅
    if (max_abs_limit <= 0.0f)
    {
        return value;
    }
    if (value > max_abs_limit)
    {
        return max_abs_limit;
    }
    if (value < -max_abs_limit)
    {
        return -max_abs_limit;
    }
    return value;
}

// 辅助函数：线速度转 RPM
float diff_chassis::linear_speed_to_rpm(float wheel_linear_speed_mps, float wheel_diameter_m)
{
    // 转速(转/分) = (线速度(米/秒) / 轮子周长(米)) * 60(秒)
    return (wheel_linear_speed_mps / (kPi * wheel_diameter_m)) * 60.0f;
}

// 强制设置电机为 Component 模式（不自行发包）
void diff_chassis::set_motors_component_mode()
{
    if (left_motor_ != 0)
    {
        left_motor_->set_control_mode(M3508::ControlMode::Component);
    }
    if (right_motor_ != 0)
    {
        right_motor_->set_control_mode(M3508::ControlMode::Component);
    }
}

// 发送单组 CAN 控制报文（0x200 或 0x1FF，包含四个电机槽位）
bool diff_chassis::send_group_command(uint32_t std_id, int16_t c1, int16_t c2, int16_t c3, int16_t c4) const
{
    uint8_t tx_data[8] = {0};
    M3508::pack_can_command(c1, c2, c3, c4, tx_data); // 打包电流值到 8 字节

    CanBsp::TxHeader tx_header;
    tx_header.id = std_id;
    tx_header.is_extended_id = false;
    tx_header.is_remote_frame = false;
    tx_header.dlc = 8u;
    tx_header.transmit_global_time = false;
    
    return CAN_BUS.send_raw(tx_header, tx_data); // 通过底层 CAN BSP 发送
}

// 将左右电机的指令按其 CAN ID 进行分类打包发送
bool diff_chassis::send_dual_motor_command(int16_t left_command, int16_t right_command) const
{
    int16_t group_200[4] = {0, 0, 0, 0}; // 对应 ID 1~4 的控制槽位
    int16_t group_1ff[4] = {0, 0, 0, 0}; // 对应 ID 5~8 的控制槽位
    bool has_group_200 = false;
    bool has_group_1ff = false;

    // 处理左电机指令
    const uint8_t left_id = left_motor_->get_motor_id();
    if (left_id <= 4u)
    {
        group_200[left_id - 1u] = left_command; // ID 1 对应数组下标 0
        has_group_200 = true;
    }
    else
    {
        group_1ff[left_id - 5u] = left_command; // ID 5 对应数组下标 0
        has_group_1ff = true;
    }

    // 处理右电机指令
    const uint8_t right_id = right_motor_->get_motor_id();
    if (right_id <= 4u)
    {
        group_200[right_id - 1u] = right_command;
        has_group_200 = true;
    }
    else
    {
        group_1ff[right_id - 5u] = right_command;
        has_group_1ff = true;
    }

    bool ok = true;
    // 如果有 1~4 号电机的指令，则发送 0x200
    if (has_group_200)
    {
        ok = send_group_command(0x200u, group_200[0], group_200[1], group_200[2], group_200[3]) && ok;
    }
    // 如果有 5~8 号电机的指令，则发送 0x1FF
    if (has_group_1ff)
    {
        ok = send_group_command(0x1FFu, group_1ff[0], group_1ff[1], group_1ff[2], group_1ff[3]) && ok;
    }
    
    return ok;
}
