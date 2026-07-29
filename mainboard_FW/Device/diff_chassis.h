#ifndef DIFF_CHASSIS_H
#define DIFF_CHASSIS_H

#include <stdint.h>

#include "../Driver/M3508.h"

/**
 * @brief 差速底盘管理器类
 *
 * 该类用于管理基于两个 M3508 电机驱动的差速底盘，负责：
 * 1. 机械参数配置（轮距、轮径、限速、安装方向等）
 * 2. 运动学正逆解：将底盘的线速度(v)和角速度(w)映射到左右轮的 RPM
 * 3. 统一打包发送 CAN 控制指令（满足同一 CAN 总线多电机必须同帧打包的要求）
 */
class diff_chassis
{
public:
    /**
     * @brief 底盘机械及控制参数配置
     */
    struct MechanicalConfig
    {
        float wheel_track_m;         // 左右轮中心距（单位：米）
        float wheel_diameter_m;      // 轮子直径（单位：米）
        float max_linear_speed_mps;  // 线速度最大限制（单位：米/秒），<=0 表示不限制
        float max_angular_speed_rad; // 角速度最大限制（单位：弧度/秒），<=0 表示不限制
        float left_wheel_scale;      // 左轮标定系数（用于补偿左右轮实际直径差异，默认1.0）
        float right_wheel_scale;     // 右轮标定系数（用于补偿左右轮实际直径差异，默认1.0）
        bool left_reversed;          // 左轮安装方向是否反向（根据实际电机安装朝向设置）
        bool right_reversed;         // 右轮安装方向是否反向（根据实际电机安装朝向设置）

        MechanicalConfig();
    };

    /**
     * @brief 左右轮目标转速结构体
     */
    struct WheelTarget
    {
        float left_rpm;              // 左轮目标转速（单位：转/分钟）
        float right_rpm;             // 右轮目标转速（单位：转/分钟）

        WheelTarget();
    };

    /**
     * @brief 构造函数 1：外部传入已实例化的电机指针（依赖注入）
     * 
     * @param left_motor 左电机实例指针
     * @param right_motor 右电机实例指针
     * @param cfg 机械配置参数
     */
    diff_chassis(M3508 *left_motor, M3508 *right_motor, const MechanicalConfig &cfg);

    /**
     * @brief 构造函数 2：指定电机 ID，由底盘类内部管理电机生命周期
     * 
     * @param left_motor_id 左电机 CAN ID (1~8)
     * @param right_motor_id 右电机 CAN ID (1~8)
     * @param cfg 机械配置参数
     */
    diff_chassis(uint8_t left_motor_id, uint8_t right_motor_id, const MechanicalConfig &cfg);
    
    /**
     * @brief 析构函数（自动释放内部创建的电机实例）
     */
    ~diff_chassis();

    /**
     * @brief 设置底盘目标运动速度并发送控制指令
     * 
     * 该方法会进行运动学解算，并将计算后的电流指令通过 CAN 统一打包下发。
     * @param linear_speed_mps 目标线速度（单位：米/秒）
     * @param angular_speed_rad 目标角速度（单位：弧度/秒）
     * @return true 控制指令发送成功
     * @return false 控制指令发送失败或电机指针为空
     */
    bool set_cmd_vel(float linear_speed_mps, float angular_speed_rad);

    /**
     * @brief 紧急停止底盘运动
     */
    void stop();

    /**
     * @brief 根据运动学模型计算左右轮目标转速（不发送指令）
     * 
     * @param linear_speed_mps 目标线速度
     * @param angular_speed_rad 目标角速度
     * @return WheelTarget 左右轮的预期 RPM
     */
    WheelTarget calc_wheel_target_rpm(float linear_speed_mps, float angular_speed_rad) const;

    /**
     * @brief 动态更新机械配置参数
     */
    void set_mechanical_config(const MechanicalConfig &cfg);
    
    /**
     * @brief 获取当前机械配置参数
     */
    const MechanicalConfig &get_mechanical_config() const;

    // 暴露电机指针供外部访问（如读取反馈状态或调试）
    M3508 *left_motor();
    M3508 *right_motor();
    const M3508 *left_motor() const;
    const M3508 *right_motor() const;

private:
    // 禁用拷贝构造和赋值操作
    diff_chassis(const diff_chassis &);
    diff_chassis &operator=(const diff_chassis &);

    MechanicalConfig cfg_;       // 当前的机械配置
    M3508 *left_motor_;          // 左电机指针
    M3508 *right_motor_;         // 右电机指针
    bool own_motors_;            // 标记电机实例是否由本类自行 new 出来的

    /**
     * @brief 对数值进行绝对值限幅
     * @param value 原始数值
     * @param max_abs_limit 绝对值上限
     * @return float 限幅后的数值
     */
    float clamp_if_enabled(float value, float max_abs_limit) const;

    /**
     * @brief 将轮子表面线速度转换为转速（RPM）
     * @param wheel_linear_speed_mps 轮子线速度（米/秒）
     * @param wheel_diameter_m 轮子直径（米）
     * @return float 对应的转速（转/分钟）
     */
    static float linear_speed_to_rpm(float wheel_linear_speed_mps, float wheel_diameter_m);

    /**
     * @brief 将左右电机统一设置为 Component 模式，防止其在外部单独发包
     */
    void set_motors_component_mode();

    /**
     * @brief 按照 CAN 报文格式打包并发送 4 个电机的电流控制指令
     * @param std_id CAN 标准帧 ID（0x200 或 0x1FF）
     * @param c1 电机 1/5 电流指令
     * @param c2 电机 2/6 电流指令
     * @param c3 电机 3/7 电流指令
     * @param c4 电机 4/8 电流指令
     * @return true 发送成功
     */
    bool send_group_command(uint32_t std_id, int16_t c1, int16_t c2, int16_t c3, int16_t c4) const;

    /**
     * @brief 将左右轮控制指令合并打包后发送
     * @param left_command 左轮电流控制值
     * @param right_command 右轮电流控制值
     * @return true 发送成功
     */
    bool send_dual_motor_command(int16_t left_command, int16_t right_command) const;
};

#endif
