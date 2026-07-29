#include "app_main.h"

#include "../Driver/cmd_port.h"
#include "../Driver/pwm_motor.h"
#include "../Utils/crc_util.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>
#include <new>
#include <string.h>

namespace
{
// ==================== Chassis calibration ====================
// These constants are the only parameters that normally need to be changed
// when the controller board is moved to another PWM differential chassis.
static const float kWheelTrackM = 0.32f;
static const float kWheelDiameterM = 0.10f;
static const float kMaxLinearSpeedMps = 0.35f;
static const float kMaxAngularSpeedRad = 2.0f;

static const float kMotorGearRatio = 30.0f;
static const uint32_t kMotorEncoderCountsPerRev = 52u;
static const float kControlPeriodS = 0.002f;

// The two motors are mounted as mirror images. A positive chassis velocity
// therefore needs opposite motor shaft directions.
static const float kLeftTargetSign = 1.0f;
static const float kRightTargetSign = -1.0f;

// Encoder direction relative to each port's positive PWM direction.
// Change an item to -1.0f if its measured speed has the opposite sign.
static const float kLeftEncoderSign = 1.0f;
static const float kRightEncoderSign = 1.0f;

static const uint32_t kCommandTimeoutMs = 200u;
static const float kPi = 3.14159265358979323846f;

class AppCmdPort : public cmd_port
{
public:
    explicit AppCmdPort(UART_HandleTypeDef *huart)
        : cmd_port(huart),
          state_(WaitHead0),
          data_index_(0u),
          received_crc_(0u),
          last_command_tick_(0u),
          has_valid_command_(false)
    {
        memset(data_, 0, sizeof(data_));
    }

    void handleReceiveData(uint8_t byte) override
    {
        // The legacy Driver parser deliberately bypasses CRC validation. This
        // application-side parser accepts only the fixed 11-byte velocity frame
        // and leaves BSP/Driver source files unchanged.
        switch (state_)
        {
        case WaitHead0:
            if (byte == CMD_FRAME_HEAD_0)
            {
                state_ = WaitHead1;
            }
            break;

        case WaitHead1:
            if (byte == CMD_FRAME_HEAD_1)
            {
                state_ = WaitId;
            }
            else
            {
                state_ = (byte == CMD_FRAME_HEAD_0) ? WaitHead1 : WaitHead0;
            }
            break;

        case WaitId:
            state_ = WaitLength;
            break;

        case WaitLength:
            if (byte == CMD_DATA_LENGTH)
            {
                data_index_ = 0u;
                state_ = WaitData;
            }
            else
            {
                state_ = WaitHead0;
            }
            break;

        case WaitData:
            data_[data_index_++] = byte;
            if (data_index_ >= CMD_DATA_LENGTH)
            {
                state_ = WaitCrcHigh;
            }
            break;

        case WaitCrcHigh:
            received_crc_ = static_cast<uint16_t>(byte) << 8;
            state_ = WaitCrcLow;
            break;

        case WaitCrcLow:
            received_crc_ |= byte;
            state_ = WaitEnd0;
            break;

        case WaitEnd0:
            state_ = (byte == CMD_FRAME_END_0) ? WaitEnd1 : WaitHead0;
            break;

        case WaitEnd1:
            if (byte == CMD_FRAME_END_1 &&
                CRC16_Table(data_, CMD_DATA_LENGTH) == received_crc_)
            {
                accept_command();
            }
            state_ = WaitHead0;
            break;

        default:
            state_ = WaitHead0;
            break;
        }
    }

    uint32_t last_command_tick() const
    {
        return last_command_tick_;
    }

    bool has_valid_command() const
    {
        return has_valid_command_;
    }

private:
    enum ParseState
    {
        WaitHead0,
        WaitHead1,
        WaitId,
        WaitLength,
        WaitData,
        WaitCrcHigh,
        WaitCrcLow,
        WaitEnd0,
        WaitEnd1
    };

    void accept_command()
    {
        float linear_speed = 0.0f;
        float angular_speed = 0.0f;
        memcpy(&linear_speed, &data_[0], sizeof(float));
        memcpy(&angular_speed, &data_[4], sizeof(float));

        cmd.linear_speed = linear_speed;
        cmd.angular_speed = angular_speed;
        // Bytes 8..10 are retained in the wire format for compatibility, but
        // this chassis-only application intentionally ignores them.
        cmd.left_seeder = false;
        cmd.right_seeder = false;
        cmd.mowing = false;

        last_command_tick_ = HAL_GetTick();
        has_valid_command_ = true;
    }

    ParseState state_;
    uint8_t data_[CMD_DATA_LENGTH];
    uint8_t data_index_;
    uint16_t received_crc_;
    volatile uint32_t last_command_tick_;
    volatile bool has_valid_command_;
};

alignas(AppCmdPort) static unsigned char g_cmd_port_storage[sizeof(AppCmdPort)];
alignas(PwmMotor) static unsigned char g_left_motor_storage[sizeof(PwmMotor)];
alignas(PwmMotor) static unsigned char g_right_motor_storage[sizeof(PwmMotor)];

static AppCmdPort *g_cmd_port = 0;
static PwmMotor *g_left_motor = 0;
static PwmMotor *g_right_motor = 0;

float clamp(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

PwmMotor::HardwareConfig make_motor_hardware_config(PwmEncBsp::MotorId motor_id,
                                                     float encoder_sign)
{
    PwmMotor::HardwareConfig config;
    config.motor_id = motor_id;
    config.gear_ratio = kMotorGearRatio;
    config.encoder_counts_per_rev = kMotorEncoderCountsPerRev;
    config.control_period_s = kControlPeriodS;
    config.direction_sign = encoder_sign;
    return config;
}

PwmMotor::SpeedPidConfig make_motor_pid_config()
{
    PwmMotor::SpeedPidConfig config;
    config.kp = 1200.0f;
    config.ki = 0.0f;
    config.kd = 5.0f;
    config.integral_limit = 3000.0f;
    config.output_limit = 65535.0f;
    config.deadzone = 0.5f;
    config.integral_separation_threshold = 50.0f;
    return config;
}

void stop_motors()
{
    if (g_left_motor != 0)
    {
        g_left_motor->control_speed(0.0f);
    }
    if (g_right_motor != 0)
    {
        g_right_motor->control_speed(0.0f);
    }
}

bool read_velocity_command(float &linear_speed_mps, float &angular_speed_rad)
{
    if (g_cmd_port == 0)
    {
        return false;
    }

    CmdData snapshot;
    uint32_t last_command_tick;
    bool has_valid_command;

    // UART5 updates CmdData in an interrupt. Take one coherent snapshot before
    // using its two float fields in the control calculation.
    taskENTER_CRITICAL();
    snapshot = g_cmd_port->cmd;
    last_command_tick = g_cmd_port->last_command_tick();
    has_valid_command = g_cmd_port->has_valid_command();
    taskEXIT_CRITICAL();

    const uint32_t now = HAL_GetTick();
    if (!has_valid_command || (now - last_command_tick) > kCommandTimeoutMs)
    {
        return false;
    }

    if (!isfinite(snapshot.linear_speed) || !isfinite(snapshot.angular_speed))
    {
        return false;
    }

    linear_speed_mps = clamp(snapshot.linear_speed, kMaxLinearSpeedMps);
    angular_speed_rad = clamp(snapshot.angular_speed, kMaxAngularSpeedRad);
    return true;
}

void apply_chassis_velocity(float linear_speed_mps, float angular_speed_rad)
{
    const float half_track = 0.5f * kWheelTrackM;
    const float left_linear_mps = linear_speed_mps - angular_speed_rad * half_track;
    const float right_linear_mps = linear_speed_mps + angular_speed_rad * half_track;
    const float rpm_per_mps = 60.0f / (kPi * kWheelDiameterM);

    const float left_target_rpm = left_linear_mps * rpm_per_mps * kLeftTargetSign;
    const float right_target_rpm = right_linear_mps * rpm_per_mps * kRightTargetSign;

    g_left_motor->control_speed(left_target_rpm);
    g_right_motor->control_speed(right_target_rpm);
}
} // namespace

extern "C" void App_ControlInit(void)
{
    const PwmMotor::SpeedPidConfig pid_config = make_motor_pid_config();

    const PwmMotor::HardwareConfig left_config =
        make_motor_hardware_config(PwmEncBsp::MOTOR_A, kLeftEncoderSign);
    const PwmMotor::HardwareConfig right_config =
        make_motor_hardware_config(PwmEncBsp::MOTOR_D, kRightEncoderSign);

    g_left_motor = new (g_left_motor_storage) PwmMotor(left_config, pid_config);
    g_right_motor = new (g_right_motor_storage) PwmMotor(right_config, pid_config);

    // Start from zero output before enabling command reception.
    stop_motors();

    g_cmd_port = new (g_cmd_port_storage) AppCmdPort(&huart5);
    g_cmd_port->startUartReceiveIT();
}

extern "C" void App_ControlStep(void)
{
    if (g_left_motor == 0 || g_right_motor == 0)
    {
        return;
    }

    float linear_speed_mps = 0.0f;
    float angular_speed_rad = 0.0f;
    if (!read_velocity_command(linear_speed_mps, angular_speed_rad))
    {
        stop_motors();
        return;
    }

    apply_chassis_velocity(linear_speed_mps, angular_speed_rad);
}
