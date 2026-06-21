# PWM Servo Driver Design

## Background

This project needs a PWM servo driver for the current TerraMind firmware.
The user provided an existing reference implementation in another project:
`GC_Race/USER/Application/servo.h` and `servo.cpp`.

The new implementation should:

- Follow the structure style of the current project, especially `pwm_motor`.
- Keep the servo angle mapping behavior consistent with the reference implementation.
- Use fixed port mapping for servo A/B/C/D.

## Goals

- Add a servo driver with the same overall layering style as the current project.
- Keep the angle-to-PWM compare conversion aligned with the reference code.
- Make the hardware mapping explicit and centralized.
- Keep the change scope small and avoid modifying existing motor code.

## Non-Goals

- No encoder feedback or closed-loop control.
- No dynamic port remapping at runtime.
- No refactor of the existing `pwm_motor` or BSP modules.
- No new FreeRTOS task or test task in this step unless later requested during implementation.

## Confirmed Hardware Mapping

- `SERVO_A -> TIM12_CH2`
- `SERVO_B -> TIM12_CH1`
- `SERVO_C -> TIM8_CH4`
- `SERVO_D -> TIM8_CH3`

## Chosen Architecture

The implementation uses the same high-level style as `pwm_motor`:

- A BSP layer handles fixed hardware mapping and low-level PWM output.
- A Driver layer handles servo semantics such as angle limit, compare conversion, and reset behavior.

Planned files:

- `TerraMind_FW/BSP/pwm_servo_bsp.h`
- `TerraMind_FW/BSP/pwm_servo_bsp.cpp`
- `TerraMind_FW/Driver/pwm_servo.h`
- `TerraMind_FW/Driver/pwm_servo.cpp`

The new files will also be added into `MDK-ARM/TerraMind.uvprojx`.

## BSP Design

`PwmServoBsp` is responsible only for hardware binding and compare output.

### Responsibilities

- Bind a fixed `ServoId` to one timer handle and one timer channel.
- Start the PWM output for that channel.
- Write compare values to the timer channel.
- Clamp compare values into a safe valid range based on timer ARR.

### Interface Sketch

`PwmServoBsp` will contain:

- `enum ServoId { SERVO_A, SERVO_B, SERVO_C, SERVO_D }`
- `explicit PwmServoBsp(ServoId servo_id)`
- `void set_compare(uint32_t compare_value)`
- `bool is_valid() const`
- private mapping and startup helpers

### Mapping Rules

- `SERVO_A` binds to `htim12` + `TIM_CHANNEL_2`
- `SERVO_B` binds to `htim12` + `TIM_CHANNEL_1`
- `SERVO_C` binds to `htim8` + `TIM_CHANNEL_4`
- `SERVO_D` binds to `htim8` + `TIM_CHANNEL_3`

## Driver Design

`PwmServo` is responsible for angle semantics and reference-compatible conversion.

### Responsibilities

- Store the servo hardware configuration.
- Accept a target angle from the caller.
- Limit the input angle to the valid range.
- Convert the limited angle to compare value using the reference behavior.
- Support resetting the servo to center.
- Provide the latest commanded angle for inspection.

### Interface Sketch

`PwmServo` will contain:

- `struct HardwareConfig`
- `PwmServo(const HardwareConfig &hardware_config)`
- `bool set_angle(float angle_deg)`
- `bool reset()`
- `float get_current_angle() const`

### HardwareConfig Fields

The config keeps the driver reusable while preserving the current style:

- `PwmServoBsp::ServoId servo_id`
- `float max_angle_deg`
- `float center_compare`
- `float compare_delta`

Default behavior will match the provided reference:

- `center_compare = 1825.0f`
- `compare_delta = 115.0f`

To preserve the original semantics, when the caller passes `max_angle_deg = 180.0f`,
the internal half-range used by the conversion logic will be `90.0f`, matching the
reference code where the constructor stores `max_angle / 2`.

## Conversion Logic

The new code keeps the same linear mapping pattern as the reference implementation.

Reference behavior:

- Constructor stores half-range from input max angle.
- `set_angle(angle)` computes compare value from center minus a linear delta.
- `reset()` writes the fixed center compare value.

Target formula:

```cpp
effective_half_angle = max_angle_deg / 2.0f;
limited_angle = clamp(angle_deg, -effective_half_angle, effective_half_angle);
compare_value = center_compare - (limited_angle / effective_half_angle) * compare_delta;
```

Reset behavior:

```cpp
compare_value = center_compare;
```

This preserves the observed behavior from the source reference while fitting the
current project structure.

## Error Handling

The implementation should fail safe and avoid invalid output:

- Invalid `ServoId` marks the BSP object invalid.
- Invalid hardware configuration prevents angle output and returns `false`.
- `set_angle()` clamps angle before conversion.
- `reset()` always attempts to return to center if the BSP is valid.

The initial output after construction should be center compare so the servo starts
from a predictable neutral position.

## Integration Plan

- Add the new BSP and Driver files under `TerraMind_FW`.
- Update `MDK-ARM/TerraMind.uvprojx` to include the files in the existing `BSP` and `Driver` groups.
- Keep all existing modules unchanged unless a build issue requires a minimal include adjustment.

## Verification Plan

Implementation verification should focus on low-risk checks:

- Ensure the new files are included in the Keil project.
- Run diagnostics for the edited files.
- Manually verify that:
  - `reset()` writes center compare.
  - Positive and negative angles change compare in opposite directions.
  - A/B/C/D map to the requested timer channels.

If a small usage example is needed later, it can be added after the driver itself is in place.

## Scope Summary

This design is intentionally limited to a simple PWM servo driver with:

- fixed A/B/C/D hardware mapping,
- reference-compatible angle conversion,
- BSP + Driver layering consistent with the current project,
- minimal impact on unrelated modules.
