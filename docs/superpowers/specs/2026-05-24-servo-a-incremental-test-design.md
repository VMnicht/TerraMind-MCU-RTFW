# Servo A Incremental Test Design

## Background

The project already has a running PWM motor test entry:

- `AppTest_PwmMotor_Init()`
- `AppTest_PwmMotor_TaskStep()`

The new request is to incrementally add a servo test without removing the
existing motor test, and only verify servo port A.

## Goal

- Add a standalone servo A test entry.
- Keep the existing PWM motor test enabled.
- Limit the scope to servo A only.
- Use a simple staged motion sequence that is easy to observe on hardware.

## Scope

Files to modify:

- `TerraMind_FW/App/test.h`
- `TerraMind_FW/App/test.cpp`
- `Core/Src/freertos.c`

No changes are needed in:

- `PwmServoBsp`
- `PwmServo`
- Keil project file

## Chosen Approach

Add a second test flow beside the existing motor test:

- `AppTest_ServoA_Init()`
- `AppTest_ServoA_TaskStep()`

`freertos.c` will keep calling the motor test, and will also call the new servo A
test in the same loop.

This keeps the change incremental and avoids mixing unrelated test logic inside
the existing motor test functions.

## Servo A Test Behavior

The servo test creates one `PwmServo` object bound to `PwmServoBsp::SERVO_A`.

The test sequence cycles through four phases:

- center
- positive angle
- center
- negative angle

Each phase lasts a few seconds and writes the target angle once per task step.
Debug logs print the current phase name and target angle periodically.

## Configuration

Servo A uses the current default servo driver settings:

- `servo_id = PwmServoBsp::SERVO_A`
- `max_angle_deg = 180.0f`
- `center_compare = 1825.0f`
- `compare_delta = 115.0f`

The staged target angles will be conservative so hardware observation is easier.

## Error Handling

- If servo object creation fails logically, mark the servo test as not ready.
- If the servo test is not ready, `AppTest_ServoA_TaskStep()` returns immediately.
- Existing PWM motor test behavior remains unchanged.

## Verification

- Confirm both motor and servo test init functions run.
- Confirm servo A moves through center, positive, center, negative phases.
- Confirm debug output shows servo phase transitions.
- Run diagnostics after modifying the test files and `freertos.c`.
