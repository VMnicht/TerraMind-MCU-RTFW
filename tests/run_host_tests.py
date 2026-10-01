"""Run protocol, PWM and real application regression tests without a board.

Usage: python tests/run_host_tests.py [--cxx g++]
Test binaries are compiled into an automatically cleaned temporary directory.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default="g++")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    fw = "TerraMind_FW/"
    common = [args.cxx, "-std=c++11", "-Wall", "-Wextra", "-Wno-missing-field-initializers",
              "-Itests/fakes"]

    with tempfile.TemporaryDirectory(prefix="terramind-tests-") as tmp:
        def build(name, sources, defines=()):
            exe = str(Path(tmp) / (name + ".exe"))
            subprocess.run(common + list(defines) + sources + ["-o", exe], cwd=root, check=True)
            return exe

        protocol = build("pc_protocol", ["tests/pc_protocol_test.cpp", fw + "Driver/pc_protocol.cpp"])
        subprocess.run([protocol], check=True)
        pwm_sources = ["tests/fakes/hal.cpp", fw + "BSP/pwm_enc_bsp.cpp",
                       fw + "Driver/pwm_open_loop_motor.cpp"]
        motor = build("pwm_open_loop", ["tests/pwm_open_loop_motor_test.cpp"] + pwm_sources)
        subprocess.run([motor], check=True)
        app_sources = ["tests/spray_app_test.cpp"] + pwm_sources + [fw + path for path in (
            "Algorithm/PID.cpp", "Device/diff_chassis.cpp", "Driver/M3508.cpp",
            "Driver/cmd_port.cpp", "Driver/pc_port.cpp", "Driver/pc_protocol.cpp",
            "Driver/debug_printer.cpp", "Driver/pwm_motor.cpp", "Driver/pwm_servo.cpp",
            "Driver/pwm_esc.cpp", "BSP/Serial_device.cpp", "BSP/pwm_servo_bsp.cpp",
            "BSP/pwm_esc_bsp.cpp")]
        app = build("spray_app", app_sources)
        subprocess.run([app], check=True)
    print("PASS: protocol, calibrated PWM, PC pump control/stop/timeout, UART5/PC chassis direction, and unchanged other actuator outputs")


if __name__ == "__main__":
    main()
