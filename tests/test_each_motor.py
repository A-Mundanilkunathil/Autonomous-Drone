from pymavlink import mavutil
import time

# ---------------- CONFIG ----------------
PORT = '/dev/tty.usbserial-0001'   # e.g. 'COM6' on Windows, '/dev/ttyUSB0' on Linux, '/dev/tty.usbmodem*' on macOS
BAUD = 57600
MOTOR_COUNT = 4           # number of motors on your frame
USE_PWM = True            # True = use PWM (1000–2000), False = use percent (0–100)

# Ramp profile
PWM_MIN = 1000            # min PWM
PWM_MAX = 2000            # max PWM
PCT_MIN = 0               # min percent
PCT_MAX = 100             # max percent
STEP = 50                 # step size (PWM µs if USE_PWM=True; percent otherwise)
DWELL = 0.20              # seconds to hold each step
HOLD = 1.5                # seconds to hold at max before ramping down
# ---------------------------------------


def motor_test(master, motor_number, throttle_value, throttle_type, timeout):
    """
    Send MAV_CMD_DO_MOTOR_TEST.
    motor_number: 1..N
    throttle_type: 0=percent, 1=PWM, 2=pilot throttle passthrough (ArduCopter)
    throttle_value: depends on throttle_type
    timeout: seconds to run this step (float)
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,                      # confirmation
        motor_number,           # param1: motor instance
        throttle_type,          # param2: throttle type
        float(throttle_value),  # param3: throttle
        float(timeout),         # param4: timeout (s)
        0,                      # param5: (unused here)
        0,                      # param6: (unused here)
        0                       # param7: (unused)
    )


def ramp_motor_pwm(master, motor, pwm_min, pwm_max, step, dwell, hold):
    # ramp up
    for pwm in range(pwm_min, pwm_max + 1, step):
        print(f"[Motor {motor}] PWM {pwm}")
        motor_test(master, motor, pwm, throttle_type=1, timeout=dwell)
        time.sleep(dwell)
    time.sleep(hold)
    # ramp down
    for pwm in range(pwm_max, pwm_min - 1, -step):
        print(f"[Motor {motor}] PWM {pwm}")
        motor_test(master, motor, pwm, throttle_type=1, timeout=dwell)
        time.sleep(dwell)


def ramp_motor_percent(master, motor, pct_min, pct_max, step, dwell, hold):
    # ramp up
    for pct in range(pct_min, pct_max + 1, step):
        print(f"[Motor {motor}] {pct}%")
        motor_test(master, motor, pct, throttle_type=0, timeout=dwell)
        time.sleep(dwell)
    time.sleep(hold)
    # ramp down
    for pct in range(pct_max, pct_min - 1, -step):
        print(f"[Motor {motor}] {pct}%")
        motor_test(master, motor, pct, throttle_type=0, timeout=dwell)
        time.sleep(dwell)


def main():
    print(f"[INFO] Connecting to {PORT} @ {BAUD}...")
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
    master.wait_heartbeat()
    print(f"[INFO] Connected: sys={master.target_system} comp={master.target_component}")

    # Arming is often required by safety checks; if your setup allows disarmed motor test, you can skip this.
    print("[INFO] Arming...")
    master.arducopter_arm()
    master.motors_armed_wait()
    time.sleep(1.0)

    try:
        for m in range(1, MOTOR_COUNT + 1):
            print(f"[INFO] Ramping Motor {m}...")
            if USE_PWM:
                ramp_motor_pwm(master, m, PWM_MIN, PWM_MAX, STEP, DWELL, HOLD)
            else:
                ramp_motor_percent(master, m, PCT_MIN, PCT_MAX, max(1, STEP), DWELL, HOLD)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user.")

    finally:
        print("[INFO] Disarming...")
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print("[INFO] Done.")


if __name__ == "__main__":
    main()
