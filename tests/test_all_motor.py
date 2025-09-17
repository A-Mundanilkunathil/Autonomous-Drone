from pymavlink import mavutil
import time

# ---------------- CONFIG ----------------
PORT = 'COM6'   # e.g. 'COM6' on Windows
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
    throttle_type: 0=percent, 1=PWM
    throttle_value: depends on throttle_type
    timeout: seconds to run this step (float)
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
        0,
        motor_number,
        throttle_type,
        float(throttle_value),
        float(timeout),
        0,
        0,
        0
    )


def ramp_all_motors_pwm(master, pwm_min, pwm_max, step, dwell, hold):
    # ramp up
    for pwm in range(pwm_min, pwm_max + 1, step):
        for motor in range(1, MOTOR_COUNT + 1):
            motor_test(master, motor, pwm, throttle_type=1, timeout=dwell)
        print(f"[INFO] PWM set to {pwm} on all motors")
        time.sleep(dwell)
    time.sleep(hold)
    # ramp down
    for pwm in range(pwm_max, pwm_min - 1, -step):
        for motor in range(1, MOTOR_COUNT + 1):
            motor_test(master, motor, pwm, throttle_type=1, timeout=dwell)
        print(f"[INFO] PWM set to {pwm} on all motors")
        time.sleep(dwell)


def ramp_all_motors_percent(master, pct_min, pct_max, step, dwell, hold):
    # ramp up
    for pct in range(pct_min, pct_max + 1, step):
        for motor in range(1, MOTOR_COUNT + 1):
            motor_test(master, motor, pct, throttle_type=0, timeout=dwell)
        print(f"[INFO] {pct}% throttle on all motors")
        time.sleep(dwell)
    time.sleep(hold)
    # ramp down
    for pct in range(pct_max, pct_min - 1, -step):
        for motor in range(1, MOTOR_COUNT + 1):
            motor_test(master, motor, pct, throttle_type=0, timeout=dwell)
        print(f"[INFO] {pct}% throttle on all motors")
        time.sleep(dwell)


def main():
    print(f"[INFO] Connecting to {PORT} @ {BAUD}...")
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
    master.wait_heartbeat()
    print(f"[INFO] Connected: sys={master.target_system} comp={master.target_component}")

    print("[INFO] Arming...")
    master.arducopter_arm()
    master.motors_armed_wait()
    time.sleep(1.0)

    try:
        print("[INFO] Ramping all motors simultaneously...")
        if USE_PWM:
            ramp_all_motors_pwm(master, PWM_MIN, PWM_MAX, STEP, DWELL, HOLD)
        else:
            ramp_all_motors_percent(master, PCT_MIN, PCT_MAX, max(1, STEP), DWELL, HOLD)

    except KeyboardInterrupt:
        print("[INFO] Interrupted by user.")

    finally:
        print("[INFO] Disarming...")
        master.arducopter_disarm()
        master.motors_disarmed_wait()
        print("[INFO] Done.")


if __name__ == "__main__":
    main()
