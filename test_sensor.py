#!/usr/bin/env python3
import time
import RPi.GPIO as GPIO

SENSORS = {
    "S1_M1_ODD": 22,
    "S2_M1_EVEN": 23,
    "S3_M2_ODD": 24,
    "S4_M2_EVEN": 10,
    "S5_M3_ODD": 9,
    "S6_M3_EVEN": 25,
    "S7_M4_ODD": 11,
    "S8_M4_EVEN": 8,
    "S9_LIGHT": 4,
}

ACTIVE_LOW = True

POLL_INTERVAL = 0.05       # 50 ms
SUMMARY_INTERVAL = 0.5     # refresh summary every 0.5 s


def is_active(pin: int) -> bool:
    value = GPIO.input(pin)
    return value == GPIO.LOW if ACTIVE_LOW else value == GPIO.HIGH


def state_text(active: bool) -> str:
    return "DETECTED" if active else "CLEAR"


def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    for pin in SENSORS.values():
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    last_state = {
        name: is_active(pin)
        for name, pin in SENSORS.items()
    }

    transition_count = {name: 0 for name in SENSORS}

    print("=" * 66)
    print("8-SENSOR STANDALONE TEST")
    print("BCM numbering | ACTIVE LOW: GPIO 0 = DETECTED, GPIO 1 = CLEAR")
    print("Press Ctrl+C to stop.")
    print("=" * 66)

    for name, pin in SENSORS.items():
        print(
            f"{name:12s} | GPIO {pin:2d} | "
            f"raw={GPIO.input(pin)} | {state_text(last_state[name])}"
        )

    print("-" * 66)

    last_summary = time.monotonic()

    try:
        while True:
            now = time.monotonic()

            # Detect and print state changes immediately.
            for name, pin in SENSORS.items():
                current = is_active(pin)

                if current != last_state[name]:
                    transition_count[name] += 1
                    raw = GPIO.input(pin)

                    timestamp = time.strftime("%H:%M:%S")
                    print(
                        f"[{timestamp}] {name:12s} GPIO {pin:2d}: "
                        f"{state_text(last_state[name])} -> "
                        f"{state_text(current)} "
                        f"(raw={raw}, changes={transition_count[name]})"
                    )

                    last_state[name] = current

            # Periodic one-line overview of all 8 sensors.
            if now - last_summary >= SUMMARY_INTERVAL:
                summary = " | ".join(
                    f"{name}={'1' if last_state[name] else '0'}"
                    for name in SENSORS
                )
                print(f"STATE: {summary}")
                last_summary = now

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\nStopping sensor test...")

    finally:
        GPIO.cleanup(list(SENSORS.values()))
        print("GPIO cleanup completed.")


if __name__ == "__main__":
    main()

