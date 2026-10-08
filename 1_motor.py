#!/usr/bin/env python3
import threading
import time

import RPi.GPIO as GPIO
from gpiozero import DigitalOutputDevice, PWMOutputDevice


# ============================================================
# 1 MOTOR + 2 LIMIT SENSORS
# BCM numbering
#
# Motor 1 wiring:
#   STEP      = GPIO18
#   DIR       = GPIO17
#   ENA       = GPIO27
#   ODD limit = GPIO22  -> CLAMPED position
#   EVEN limit= GPIO23  -> RELEASED position
#
# Mechanical convention:
#   DIR = 1 -> RELEASE -> move toward EVEN
#   DIR = 0 -> CLAMP   -> move toward ODD
#
# Limit sensors are active LOW:
#   GPIO = 0 -> sensor ACTIVE
#   GPIO = 1 -> sensor INACTIVE
# ============================================================

STEP_PIN = 18
DIR_PIN = 17
ENA_PIN = 27

ODD_PIN = 22
EVEN_PIN = 23

PWM_FREQUENCY = 5000
TIMEOUT = 13.0
POLL_INTERVAL = 0.01


class Motor:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ODD_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(EVEN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.direction = DigitalOutputDevice(DIR_PIN)

        # ENA is treated as active-low:
        # on()  -> disabled
        # off() -> enabled
        self.enable = DigitalOutputDevice(ENA_PIN, initial_value=True)

        self.step = PWMOutputDevice(
            STEP_PIN,
            frequency=PWM_FREQUENCY,
            initial_value=0.0,
        )

        self.moving = False
        self.target = None
        self.started_at = 0.0
        self.fault = False

    # --------------------------------------------------------
    # Sensor helpers
    # --------------------------------------------------------
    def odd_active(self):
        return GPIO.input(ODD_PIN) == GPIO.LOW

    def even_active(self):
        return GPIO.input(EVEN_PIN) == GPIO.LOW

    def endpoint(self):
        odd = self.odd_active()
        even = self.even_active()

        if odd and even:
            return "INVALID_BOTH"
        if odd:
            return "ODD"
        if even:
            return "EVEN"
        return "MIDDLE"

    # --------------------------------------------------------
    # Motor control
    # --------------------------------------------------------
    def stop(self):
        self.step.value = 0.0
        self.moving = False
        self.target = None

    def fault_stop(self, reason):
        self.stop()
        self.enable.on()
        self.fault = True
        print(f"\nFAULT: {reason}")
        print("Motor disabled. Fix the problem and restart the program.")

    def start_toward(self, target):
        if self.fault:
            print("Start ignored: motor is in FAULT.")
            return False

        if self.moving:
            print("Start ignored: motor is already moving.")
            return False

        if self.odd_active() and self.even_active():
            self.fault_stop("ODD and EVEN sensors are active at the same time.")
            return False

        if target == "ODD":
            # CLAMP
            target_pin = ODD_PIN
            dir_value = 0
            action = "CLAMP"
        elif target == "EVEN":
            # RELEASE
            target_pin = EVEN_PIN
            dir_value = 1
            action = "RELEASE"
        else:
            raise ValueError("target must be 'ODD' or 'EVEN'")

        # Already at requested destination.
        if GPIO.input(target_pin) == GPIO.LOW:
            print(f"Already at {target} limit. No movement needed.")
            return False

        # Set direction.
        self.direction.value = bool(dir_value)

        # Enable driver.
        self.enable.off()
        time.sleep(0.001)

        self.target = target
        self.started_at = time.monotonic()
        self.moving = True

        # 50% duty cycle STEP signal.
        self.step.value = 0.5

        print(
            f"{action} started: DIR={dir_value}, "
            f"target={target} GPIO{target_pin}"
        )
        return True

    def clamp(self):
        # DIR = 0 -> ODD
        return self.start_toward("ODD")

    def release(self):
        # DIR = 1 -> EVEN
        return self.start_toward("EVEN")

    # --------------------------------------------------------
    # Safety / limit checking
    # --------------------------------------------------------
    def check(self):
        if self.fault:
            return

        # Impossible / unsafe sensor state.
        if self.odd_active() and self.even_active():
            self.fault_stop("ODD and EVEN sensors became active together.")
            return

        if not self.moving:
            return

        # Only the TARGET sensor is allowed to stop the motor.
        if self.target == "ODD" and self.odd_active():
            self.stop()
            print("\nReached ODD -> CLAMPED. Motor stopped.")
            return

        if self.target == "EVEN" and self.even_active():
            self.stop()
            print("\nReached EVEN -> RELEASED. Motor stopped.")
            return

        # Timeout protection.
        if time.monotonic() - self.started_at >= TIMEOUT:
            target = self.target
            self.fault_stop(
                f"timeout while moving toward {target} "
                f"(>{TIMEOUT:.1f} s)"
            )

    def toggle_from_endpoint(self):
        """
        Enter is accepted only when motor is stopped at a valid endpoint.

        EVEN -> Enter -> CLAMP  -> DIR=0 -> ODD
        ODD  -> Enter -> RELEASE -> DIR=1 -> EVEN

        If motor is moving or is in the middle, Enter is ignored.
        """
        if self.fault:
            print("ENTER ignored: motor is in FAULT.")
            return

        if self.moving:
            print("ENTER ignored: motor is still moving.")
            return

        position = self.endpoint()

        if position == "EVEN":
            print("ENTER: RELEASED at EVEN -> start CLAMP.")
            self.clamp()

        elif position == "ODD":
            print("ENTER: CLAMPED at ODD -> start RELEASE.")
            self.release()

        elif position == "MIDDLE":
            print(
                "ENTER ignored: motor is in MIDDLE; "
                "neither ODD nor EVEN sensor is active."
            )

        elif position == "INVALID_BOTH":
            self.fault_stop(
                "ODD and EVEN sensors are active at the same time."
            )

    def close(self):
        try:
            self.stop()
            self.enable.on()
        finally:
            self.step.close()
            self.enable.close()
            self.direction.close()


def keyboard_worker(motor, stopping):
    while not stopping.is_set():
        try:
            input("\nPress ENTER to toggle CLAMP / RELEASE...\n")
        except (EOFError, OSError):
            return

        if stopping.is_set():
            return

        motor.toggle_from_endpoint()


def main():
    GPIO.setwarnings(False)

    motor = None
    stopping = threading.Event()

    try:
        motor = Motor()

        print("=" * 64)
        print("1 MOTOR + 2 SENSOR TEST")
        print("DIR=1 -> RELEASE -> EVEN GPIO23")
        print("DIR=0 -> CLAMP   -> ODD  GPIO22")
        print("Sensor ACTIVE = GPIO LOW")
        print("=" * 64)

        print(
            f"Initial sensors: "
            f"ODD={int(motor.odd_active())}, "
            f"EVEN={int(motor.even_active())}, "
            f"position={motor.endpoint()}"
        )

        # ====================================================
        # STARTUP RESET / HOMING
        # ====================================================
        # On power-up, the clamp MUST end in RELEASED position.
        # Therefore force DIR=1 and move toward EVEN.
        print("\nSTARTUP: homing to RELEASED / EVEN position...")

        motor.release()

        while not motor.fault:
            motor.check()

            if not motor.moving:
                break

            time.sleep(POLL_INTERVAL)

        if motor.fault:
            return

        if motor.endpoint() != "EVEN":
            motor.fault_stop(
                "startup ended but EVEN sensor is not active."
            )
            return

        print("\nSTARTUP COMPLETE: motor is RELEASED at EVEN.")
        print("Now ENTER can toggle between EVEN and ODD.")

        # Start keyboard only after startup homing succeeds.
        keyboard_thread = threading.Thread(
            target=keyboard_worker,
            args=(motor, stopping),
            daemon=True,
        )
        keyboard_thread.start()

        # Main safety loop.
        while not motor.fault:
            motor.check()
            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\nStopping program...")

    finally:
        stopping.set()

        if motor is not None:
            motor.close()

        GPIO.cleanup([ODD_PIN, EVEN_PIN])
        print("GPIO cleanup completed.")


if __name__ == "__main__":
    main()
