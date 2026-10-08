import queue
import threading
import time

import RPi.GPIO as GPIO
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from ctuav_link_interfaces.msg import LinkStatus
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from ros2_ws.subscribe import MinimalSubscriber


# (STEP, DIR, ENA, ODD_LIMIT, EVEN_LIMIT)
#
# Convention:
#   DIR = 1 -> RELEASE -> move toward EVEN limit
#   DIR = 0 -> CLAMP   -> move toward ODD limit
#
# Limit inputs are active LOW:
#   GPIO = 0 -> limit sensor active
#   GPIO = 1 -> limit sensor inactive
MOTOR_PINS = (
    (18, 17, 27, 22, 23),  # Motor 1: ODD=INT1, EVEN=INT2
    (12, 5, 7, 24, 10),    # Motor 2: ODD=INT3, EVEN=INT4
    (13, 16, 26, 9, 25),   # Motor 3: ODD=INT5, EVEN=INT6
    (19, 20, 21, 11, 8),   # Motor 4: ODD=INT7, EVEN=INT8
)

LIGHT_PIN = 4               # INT9, physical pin 7
PWM_FREQUENCY = 8000
TIMEOUT = 500.0
BOUNCE_MS = 50
CLAMP_STAGE_DELAY = 4.0  # Seconds between starting motors 2/4 and 1/3.

# False: home to EVEN, then move only on Enter.
# True: allow ROS and light commands until Enter selects manual control.
AUTO_CONTROL = True


class Motor:
    def __init__(self, number, pins):
        self.number = number
        step, direction, enable, self.odd_pin, self.even_pin = pins

        self.devices = []
        try:
            self.direction = DigitalOutputDevice(direction)
            self.devices.append(self.direction)

            # ENA is treated as active-low:
            # on()  -> disabled
            # off() -> enabled
            self.enable = DigitalOutputDevice(enable, initial_value=True)
            self.devices.append(self.enable)

            self.step = PWMOutputDevice(step, frequency=PWM_FREQUENCY)
            self.devices.append(self.step)

        except Exception:
            for device in reversed(self.devices):
                device.close()
            raise

        self.moving = False
        self.started_at = 0.0
        self.target = None
        self.fault = False

        # Keep the original semantic:
        # object = 1 means released at EVEN
        # object = 0 means clamped / not released
        self.object = 1 if self.even_active() else 0

    def odd_active(self):
        return GPIO.input(self.odd_pin) == GPIO.LOW

    def even_active(self):
        return GPIO.input(self.even_pin) == GPIO.LOW

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

    def fault_stop(self, reason):
        self.step.value = 0.0
        self.moving = False
        self.target = None
        self.enable.on()
        self.fault = True
        print(f"Motor{self.number}: FAULT - {reason}")

    def start(self, toward_odd):
        """
        toward_odd=True:
            DIR=0, CLAMP, target=ODD

        toward_odd=False:
            DIR=1, RELEASE, target=EVEN
        """
        if self.fault:
            print(f"Motor{self.number}: start ignored because motor is in FAULT")
            return False

        if self.moving:
            print(f"Motor{self.number}: start ignored because motor is already moving")
            return False

        if self.odd_active() and self.even_active():
            self.fault_stop("both ODD and EVEN limits are active")
            return False

        target = self.odd_pin if toward_odd else self.even_pin
        action = "CLAMP" if toward_odd else "RELEASE"

        # Already at the requested destination.
        if GPIO.input(target) == GPIO.LOW:
            self.stop()
            self.object = 0 if toward_odd else 1
            print(
                f"Motor{self.number}: already at {action} limit "
                f"(GPIO{target})"
            )
            return False

        self.step.value = 0.0

        # Required mechanical convention:
        # DIR=0 -> clamp toward ODD
        # DIR=1 -> release toward EVEN
        self.direction.value = not bool(toward_odd)

        # Enable driver, then start step pulses.
        self.enable.off()
        time.sleep(0.001)

        self.target = target
        self.started_at = time.monotonic()
        self.moving = True
        self.step.value = 0.5

        print(
            f"Motor{self.number}: {action} started, "
            f"DIR={0 if toward_odd else 1}, target=GPIO{target}"
        )
        return True

    def clamp(self):
        return self.start(True)

    def release(self):
        return self.start(False)

    def stop(self):
        self.step.value = 0.0
        self.moving = False
        self.target = None

    def limit(self, pin):
        """
        A limit interrupt is allowed to stop the motor ONLY when:
          1. the motor is currently moving, and
          2. this pin is the current target limit.

        Therefore, when releasing toward EVEN, an ODD interrupt is ignored.
        When clamping toward ODD, an EVEN interrupt is ignored.
        """
        if not self.moving:
            return

        if pin != self.target:
            print(
                f"Motor{self.number}: ignored non-target limit GPIO{pin}; "
                f"current target is GPIO{self.target}"
            )
            return

        self.stop()

        if pin == self.odd_pin:
            self.object = 0
            position = "CLAMPED / ODD"
        else:
            self.object = 1
            position = "RELEASED / EVEN"

        print(
            f"Motor{self.number}: reached {position} limit "
            f"at GPIO{pin}; stopped"
        )

    def check(self):
        if self.odd_active() and self.even_active():
            self.fault_stop("both ODD and EVEN limits became active")
            return

        if not self.moving:
            return

        # Poll the TARGET as a backup to the GPIO interrupt.
        if GPIO.input(self.target) == GPIO.LOW:
            self.limit(self.target)
            return

        if time.monotonic() - self.started_at >= TIMEOUT:
            target = self.target
            self.fault_stop(
                f"timeout while moving toward GPIO{target}; inspect mechanism"
            )

    def close(self):
        try:
            self.stop()
            self.enable.on()
        finally:
            for device in reversed(self.devices):
                device.close()


class FourMotorPublisher(Node):
    def __init__(self, motors):
        super().__init__("minimal_publisher")
        self.motors = motors
        self.publisher = self.create_publisher(String, "topic", 10)
        self.create_timer(0.5, self.publish_objects)

    def publish_objects(self):
        msg = String()
        msg.data = ", ".join(
            f"object{motor.number}: {motor.object}" for motor in self.motors
        )
        self.publisher.publish(msg)


def keyboard_thread(events, stopping):
    while not stopping.is_set():
        try:
            input("Press Enter to toggle CLAMP / RELEASE\n")
        except (EOFError, OSError):
            return

        events.put(("keyboard", None))


def motor_states(motors):
    return ", ".join(
        f"M{motor.number}={motor.endpoint()}"
        for motor in motors
    )


class ClampSequence:
    """Clamp 2/4 first, then start 1/3 after a non-blocking five-second delay."""

    def __init__(self, motors):
        self.motors = motors
        self.first = [motor for motor in motors if motor.number in (2, 4)]
        self.second = [motor for motor in motors if motor.number in (1, 3)]
        self.stage = None
        self.started_at = None

    @property
    def active(self):
        return self.stage is not None

    def cancel(self):
        # Cancel pending stages; moving motors still stop at their target limits.
        self.stage = None
        self.started_at = None

    def start(self):
        if self.active or any(motor.fault or motor.moving for motor in self.motors):
            return
        if any(motor.endpoint() not in ("ODD", "EVEN") for motor in self.motors):
            return
        if all(motor.endpoint() == "ODD" for motor in self.motors):
            return
        self.stage = 1
        self.started_at = time.monotonic()
        print("CLAMP stage 1: motors 2 and 4")
        for motor in self.first:
            motor.clamp()

    def check(self):
        if not self.active:
            return
        if any(motor.fault for motor in self.motors):
            self.cancel()
            print("CLAMP sequence cancelled: motor fault")
            return
        if (
            self.stage == 1
            and time.monotonic() - self.started_at >= CLAMP_STAGE_DELAY
        ):
            self.stage = 2
            print("CLAMP stage 2: motors 1 and 3 after 5 seconds")
            for motor in self.second:
                motor.clamp()
        if self.stage == 2 and all(
            not motor.moving and motor.endpoint() == "ODD"
            for motor in self.motors
        ):
            self.cancel()
            print("CLAMP complete: all 4 motors at ODD")


def toggle_all_from_limits(motors, clamp_sequence):
    """
    Manual Enter control is intentionally strict:

      - all 4 at EVEN -> CLAMP 2/4, then 1/3 after 5 seconds (DIR=0 -> ODD)
      - all 4 at ODD  -> RELEASE all 4 (DIR=1 -> EVEN)

    Enter is ignored if:
      - any motor is moving,
      - any motor is in fault,
      - any motor is in the middle,
      - limits are mixed between motors,
      - both limits are active on any motor.
    """
    if any(motor.fault for motor in motors):
        print("ENTER ignored: at least one motor is in FAULT")
        return

    if clamp_sequence.active or any(motor.moving for motor in motors):
        print("ENTER ignored: clamp sequence or motor motion is active")
        return

    states = [motor.endpoint() for motor in motors]

    if all(state == "EVEN" for state in states):
        print("ENTER: all motors RELEASED -> start CLAMP (DIR=0)")
        clamp_sequence.start()
        return

    if all(state == "ODD" for state in states):
        print("ENTER: all motors CLAMPED -> start RELEASE (DIR=1)")
        for motor in motors:
            motor.release()
        return

    print(
        "ENTER ignored: motors are not all at the same valid endpoint. "
        f"States: {motor_states(motors)}"
    )


def main():
    motors = []
    events = queue.Queue()
    messages = queue.Queue()
    stopping = threading.Event()

    input_pins = [
        pin
        for pins in MOTOR_PINS
        for pin in pins[3:]
    ] + [LIGHT_PIN]

    registered = []

    publisher = None
    subscriber = None
    executor = None

    ros_initialized = False
    last_light_block = None
    status = None
    manual_control = not AUTO_CONTROL

    # Startup state:
    # Before normal control is allowed, every clamp must be released
    # and physically reach its EVEN limit.
    homing = True
    homing_failed = False
    keyboard_started = False

    try:
        GPIO.setmode(GPIO.BCM)

        for pin in input_pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        for number, pins in enumerate(MOTOR_PINS, 1):
            motors.append(Motor(number, pins))

        clamp_sequence = ClampSequence(motors)

        limit_motors = {
            pin: motor
            for motor in motors
            for pin in (motor.odd_pin, motor.even_pin)
        }

        # Both ODD and EVEN sensors are monitored,
        # but Motor.limit() only accepts the CURRENT TARGET sensor.
        for pin in limit_motors:
            GPIO.add_event_detect(
                pin,
                GPIO.FALLING,
                callback=lambda channel: events.put(("limit", channel)),
                bouncetime=BOUNCE_MS,
            )
            registered.append(pin)

        rclpy.init()
        ros_initialized = True

        publisher = FourMotorPublisher(motors)
        subscriber = MinimalSubscriber(msg_queue=messages)

        executor = SingleThreadedExecutor()
        executor.add_node(publisher)
        executor.add_node(subscriber)

        # ----------------------------------------------------------
        # STARTUP HOME / RESET
        # ----------------------------------------------------------
        # Mandatory initial condition:
        #   DIR=1 -> RELEASE toward EVEN.
        # Each motor stops only when its own EVEN sensor is active.
        print("=" * 70)
        print("STARTUP HOMING: releasing all clamps toward EVEN limits (DIR=1)")
        print("Normal ROS / keyboard motion is locked until all EVEN limits are active.")
        print("=" * 70)

        for motor in motors:
            motor.release()

        while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)

            # Always keep the latest ROS status, but do not act on it
            # until startup homing has successfully completed.
            while True:
                try:
                    status = messages.get_nowait().status
                except queue.Empty:
                    break

            # Process limit interrupts first.
            pending_non_limit_events = []

            while True:
                try:
                    kind, pin = events.get_nowait()
                except queue.Empty:
                    break

                if kind == "limit":
                    limit_motors[pin].limit(pin)
                else:
                    pending_non_limit_events.append((kind, pin))

            # Polling backup + timeout/fault protection.
            for motor in motors:
                motor.check()

            # ------------------------------------------------------
            # HOMING STATE
            # ------------------------------------------------------
            if homing:
                if any(motor.fault for motor in motors):
                    homing = False
                    homing_failed = True

                    print(
                        "STARTUP HOMING FAILED. "
                        f"States: {motor_states(motors)}"
                    )
                    print(
                        "All motors are being stopped. "
                        "Restart after inspecting sensors/mechanics."
                    )

                    for motor in motors:
                        motor.stop()
                        motor.enable.on()

                    # Discard non-limit events received during homing.
                    continue

                all_even = all(
                    motor.endpoint() == "EVEN"
                    for motor in motors
                )

                all_stopped = all(
                    not motor.moving
                    for motor in motors
                )

                if all_even and all_stopped:
                    homing = False

                    print("=" * 70)
                    print("STARTUP HOMING COMPLETE")
                    print("All 4 motors are RELEASED at EVEN limits.")
                    print("Control: MANUAL (Enter)" if manual_control else "Control: AUTO (ROS / light)")
                    print("=" * 70)

                    if not keyboard_started:
                        threading.Thread(
                            target=keyboard_thread,
                            args=(events, stopping),
                            daemon=True,
                        ).start()
                        keyboard_started = True

                # Do not execute normal commands in the same iteration
                # while homing is still active.
                if homing:
                    continue

            # If startup homing failed, no further motion is allowed.
            if homing_failed:
                continue

            # Restore non-limit events that may have been queued this cycle.
            for item in pending_non_limit_events:
                events.put(item)

            # ------------------------------------------------------
            # NORMAL OPERATION
            # ------------------------------------------------------
            # Poll INT9 as an active-HIGH level, not an edge trigger.
            if (
                not manual_control
                and status == LinkStatus.UAV_DISARMED
                and GPIO.input(LIGHT_PIN) == GPIO.HIGH
            ):
                events.put(("light", LIGHT_PIN))
            else:
                last_light_block = None

            while True:
                try:
                    kind, pin = events.get_nowait()
                except queue.Empty:
                    break

                if kind == "limit":
                    limit_motors[pin].limit(pin)

                elif (
                    kind == "light"
                    and not manual_control
                    and status == LinkStatus.UAV_DISARMED
                ):
                    # Only HIGH (raw=1) may initiate a light-triggered clamp.
                    if GPIO.input(LIGHT_PIN) != GPIO.HIGH:
                        continue
                    # An accepted sequence continues independently of S9.
                    if clamp_sequence.active:
                        continue

                    states = tuple(
                        (motor.number, motor.endpoint(), motor.moving, motor.fault)
                        for motor in motors
                    )
                    if all(
                        endpoint == "ODD" and not moving and not fault
                        for _, endpoint, moving, fault in states
                    ):
                        last_light_block = None
                        continue

                    if all(
                        endpoint == "EVEN" and not moving and not fault
                        for _, endpoint, moving, fault in states
                    ):
                        last_light_block = None
                        print("LIGHT HIGH raw=1: CLAMP all motors (DIR=0 -> ODD)")
                        clamp_sequence.start()
                    elif states != last_light_block:
                        details = ", ".join(
                            f"M{number}={endpoint} moving={moving} fault={fault}"
                            for number, endpoint, moving, fault in states
                        )
                        print(f"LIGHT ignored: requires stopped EVEN, no faults; {details}")
                        last_light_block = states

                elif kind == "keyboard":
                    if not manual_control:
                        manual_control = True
                        print("Control: MANUAL until restart; ROS / light motion disabled")
                    toggle_all_from_limits(motors, clamp_sequence)

            for motor in motors:
                motor.check()

            if not manual_control:
                if status == LinkStatus.UAV_APPROACHING:
                    # State 20 cancels pending clamp stages and releases motors.
                    clamp_sequence.cancel()
                    for motor in motors:
                        if not motor.moving and motor.endpoint() == "ODD":
                            motor.release()
                elif status == LinkStatus.UAV_WANNA_TAKEOFF:
                    clamp_sequence.start()

            # Continue accepted light/manual sequences until completion.
            clamp_sequence.check()

    except KeyboardInterrupt:
        pass

    finally:
        stopping.set()

        for motor in motors:
            motor.close()

        for pin in list(registered):
            try:
                GPIO.remove_event_detect(pin)
            except RuntimeError:
                pass

        GPIO.cleanup(input_pins)

        if executor is not None:
            executor.shutdown()

        for node in (publisher, subscriber):
            if node is not None:
                node.destroy_node()

        if ros_initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
