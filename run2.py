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

MOTOR_PINS = (
    (18, 17, 27, 22, 23),  # Motor 1: INT1, INT2
    (12, 5, 7, 24, 10),    # Motor 2: INT3, INT4
    (13, 16, 26, 9, 25),   # Motor 3: INT5, INT6
    (19, 20, 21, 11, 8),   # Motor 4: INT7, INT8
)
LIGHT_PIN = 4              # INT9, physical pin 7
PWM_FREQUENCY = 5000
TIMEOUT = 13.0
BOUNCE_MS = 50

class Motor:
    def __init__(self, number, pins):
        self.number = number
        step, direction, enable, self.odd_pin, self.even_pin = pins
        self.devices = []
        try:
            self.direction = DigitalOutputDevice(direction)
            self.devices.append(self.direction)
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
        self.object = 1 if GPIO.input(self.even_pin) == 0 else 0

    def start(self, toward_odd):
        if self.fault:
            return
        target = self.odd_pin if toward_odd else self.even_pin
        if GPIO.input(self.odd_pin) == 0 and GPIO.input(self.even_pin) == 0:
            self.stop()
            self.enable.on()
            self.fault = True
            print(f"Motor{self.number}: both limits active; stopped")
            return
        if GPIO.input(target) == 0:
            self.stop()
            self.object = 0 if toward_odd else 1
            return
        if self.moving:
            return
        self.step.value = 0.0
        self.direction.value = bool(toward_odd)
        self.enable.off()
        time.sleep(0.001)
        self.target = target
        self.started_at = time.monotonic()
        self.moving = True
        self.step.value = 0.5

    def stop(self):
        self.step.value = 0.0
        self.moving = False
        self.target = None

    def limit(self, pin):
        self.stop()
        self.object = 0 if pin == self.odd_pin else 1
        print(f"Motor{self.number}: limit GPIO{pin}, object={self.object}")

    def check(self):
        if not self.moving:
            return
        if GPIO.input(self.target) == 0:
            self.limit(self.target)
        elif time.monotonic() - self.started_at >= TIMEOUT:
            self.stop()
            self.enable.on()
            self.fault = True
            print(f"Motor{self.number} timeout! Inspect before restarting.")

    def close(self):
        try:
            self.stop()
            self.enable.on()
        finally:
            for device in reversed(self.devices):
                device.close()


class FourMotorPublisher(Node):
    """Keep the original String topic and include all four object states."""

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
            input("Enter to start\n")
        except (EOFError, OSError):
            return  # Autostart services may not have an interactive stdin.
        events.put(("keyboard", None))


def main():
    motors = []
    events = queue.Queue()
    messages = queue.Queue()
    stopping = threading.Event()
    input_pins = [pin for pins in MOTOR_PINS for pin in pins[3:]] + [LIGHT_PIN]
    registered = []
    publisher = subscriber = executor = None
    ros_initialized = False
    # All state changes and motor commands run on the main thread.
    light_enabled = False
    status = None

    try:
        GPIO.setmode(GPIO.BCM)
        for pin in input_pins:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        for number, pins in enumerate(MOTOR_PINS, 1):
            motors.append(Motor(number, pins))
        limit_motors = {
            pin: motor for motor in motors for pin in (motor.odd_pin, motor.even_pin)
        }
        for pin in limit_motors:
            GPIO.add_event_detect(
                pin, GPIO.FALLING,
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
        threading.Thread(target=keyboard_thread, args=(events, stopping), daemon=True).start()

        # Same startup direction as run.py, now with timeout protection.
        for motor in motors:
            motor.start(True)

        while rclpy.ok():
            executor.spin_once(timeout_sec=0.01)
            while True:
                try:
                    status = messages.get_nowait().status
                except queue.Empty:
                    break

            if status in (LinkStatus.UAV_WANNA_TAKEOFF, LinkStatus.UAV_APPROACHING):
                if light_enabled:
                    GPIO.remove_event_detect(LIGHT_PIN)
                    registered.remove(LIGHT_PIN)
                    light_enabled = False

            while True:
                try:
                    kind, pin = events.get_nowait()
                except queue.Empty:
                    break
                if kind == "limit":
                    limit_motors[pin].limit(pin)
                elif kind == "light" and light_enabled:
                    for motor in motors:
                        motor.start(False)
                elif kind == "keyboard":
                    for motor in motors:
                        if GPIO.input(motor.odd_pin) == 0:
                            motor.start(False)
                        elif GPIO.input(motor.even_pin) == 0:
                            motor.start(True)

            for motor in motors:
                motor.check()
                if status == LinkStatus.UAV_WANNA_TAKEOFF and GPIO.input(motor.even_pin) == 0:
                    motor.start(True)
                elif status == LinkStatus.UAV_APPROACHING and GPIO.input(motor.odd_pin) == 0:
                    motor.start(False)

            if (status == LinkStatus.UAV_DISARMED and not light_enabled
                    and all(GPIO.input(m.odd_pin) == 0 for m in motors)):
                GPIO.add_event_detect(
                    LIGHT_PIN, GPIO.RISING,
                    callback=lambda channel: events.put(("light", channel)),
                    bouncetime=BOUNCE_MS,
                )
                registered.append(LIGHT_PIN)
                light_enabled = True
    except KeyboardInterrupt:
        pass
    finally:
        stopping.set()
        for motor in motors:
            motor.close()
        for pin in registered:
            GPIO.remove_event_detect(pin)
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
