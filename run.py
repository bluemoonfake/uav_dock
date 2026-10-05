
import RPi.GPIO as GPIO
import threading
import time
import os
import sys
import queue
sys.path.append(os.path.join(os.path.dirname(__file__), "ros2_ws"))
import rclpy
from ros2_ws.publish import MinimalPublisher
from ros2_ws.subscribe import MinimalSubscriber
from ctuav_link_interfaces.msg import LinkStatus
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from gpiozero import DigitalInputDevice, Servo
from time import sleep

msg_queue = queue.Queue()

DIR_PIN1 = 16
ENA_PIN1 = 23
STEP_PIN1 = 12

DIR_PIN2 = 24
ENA_PIN2 = 25
STEP_PIN2 = 13

PIN1 = 19
PIN2 = 26
PIN3 = 5
PIN4 = 6
PIN5 = 27

TIMEOUT = 13.0

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN2, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN3, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN4, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(PIN5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

uav_link_status = {
    "interface_version": 0,
    "self_id": "",
    "peer_id": "",
    "stamp": 0,
    "seq": 0,
    "role": 0,
    "status": 0,
    "online": False,
    "gps_ok": False,
    "peer_seen": False,
    "peer_gps_ok": False,
    "peer_status_ok": False,
    "reason": ""
}

dir_ctrl1 = DigitalOutputDevice(DIR_PIN1)
ena_ctrl1 = DigitalOutputDevice(ENA_PIN1)
step_ctrl1 = PWMOutputDevice(STEP_PIN1, frequency=5000)

dir_ctrl2 = DigitalOutputDevice(DIR_PIN2)
ena_ctrl2 = DigitalOutputDevice(ENA_PIN2)
step_ctrl2 = PWMOutputDevice(STEP_PIN2, frequency=5000)

moving1 = False
start_time1 = 0

moving2 = False
start_time2 = 0

object1 = 0
object2 = 0

pin5_interrupt = 1

def callback1(channel):
    global moving1, object1
    print("Interrupt1", GPIO.input(channel))
    step_ctrl1.value = 0.0
    moving1 = False
    object1 = 0

def callback2(channel):
    global moving1, object1
    print("Interrupt2", GPIO.input(channel))
    step_ctrl1.value = 0.0
    moving1 = False
    object1 = 1

def callback3(channel):
    global moving2, object2
    print("Interrupt3", GPIO.input(channel))
    step_ctrl2.value = 0.0
    moving2 = False
    object2 = 0

def callback4(channel):
    global moving2, object2
    print("Interrupt4", GPIO.input(channel))
    step_ctrl2.value = 0.0
    moving2 = False
    object2 = 1
    
def callback5(channel):
    print("Interrupt5", GPIO.input(channel))
    if GPIO.input(PIN2) == 1:
        dir_ctrl1.off()
        step_ctrl1.value = 0.5
    if GPIO.input(PIN4) == 1:
        dir_ctrl2.off()
        step_ctrl2.value = 0.5

def start_motor1(on):
    global moving1, start_time1

    if on == True and GPIO.input(PIN1) == 1:
        dir_ctrl1.on()
        step_ctrl1.value = 0.5
    
    if on == False and GPIO.input(PIN2) == 1:
        dir_ctrl1.off()
        step_ctrl1.value = 0.5

    moving1 = True
    start_time1 = time.monotonic()

def start_motor2(on):
    global moving2, start_time2

    if on == True and GPIO.input(PIN3) == 1:
        dir_ctrl2.on()
        step_ctrl2.value = 0.5
    
    if on == False and GPIO.input(PIN4) == 1:
        dir_ctrl2.off()
        step_ctrl2.value = 0.5

    moving2 = True
    start_time2 = time.monotonic()

def keyboard_thread():
    while True:
        print("Enter to start")
        input()

        if GPIO.input(PIN1) == 0:
            start_motor1(False)

        if GPIO.input(PIN3) == 0:
            start_motor2(False)
        
        if GPIO.input(PIN2) == 0:
            start_motor1(True)

        if GPIO.input(PIN4) == 0:
            start_motor2(True)

GPIO.add_event_detect(
    PIN1,
    GPIO.FALLING,
    callback=callback1,
    bouncetime=50
)

GPIO.add_event_detect(
    PIN2,
    GPIO.FALLING,
    callback=callback2,
    bouncetime=50
)

GPIO.add_event_detect(
    PIN3,
    GPIO.FALLING,
    callback=callback3,
    bouncetime=50
)

GPIO.add_event_detect(
    PIN4,
    GPIO.FALLING,
    callback=callback4,
    bouncetime=50
)

def ros2_thread():
    rclpy.init()
    publisher = MinimalPublisher(get_objects_callback=lambda: (object1, object2))
    subscriber = MinimalSubscriber(msg_queue=msg_queue)
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(publisher)
    executor.add_node(subscriber)
    
    try:
        executor.spin()
    except Exception as e:
        print(f"ROS 2 Thread Exception: {e}")
    finally:
        executor.shutdown()
        publisher.destroy_node()
        subscriber.destroy_node()
        rclpy.shutdown()

threading.Thread(target=ros2_thread, daemon=True).start()
threading.Thread(target=keyboard_thread, daemon=True).start()

def read_status(msg):
    uav_link_status["interface_version"] = msg.interface_version
    uav_link_status["self_id"] = msg.self_id
    uav_link_status["peer_id"] = msg.peer_id
    uav_link_status["stamp"] = msg.stamp
    uav_link_status["seq"] = msg.seq
    uav_link_status["role"] = msg.role
    uav_link_status["status"] = msg.status
    uav_link_status["online"] = msg.online
    uav_link_status["gps_ok"] = msg.gps_ok
    uav_link_status["peer_seen"] = msg.peer_seen
    uav_link_status["peer_gps_ok"] = msg.peer_gps_ok
    uav_link_status["peer_status_ok"] = msg.peer_status_ok
    uav_link_status["reason"] = msg.reason
    
    return uav_link_status

try:
    ena_ctrl1.off()
    ena_ctrl2.off()

    # # dir_ctrl1.off()
    # dir_ctrl2.off()
    # # step_ctrl1.value = 0.5
    # step_ctrl2.value = 0.5

    # time.sleep(6)

    # # ena_ctrl1.on()
    # ena_ctrl2.on()
    # # dir_ctrl1.off()
    # dir_ctrl2.off()
    # # step_ctrl1.value = 0.0
    # step_ctrl2.value = 0.0

    if GPIO.input(PIN1) == 1:
        dir_ctrl1.on()
        step_ctrl1.value = 0.5

    if GPIO.input(PIN3) == 1:
        dir_ctrl2.on()
        step_ctrl2.value = 0.5

    while True:
        # print("status: ", GPIO.input(PIN5))
        # print("PIN1: ", GPIO.input(PIN1))
        # print("PIN3: ", GPIO.input(PIN3))
        if moving1:
            if time.monotonic() - start_time1 > TIMEOUT:
                print("Motor1 timeout!")
                step_ctrl1.value = 0.0
                ena_ctrl1.on()
                moving1 = False

        if moving2:
            if time.monotonic() - start_time2 > TIMEOUT:
                print("Motor2 timeout!")
                step_ctrl2.value = 0.0
                ena_ctrl2.on()
                moving2 = False

        if uav_link_status["status"] == LinkStatus.UAV_WANNA_TAKEOFF and GPIO.input(PIN2) == 0:
            if pin5_interrupt == 1:
                GPIO.remove_event_detect(PIN5)

            pin5_interrupt = 0
            start_motor1(True)

        if uav_link_status["status"] == LinkStatus.UAV_WANNA_TAKEOFF and GPIO.input(PIN4) == 0:
            if pin5_interrupt == 1:
                GPIO.remove_event_detect(PIN5)
            
            pin5_interrupt = 0
            start_motor2(True)

        if uav_link_status["status"] == LinkStatus.UAV_APPROACHING and GPIO.input(PIN1) == 0:
            if pin5_interrupt == 1:
                GPIO.remove_event_detect(PIN5)
            
            pin5_interrupt = 0
            start_motor1(False)

        if uav_link_status["status"] == LinkStatus.UAV_APPROACHING and GPIO.input(PIN3) == 0:
            if pin5_interrupt == 1:
                GPIO.remove_event_detect(PIN5)
                
            pin5_interrupt = 0
            start_motor2(False)

        if uav_link_status["status"] == LinkStatus.UAV_DISARMED and GPIO.input(PIN1) == 0 and GPIO.input(PIN3) == 0:
            if pin5_interrupt == 0:
                GPIO.add_event_detect(
                    PIN5,
                    GPIO.RISING,
                    callback=callback5,
                    bouncetime=50
                )

            pin5_interrupt = 1

        while not msg_queue.empty():
            try:
                msg = msg_queue.get_nowait()
                uav_link_status = read_status(msg)
            except queue.Empty:
                break

        sleep(0.2)
        
finally:
    step_ctrl1.value = 0.0
    step_ctrl2.value = 0.0
    ena_ctrl1.on()
    ena_ctrl2.on()