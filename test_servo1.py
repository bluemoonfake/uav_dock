from time import sleep

from gpiozero import DigitalOutputDevice, PWMOutputDevice

MOTOR_PINS = (
    (20, 21, 19),
    (17, 27, 18),
    (5, 7, 12),
    (16, 26, 13),
)
STEP_FREQUENCY = 800


def main():
    motors = []
    devices = []
    right = 1

    try:
        for dir_pin, ena_pin, step_pin in MOTOR_PINS:
            ena_ctrl = DigitalOutputDevice(ena_pin, initial_value=True)
            devices.append(ena_ctrl)
            dir_ctrl = DigitalOutputDevice(dir_pin)
            devices.append(dir_ctrl)
            step_ctrl = PWMOutputDevice(
                step_pin, frequency=STEP_FREQUENCY, initial_value=0
            )
            devices.append(step_ctrl)
            motors.append((dir_ctrl, ena_ctrl, step_ctrl))

        while True:
            input("Nhan Enter de chay/dao chieu ca 4 motor; Ctrl+C de dung: ")

            for _, _, step_ctrl in motors:
                step_ctrl.value = 0
            sleep(0.01)

            right = 1 - right
            for dir_ctrl, ena_ctrl, _ in motors:
                dir_ctrl.value = right
                ena_ctrl.off()
            sleep(0.01)

            for _, _, step_ctrl in motors:
                step_ctrl.value = 0.5
            print("Ca 4 motor dang chay, right:", right)

    except (KeyboardInterrupt, EOFError):
        print("\nDung ca 4 motor.")
    finally:
        for _, _, step_ctrl in motors:
            step_ctrl.value = 0
        for _, ena_ctrl, _ in motors:
            ena_ctrl.on()
        for device in reversed(devices):
            device.close()

if __name__ == "__main__":
    main()
