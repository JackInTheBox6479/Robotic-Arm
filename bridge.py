import pygame
import serial
import time

try:
    ser = serial.Serial('COM6', 115200, timeout=0)
    time.sleep(2)
except Exception as e:
    print(f"Could not open serial port: {e}")
    exit()

pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

last_command = ""
last_servo_angle = -1 

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                y_axis = joystick.get_axis(2)
                if y_axis < -0.5:
                    current_command = 'B'
                elif y_axis > 0.5:
                    current_command = 'F'
                else:
                    current_command = 'S'

                if current_command != last_command:
                    ser.write(current_command.encode())
                    last_command = current_command
                    print(f"Stepper: {current_command}")

                left_stick_y = joystick.get_axis(1)
                servo_angle = int((left_stick_y + 1) * 90)

                if abs(servo_angle - last_servo_angle) > 2:
                    ser.write(f"V{servo_angle}\n".encode())
                    last_servo_angle = servo_angle
                    print(f"Servo Angle: {servo_angle}")

            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 9:
                    ser.write(b'Z')
                    print("Sent: ZERO")

        time.sleep(0.01)
except KeyboardInterrupt:
    ser.close()
    pygame.quit()