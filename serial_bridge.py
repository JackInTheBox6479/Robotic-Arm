import pygame, serial, time

try:
    ser = serial.Serial('COM6', 115200, timeout=0)
    time.sleep(2)
except Exception as e:
    print(f"Could not open serial port: {e}")
    exit()

pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()

RJoystick = 9
LJoystick = 8

last_base = "Base_Stop"
last_shoulder = "Shoulder_Stop"

last_send_time = 0

try:
    while True:
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit()
            
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == LJoystick:
                    ser.write(b'Set_Zero\n')
                
                if event.button == RJoystick:
                    ser.write(b'Zero\n')

        rx = joystick.get_axis(2) 
        ly = joystick.get_axis(1)

        if rx < -0.5: current_base = "Left"
        elif rx > 0.5: current_base = "Right"
        else: current_base = "Horizontal_Stop"

        if ly < -0.5: current_shoulder = "Up"
        elif ly > 0.5: current_shoulder = "Down"
        else: current_shoulder = "Vertical_Stop"

        if current_base != last_base:
            ser.write(f"{current_base}\n".encode())
            last_base = current_base

        if current_shoulder != last_shoulder:
            ser.write(f"{current_shoulder}\n".encode())
            last_shoulder = current_shoulder

        current_time = time.time()
        if current_time - last_send_time > 0.1:
            if current_base != "Horizontal_Stop":
                ser.write(f"{current_base}\n".encode())
    
            if current_shoulder != "Vertical_Stop":
                ser.write(f"{current_shoulder}\n".encode())
    
            ser.flush()
            last_send_time = current_time
        time.sleep(0.01) 

except KeyboardInterrupt:
    ser.close()
    pygame.quit()