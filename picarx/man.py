from picarx_improved import Picarx
from time import sleep
import readchar

manual = '''
Press keys on keyboard to control PiCar-X!
    q: Forward and left
    w: Forward
    e: Forward and right
    ctrl+c: Press twice to exit the program
'''


if __name__ == "__main__":
    try:
        pan_angle = 0
        tilt_angle = 0
        px = Picarx()
        print(manual)
        px.forward(50)
        while True:
            key = readchar.readkey()
            key = key.lower()
            
            if 'q' == key:
                print('forward and left.')
                px.set_dir_servo_angle(-15)
            elif 'w' == key:
                print('forward.')
                px.set_dir_servo_angle(0)
            elif 'e' == key:
                print('forward and right.')
                px.set_dir_servo_angle(15)
          
            elif key == readchar.key.CTRL_C:
                print("\n Quit")
                break

    finally:
        px.set_cam_tilt_angle(0)
        px.set_cam_pan_angle(0)  
        px.set_dir_servo_angle(0)  
        px.stop()
        sleep(.2)


