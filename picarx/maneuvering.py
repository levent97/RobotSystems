from picarx_improved import Picarx
from time import sleep

if __name__ == "__main__":
    
    px = Picarx()

    while True:
    
        choice = input('Choose an action to take: (1 - Move, 2 - Parallel Park, 3 - K-turn)')
        if choice == '1':
            print('Moving forward...')
            px.move(50,2,0)
        elif choice == '2':
            print('Parallel parking...')
            px.p_park(75, 1.75,-1)
        elif choice == '3':
            print('3-point turning...')
            px.k_turn(75,2.25,-1)
        else:
            print('What?')
            px.stop()
            break

    px.set_dir_servo_angle(0)
    px.set_cam_tilt_angle(0)
    px.set_cam_pan_angle(0)  
      
    px.stop()
    sleep(.2)
