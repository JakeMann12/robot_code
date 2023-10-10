from math import sin, cos
from pylx16a.lx16a import *
import time

def stand(servo1,servo2,servo3,servo4,servo5,servo6):
    servo1.move(32.88)
    servo2.move(94.32)
    servo3.move(50)
    servo4.move(147)
    servo5.move(98.16)
    servo6.move(136)
    
    
def do_the_nod_backwards(servo1,servo2,servo3,servo4,servo5,servo6):
    i =0
    while True:
        #motor 4: 138 to 115, (move to 138 first)
        factor = -0.35
        try:
            #slow shuffle
            for x in range(600):
                #lower hips
                servo2.move(7.1*cos(factor*i+1.5708)+75.48) #higher = forward
                servo5.move(7*cos(factor*i+1.487)+105.5) #lower = forward
                #knees
                servo1.move(4*cos(factor*i+0.07)+55.03)
                servo4.move(4*cos(factor*i-2.05)+142)
                #hips
                servo3.move(8*cos(factor*i-1.41)+53)
                servo6.move(8*cos(factor*i-1.43)+138)
                i+= 0.1
            stand(servo1,servo2,servo3,servo4,servo5,servo6)
            break
            print("done with move 1")
        except ServoError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()
def do_the_nod(servo1,servo2,servo3,servo4,servo5,servo6):
    i =0
    while True:
        #motor 4: 138 to 115, (move to 138 first)
        factor = 0.35
        try:
            #slow shuffle
            for x in range(600):
                #lower hips
                servo2.move(7.1*cos(factor*i+1.5708)+75.48) #higher = forward
                servo5.move(7*cos(factor*i+1.487)+105.5) #lower = forward
                #knees
                servo1.move(4*cos(factor*i+0.07)+55.03)
                servo4.move(4*cos(factor*i-2.05)+142)
                #hips
                servo3.move(8*cos(factor*i-1.41)+53)
                servo6.move(8*cos(factor*i-1.43)+138)
                i+= 0.1
            stand(servo1,servo2,servo3,servo4,servo5,servo6)
            break
            print("done with move 1")
        except ServoError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()
def do_the_shake(servo1,servo2,servo3,servo4,servo5,servo6):
    i =0
    while True:
        #motor 4: 138 to 115, (move to 138 first)
        factor = 0.45
        try:
            #slow shuffle
            for x in range(1000):
                #lower hips
                servo2.move(5.1*cos(factor*i+1.5708)+75.48) #higher = forward
                servo5.move(5*cos(factor*i+1.487)+105.5) #lower = forward
                # #knees
                # servo1.move(7*cos(factor*i+0.07)+55.03)
                # servo4.move(7*cos(factor*i-2.05)+142)
                # #hips
                # servo3.move(2*cos(factor*i-1.41)+53)
                # servo6.move(2*cos(factor*i-1.43)+138)
                i+= 0.1
            stand(servo1,servo2,servo3,servo4,servo5,servo6)
            break
            print("done with move 1")
        except ServoError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()
def walk_backwards(servo1,servo2,servo3,servo4,servo5,servo6):
    i =0
    while True:
        #motor 4: 138 to 115, (move to 138 first)
        factor = 0.4
        try:
            #slow shuffle
            for x in range(500):
                #lower hips
                servo2.move(4.1*cos(factor*i+1.5708)+69.48) #higher = forward
                servo5.move(4*cos(factor*i+1.487)+105.5) #lower = forward
                # #knees
                servo1.move(2*cos(factor*i+0.07)+55.03)
                servo4.move(2*cos(factor*i-2.05)+142)
                # #hips
                # servo3.move(2*cos(factor*i-1.41)+53)
                # servo6.move(2*cos(factor*i-1.43)+138)
                i+= 0.1
            break
            print("done with move 1")
        except ServoError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()
def take_a_bow(servo1,servo2,servo3,servo4,servo5,servo6):
    stand(servo1,servo2,servo3,servo4,servo5,servo6)
    factor = 0.05
    i=0
    while True:
        try:
            for x in range(500):
                servo1.move(26.64*sin(factor*i-1.57)+59.52)
                servo2.move(5.16*sin(factor*i-1.57)+99.48)
                servo4.move(19.38*sin(factor*i-4.71)+127.62)
                servo5.move(18.84*sin(factor*i-4.71)+79.32)
                i +=0.1
            # stand(servo1,servo2,servo3,servo4,servo5,servo6)
            break
        except ServoError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()
def do_the_jerk(servo1,servo2,servo3,servo4,servo5,servo6):
    stand(servo1,servo2,servo3,servo4,servo5,servo6)
    factor = 0.25
    i=0
    while True:
        try:
            for x in range(1500):
                servo1.move(3.64*sin(factor*i-1.57)+59.52)
                servo2.move(3.16*sin(factor*i-1.57)+89.48)
                servo4.move(3.38*sin(factor*i-4.71)+127.62)
                servo5.move(3.84*sin(factor*i-4.71)+89.32)
                i +=0.1
            # stand(servo1,servo2,servo3,servo4,servo5,servo6)
            break
        except ServoError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()
    
def wag_tail(servo1,servo2,servo3,servo4,servo5,servo6, servo7):
    stand(servo1,servo2,servo3,servo4,servo5,servo6)
    factor = 0.25
    i=0
    while True:
        try:
            for x in range(1500):
                servo7.move(70*sin(factor*i-1.57)+70)
                i +=0.1
            # stand(servo1,servo2,servo3,servo4,servo5,servo6)
            break
        except ServoError as e:
            print(f"Servo {e.id_} is not responding. Exiting...")
            quit()
    
    
def run_robot():

    LX16A.initialize("COM5")

    try:
        servo1 = LX16A(1)#knee
        servo3 = LX16A(3)#hip
        servo2 = LX16A(2)#hip
        servo4 = LX16A(4)#knee
        servo5 = LX16A(5)#hip
        servo6 = LX16A(6)#hip
        # servo1.set_angle_offset(-30.00)
        servo1.set_angle_limits(0, 210)
        servo2.set_angle_limits(30, 150)
        servo3.set_angle_limits(25, 160)
        servo4.set_angle_limits(20, 220)
        servo5.set_angle_limits(20, 178)
        servo6.set_angle_limits(100, 230)
        
        
        print("moving")
        stand(servo1,servo2,servo3,servo4,servo5,servo6)
        # # servo1.move(120)
        # servo1.move(98)
        # servo2.move(30.0)
        # servo3.move(120)
        # # servo4.move(120)
        # servo4.move(140)
        # servo5.move(140)
        # servo6.move(130)
        
    except ServoTimeoutError as e:
        print(f"Servo {e.id_} is not responding. Timeout or disconnect error. Exiting...")
        quit()

    except ServoChecksumError as e:
        print(f"Servo {e.id_} is not responding. Bad Checksum. Exiting...")
        quit()
        
    except ServoArgumentError as e:
        print(f"Servo {e.id_} is not responding. Bad Argument. Exiting...")
        quit()

    except ServoLogicalError as e:
        print(f"Servo {e.id_} is not responding. Bad Argument. Exiting...")
        quit()

    t = 0

    i=0
    stand(servo1,servo2,servo3,servo4,servo5,servo6)
    do_the_nod_backwards(servo1,servo2,servo3,servo4,servo5,servo6)
    # do_the_nod(servo1,servo2,servo3,servo4,servo5,servo6)
    # stand(servo1,servo2,servo3,servo4,servo5,servo6)
    # walk_backwards(servo1,servo2,servo3,servo4,servo5,servo6)
    # do_the_shake(servo1,servo2,servo3,servo4,servo5,servo6)
    # walk_backwards(servo1,servo2,servo3,servo4,servo5,servo6)
    # stand(servo1,servo2,servo3,servo4,servo5,servo6)
    # do_the_nod(servo1,servo2,servo3,servo4,servo5,servo6)
    # walk_backwards(servo1,servo2,servo3,servo4,servo5,servo6)
    # do_the_shake(servo1,servo2,servo3,servo4,servo5,servo6)
    # stand(servo1,servo2,servo3,servo4,servo5,servo6)
    # do_the_nod(servo1,servo2,servo3,servo4,servo5,servo6)
    # walk_backwards(servo1,servo2,servo3,servo4,servo5,servo6)
    # do_the_shake(servo1,servo2,servo3,servo4,servo5,servo6)
    # stand(servo1,servo2,servo3,servo4,servo5,servo6)
    # do_the_nod(servo1,servo2,servo3,servo4,servo5,servo6)
    # do_the_nod_backwards(servo1,servo2,servo3,servo4,servo5,servo6)
    # stand(servo1,servo2,servo3,servo4,servo5,servo6)
    # take_a_bow(servo1,servo2,servo3,servo4,servo5,servo6)
    do_the_jerk(servo1,servo2,servo3,servo4,servo5,servo6)
run_robot()