from math import sin, cos
from pylx16a.lx16a import *
import time

LX16A.initialize("/dev/ttyUSB0")

try:
    servo1 = LX16A(1)#knee
    servo3 = LX16A(3)#hip
    servo2 = LX16A(2)#hip
    servo4 = LX16A(4)#knee
    servo5 = LX16A(5)#hip
    servo6 = LX16A(6)#hip
    servo1.set_angle_offset(-30.00)
    servo1.set_angle_limits(0, 180)
    servo2.set_angle_limits(30, 150)
    servo3.set_angle_limits(25, 160)
    servo4.set_angle_limits(20, 220)
    servo5.set_angle_limits(20, 178)
    servo6.set_angle_limits(100, 230)
    
    print("moving")
    # servo1.move(120)
    servo1.move(98)
    servo2.move(30.0)
    servo3.move(120)
    # servo4.move(120)
    servo4.move(140)
    servo5.move(140)
    servo6.move(130)
    
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
while True:
    #motor 4: 138 to 115, (move to 138 first)
    print(i)
    factor = 0.65
    try:
        for x in range(1000):
            #lower hips
            servo2.move(7.1*cos(factor*i+1.5708)+90.48) #higher = forward
            servo5.move(7*cos(factor*i+1.487)+105.5) #lower = forward
            #knees
            servo1.move(7*cos(factor*i+0.07)+71.03)
            servo4.move(12*cos(factor*i-2.05)+142)
            #hips
            servo3.move(5*cos(factor*i-1.41)+53)
            servo6.move(5*cos(factor*i-1.43)+138)
            i+= 0.1
    except ServoError as e:
        print(f"Servo {e.id_} is not responding. Exiting...")
        quit()
    
