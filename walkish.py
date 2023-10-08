from math import sin, cos
from pylx16a.lx16a import *

LX16A.initialize("COM5")

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

servo1.move(32.88)
servo2.move(94.32)
servo3.move(50)
servo4.move(147)
servo5.move(98.16)
servo6.move(136)

        
i =0
while True:
    #motor 4: 138 to 115, (move to 138 first)
    factor = 0.45
    try:
        #slow shuffle
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
    except ServoError as e:
        print(f"Servo {e.id_} is not responding. Exiting...")
        quit()
