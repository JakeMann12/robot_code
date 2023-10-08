from math import sin, cos
from pylx16a.lx16a import *
import time

LX16A.initialize("/dev/ttyUSB0")

try:
    servo1 = LX16A(1)#knee
    servo3 = LX16A(3)#hip2
    servo4 = LX16A(4)#knee
    servo2 = LX16A(2)#hip2
    servo5 = LX16A(5)#knee
    servo6 = LX16A(6)#hip
    servo1.set_angle_offset(-30.00)
    servo1.set_angle_limits(0, 180)
    servo2.set_angle_limits(30, 150)
    servo3.set_angle_limits(40, 160)
    
    servo4.set_angle_limits(20, 220)
    servo5.set_angle_limits(20, 158)
    servo6.set_angle_limits(100, 230)
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
while True:
    try: 
        servo4.move(sin(0.5*t) * 90 + 90)
        servo5.move(sin(0.5*t) * 60 + 90)
        #servo6.move(sin(0.5*t) * 55 + 100)
        time.sleep(0.05)
        t += 0.1
    except ServoError as e:
        print(f"Servo {e.id_} is not responding. Exiting...")
        quit()
    
