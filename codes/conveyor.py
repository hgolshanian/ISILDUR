#!/usr/bin/env python

import minimalmodbus
import serial
import sys
import rospy
from gant.msg import gant
import time

port = "/dev/ttyUSB1"
slave_id = 1

try:
    instrument = minimalmodbus.Instrument(port=port, slaveaddress=slave_id)
    print("connected!")
    instrument.serial.baudrate = 38400
    instrument.serial.parity = serial.PARITY_NONE
    instrument.serial.stopbits=1

except:
    print("could not connect!")
    sys.exit(1)
    

rospy.init_node('conveyor_move',anonymous=True)
conveyor_pub = rospy.Publisher('gantry_movement',gant, queue_size=10)     


rospy.sleep(0.1)  #this is mandatory to publish
flag = gant()    
flag.f=50
conveyor_pub.publish(flag)  

print("ready")
    
def conveyor_callback(data):
  max_retries = 3  
  for attempt in range(max_retries):
    try:
        if data.f == 5:
            if data.x == 0 and data.y == 0 and data.d == 0 and data.z == 0 and data.o == 0:
                print("Run!")
                value_to_write = 1
                print("Writing value:", value_to_write, "to register 0")
                instrument.write_register(0, value_to_write)
                rospy.sleep(0.1)

                flag = gant()
                flag.f = 51
                conveyor_pub.publish(flag)
                break

            else:
                print("Stop!")
                value_to_write = 0
                print("Writing value:", value_to_write, "to register 0")
                instrument.write_register(0, value_to_write)
                print("x =", data.x)
                print("y =", data.y)
                print("depth =", data.d)
                print("class-ID =", data.z)
                print("orientation =", data.o)
                rospy.sleep(0.1)

                flag = gant()
                flag.f = 50
                conveyor_pub.publish(flag)
                break

    except (minimalmodbus.ModbusException, minimalmodbus.NoResponseError) as e:
        print(f"Attempt {attempt + 1} failed: {e}")
        rospy.sleep(0.5)

  else:
     print("All attempts failed. Could not write to register.") 

if __name__ == '__main__': 
     value_to_write = 105
     instrument.write_register(1, value_to_write)
     print("Value", value_to_write, "written to register 1") 
     conveyor_sub = rospy.Subscriber("gantry_request",gant,callback=conveyor_callback)      
     rospy.spin() #if you recieved data do this loop we donot have this in publisher
   
