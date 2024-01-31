#!/usr/bin/env python

import rospy
from gant.msg import gant
import socket
import time
import sys

  
#Bus-Verbindung herstellen
#Establish bus connection
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
except socket.error:
    print ('failed to create sockt')
    
s.connect(("169.254.184.18", 502))
print ('Socket created')
#Wird beim Ausfuehren des Programms nur der Speicherort und der Programmname in der Shell angezeigt, so sind die IP Adressen des Programms und der dryve D1 nicht uebereinstimmend
#When executing the program and the shell displays the storing folder and the program name, the set IP address in the program and the dryve D1 doesn't match



#Durchlauf State Machine (Handbuch: Visualisieung State Machine) 
#State Machine pass through (Manual: Visualisation State Machine)
    

# Statusword 6041h
# Status request
status = [0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2]
status_array = bytearray(status)
print(status_array)

# Controlword 6040h
# Command: Shutdown
shutdown = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 6, 0]
shutdown_array = bytearray(shutdown)
print(shutdown_array)

# Controlword 6040h
# Command: Switch on
switchOn = [0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 7, 0]
switchOn_array = bytearray(switchOn)
print(switchOn_array)

# Controlword 6040h
# Command: enable Operation
enableOperation = [0, 0, 0, 0, 0, 15, 0, 43,13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 15, 0]
enableOperation_array = bytearray(enableOperation)
print(enableOperation_array)

# Controlword 6040h
# Command: reset dryve
reset = [0, 0, 0, 0, 0, 15, 0, 43,13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 0, 1]
reset_array = bytearray(reset)
print(reset_array)


#Definition der Funktion zum Senden und Empfangen von Daten
#Definition of the function to send and receive data 
def sendCommand(data):
    #Socket erzeugen und Telegram senden
    #Create socket and send request
    s.send(data)
    res = s.recv(24)
    #Ausgabe Antworttelegram 
    #Print response telegram
    print(list(res))
    return list(res)

#Shutdown Controlword senden und auf das folgende Statuswort pruefen. Pruefung auf mehrer Statuswords da mehrere Szenarien siehe Bit assignment Statusword, data package im Handbuch 
#sending Shutdown Controlword and check the following Statusword. Checking several Statuswords because of various options. look at Bit assignment Statusword, data package in user manual 
def set_shdn():
    sendCommand(shutdown_array)
    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 6]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 22]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 33, 2]):
        print("wait for shdn")

        #1 Sekunde Verzoegerung
        #1 second delay
        time.sleep(1)

#Switch on Disabled Controlword senden und auf das folgende Statuswort pruefen. Pruefung auf mehrer Statuswords da mehrere Szenarien siehe Bit assignment Statusword, data package im Handbuch 
#sending Switch on Disabled Controlword and check the following Statusword. Checking several Statuswords because of various options. look at Bit assignment Statusword, data package in user manual 
def set_swon():
    sendCommand(switchOn_array)
    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 6]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 22]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 35, 2]):
        print("wait for sw on")

        #1 Sekunde Verzoegerung
        #1 second delay
        time.sleep(1)

#Operation Enable Controlword senden und auf das folgende Statuswort pruefen. Pruefung auf mehrer Statuswords da mehrere Szenarien siehe Bit assignment Statusword, data package im Handbuch 
#Operation Enable Controlword and check the following Statusword. Checking several Statuswords because of various options. look at Bit assignment Statusword, data package in user manual 
def set_op_en():
    sendCommand(enableOperation_array)
    while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 6]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]
           and sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 2]):
        print("wait for op en")

        #1 Sekunde Verzoegerung
        #1 second delay
        time.sleep(1)

def init():

    #Aufruf der Funktion sendCommand zum hochfahren der State Machine mit vorher definierten Telegrammen (Handbuch: Visualisieung State Machine)
    #Call of the function sendCommand to start the State Machine with the previously defined telegrams (Manual: Visualisation State Machine)
    sendCommand(reset_array)
    set_shdn()
    set_swon()
    set_op_en()


def set_mode(mode):

    #Setzen der Operationsmodi im Objekt 6060h Modes of Operation
    #Set operation modes in object 6060h Modes of Operation
    sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 96, 0, 0, 0, 0, 1, mode]))
    while (sendCommand(bytearray([0, 0, 0, 0, 0, 13, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1])) != [0, 0, 0, 0, 0, 14, 0, 43, 13, 0, 0, 0, 96, 97, 0, 0, 0, 0, 1, mode]):

        print("wait for mode")

        #1 Sekunde Verzoegerung
        #1 second delay
        time.sleep(1)


init()



rospy.init_node('gantryy_move',anonymous=True)
gant_pub = rospy.Publisher('gantry_movement',gant, queue_size=10)  



#Parametrierung der Objekte gemäß Handbuch
#Parameterization of the objects according to the manual

# 6060h Modes of Operation
#Setzen auf Homing Modus (see "def set_mode(mode):"; Byte 19 = 6)
#Set Homing mode (see "def set_mode(mode):"; Byte 19 = 6)
set_mode(6)

# 6092h_01h Feed constant Subindex 1 (Feed)
#Setzen des Vorschubs auf den Wert 6000; vgl. Handbuch  (Byte 19 = 112; Byte 20= 23)
#Set feed constant to 6000; refer to manual (Byte 19 = 112; Byte 20= 23)
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 146, 1, 0, 0, 0, 2, 112, 23]))

# 6092h_02h Feed constant Subindex 2 (Shaft revolutions)
#Setzen der Wellenumdrehung auf 1; vgl. Handbuch (Byte 19 = 1)
#Set shaft revolutions to 1; refer to manual (Byte 19 = 1)
sendCommand(bytearray([0, 0, 0, 0, 0, 14, 0, 43, 13, 1, 0, 0, 96, 146, 2, 0, 0, 0, 1, 1]))

# 6099h_01h Homing speeds Switch
#Vorgabe der Verfahrgeschwindigkeit beim Suchen auf den Schalter wird auf 60 U/min gesetzt (Byte 19 = 112; Byte 20 = 23))
#Speed during search for switch is set to 60 rpm (Byte 19 = 112; Byte 20 = 23))
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 153, 1, 0, 0, 0, 2, 208, 15]))   #more than 20rpm less than 60

# 6099h_02h Homing speeds Zero
#Setzen Verfahrgeschwindigkeit beim Suchen von Null auf 60 U/min (Byte 19 = 112; Byte 20 = 23))
#Set speed during Search for zero to 60 rpm (Byte 19 = 112; Byte 20 = 23))
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 153, 2, 0, 0, 0, 2, 208, 15]))  #more than 20rpm less than 60

# 609Ah Homing acceleration
#Setzen der Refernzfahrt-Beschleunigung wird auf 500 U/min² (Byte 19 = 80; Byte 20 = 195)
#Set Homing acceleration to 500 rpm/min² (Byte 19 = 80; Byte 20 = 195)
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 154, 0, 0, 0, 0, 2, 80, 195]))

# 6040h Controlword
#Start Homing
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 31, 0]))

#Check Statusword nach Referenziert Signal
#Check Statusword for signal referenced 
while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
      
        print("wait for Homing to end")
        
        #1 Sekunde Verzoegerung
        #1 second delay 
        time.sleep(1)

sendCommand(enableOperation_array)

# 6060h Modes of Operation
#Setzen auf Profile Position Mode (see "def set_mode(mode):"; Byte 19 = 1)
#Set Profile Position Mode (see "def set_mode(mode):"; Byte 19 = 1)
set_mode(1)

v=60
a=250
Velocity = v*100
Acceleration = a*100
intv = int(Velocity)
inta = int(Acceleration)
hex_v = hex(intv)
hex_a = hex(inta)
hex_v = hex_v[2:].zfill(8)
hex_a = hex_a[2:].zfill(8)
  
bytev_1 = int(hex_v[6:8], 16)
bytev_2 = int(hex_v[4:6], 16)
bytev_3 = int(hex_v[2:4], 16)
bytev_4 = int(hex_v[0:2], 16)
  
bytea_1 = int(hex_a[6:8], 16)
bytea_2 = int(hex_a[4:6], 16)
bytea_3 = int(hex_a[2:4], 16)
bytea_4 = int(hex_a[0:2], 16)



# 6081h Profile Velocity
#Setzen der Geschwindigkeit auf 60 U/min (Byte 19 = 112; Byte 20 = 23)
#Set velocity to 60 rpm (Byte 19 = 112; Byte 20 = 23)
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 129, 0, 0, 0, 0, 2, bytev_1, bytev_2]))

# 6083h Profile Acceleration
#Setzen der Beschleunigung auf 500 U/min² (Byte 19 = 80; Byte 20 = 195)
#Set acceleration to 500 rpm/min² (Byte 19 = 80; Byte 20 = 195)
sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 131, 0, 0, 0, 0, 2, bytea_1, bytea_2]))



flag = gant()    
flag.f=4000
gant_pub.publish(flag) 

def gantry_callback(data):
 
 if data.f == 4:
 

  global setPosition0, setPosition1, setPosition2, setPosition3
  #rospy.loginfo("decision_msg:(%2f,%2f,%2f,%2f)",data.x,data.y,data.z,data.f)
    
  #data.z=30000
  zr=data.z
  data.z=data.z*87.5   #85.7 is for tranfering to motor rotation
  
  intz = int(data.z)
  hex_z = hex(intz)
    
  hex_z = hex_z[2:].zfill(8)  # Removing '0x' and padding zeros to make it 8 characters

  byte_1 = int(hex_z[6:8], 16)
  byte_2 = int(hex_z[4:6], 16)
  byte_3 = int(hex_z[2:4], 16)
  byte_4 = int(hex_z[0:2], 16)


  #print(f"setPosition0: {byte_1}")  
  #print(f"setPosition1: {byte_2}")
  #print(f"setPosition2: {byte_3}")
  #print(f"setPosition3: {byte_4}")

  
 
  setPosition0 = byte_1    
  setPosition1 = byte_2    
  setPosition2 = byte_3     
  setPosition3 = byte_4
  print(f"setPosition0: {setPosition0}")  
  print(f"setPosition1: {setPosition1}")
  print(f"setPosition2: {setPosition2}")
  print(f"setPosition3: {setPosition3}")    
          
#Schleife Links- und Rechtslauf des Motors
#Clockwise/counter-clockwise motor movement Loop
#while True:

 

    #Ziel-Position nach jeden Schleifendurchlauf zurücksetzen; die Variablen setPositionX werden nach jedem Schleifendurchlauf auf den Anfangswert zurückgesetzt 
    #Reset target position after each loop; the variables setPositionX are rewritten with the default value after each loop  
  sendCommand(bytearray([0, 0, 0, 0, 0, 17, 0, 43, 13, 1, 0, 0, 96, 122, 0, 0, 0, 0, 4, setPosition0, setPosition1,   setPosition2, setPosition3]))
    
    #Startbefehl zur Bewegung des Motors über Bit 4 
    #Set Bit 4 true to excecute the movoment of the motor 
  sendCommand(bytearray([0, 0, 0, 0, 0, 15, 0, 43, 13, 1, 0, 0, 96, 64, 0, 0, 0, 0, 2, 31, 0]))
    
  print("go")
    
    #1 Sekunde Verzoegerung
    #1 second delay 
  time.sleep(1)

    #Check Statusword nach Ziel ereicht
    #Check Statusword for target reached 
  while (sendCommand(status_array) != [0, 0, 0, 0, 0, 15, 0, 43, 13, 0, 0, 0, 96, 65, 0, 0, 0, 0, 2, 39, 22]):
        
      print("wait for next command")
        
        #1 Sekunde Verzoegerung
        #1 second delay 
      time.sleep(1)

  sendCommand(enableOperation_array)
  
  #time.sleep(5)
  
  flag = gant()    
  flag.f=4000+zr
  gant_pub.publish(flag) 

    

if __name__ == '__main__':  

    #rospy.init_node('gantryy_move',anonymous=True)
    gant_sub = rospy.Subscriber("gantry_request",gant,callback=gantry_callback)
   # gant_pub = rospy.Publisher('gantry_movement',gant, queue_size=10)  
    rospy.spin() 


    
    
    
    



