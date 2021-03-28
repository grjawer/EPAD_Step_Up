import smbus2
import time
import struct
import numpy as np
import VL53L0X
import csv

# Create a VL53L0X object
tof = VL53L0X.VL53L0X()
tof.start_ranging(VL53L0X.VL53L0X_HIGH_SPEED_MODE)


addr = 0x28 # for BNO055
conn = smbus2.SMBus(1)
time.sleep(.5)
print("Hello World")
# Reset BNO055 chip
conn.write_byte(addr,0x3F)
time.sleep(.5)
response = conn.read_byte(addr)
reply = conn.read_i2c_block_data(addr, 0x00,7)
##print("chip ID: ",reply[0])
##print("Accelerometer: ",reply[1])
##print("Gyroscope: ",reply[2])
##print("Magnetometer ",reply[3])
##print("Firmware version V",reply[4],".",reply[5])
##print("Bootloader version V",reply[6])

# Set to page 1
conn.write_byte_data(addr,0x07,0x01)
# Get into config mode
conn.write_byte(addr,0x00)
# Set to page 0
conn.write_byte_data(addr,0x07,0x00)
time.sleep(0.014)
# Set NDOF mode
conn.write_byte_data(addr,0x3D,0x0C)
time.sleep(0.00024)
#print(When ready, enter 1")
time.sleep(1)
    
#Orient sensors
print("Let's orient the sensor before we take measurements. Please make sure the sensor is flat, and keep turning it until you see [1,0,0,0].")
euler=[0,0,0] #ideal euler
quaternion=[1,0,0,0] #ideal quaternion
accurate = 0
while (accurate!= 4):
    accurate = 0
    quat =  struct.unpack('<4h',bytearray(conn.read_i2c_block_data(addr, 0x20,8)))#unpacks 2 bytes for each quaternion, 8 bytes total
    euler = struct.unpack('<3h',bytearray(conn.read_i2c_block_data(addr, 0x1A,6)))#same process for euler angles
    quat1 = [number/16384. for number in quat]#makes list of 4 quaternion values (0 to 1)
    euler1 = [number/16. for number in euler]#makes list of 3 euler values (0 to 360)
    #makes sure each of the 4 quaternions is withion 0.1 of our ideal orientation 
    for i,j in zip(quaternion, quat1): 
        if abs(i-abs(j)) < 0.1:
            accurate += 1
    if accurate == 4:
        print("Your sensor is oriented!")
        #print("Quaternions:", quat1, "Eulers:", euler1)
        break
    else:
        print(quat1)

#Initial Orientation
quat = struct.unpack('<4h',bytearray(conn.read_i2c_block_data(addr, 0x20,8)))
quat0 = [number / 16384. for number in quat]
euler = struct.unpack('<3h',bytearray(conn.read_i2c_block_data(addr, 0x1A,6)))
euler0 = [number / 16 for number in euler]
print("Initial orientation:", euler0,"(degrees) and", quat0, "quaternions.")
cosAngle = sum([x*y for x,y in zip(quaternion,quat0)]) #angle between ideal orientation and actual orientation
if (abs(cosAngle)<1):
    angle0 = 2*180*(np.arccos(cosAngle)/np.pi) #angle difference in degrees
    theta0 = angle0*np.pi/180 #angle difference in radians
dist0 = ((tof.get_distance())+0.36)/1.022 #distance measured including calibration of tof for height
dist0_angled = (dist0*cos(theta)) / 1.008 #vertical component at stast


#create arrays for data input
quats = [quat0]
eulers = [euler0]
angles = [angle0]
dists = [dist0]

value = 0 #initialize variable for user input

#start = time.time()

while value != "1":
#for count in range(1,201):
    value = input("Gathering data... Press enter to continue, press 1 to stop.")
    quat =  struct.unpack('<4h',bytearray(conn.read_i2c_block_data(addr, 0x20,8)))
    euler = struct.unpack('<3h',bytearray(conn.read_i2c_block_data(addr, 0x1A,6)))
    quat1 = [number/16384. for number in quat]
    euler1 = [number/16. for number in euler]
    #print(euler1,quat1)
    cosAngle = sum([x*y for x,y in zip(quat0,quat1)])
    if (abs(cosAngle)<1):
        angle = 2*180*(np.arccos(cosAngle)/np.pi) #angle change in degrees
        theta = angle*np.pi/180 #angle change in radians
    else:
        angle = None
        theta = None
    #print("Angle change is",angle)
    
    distance = ((tof.get_distance())+0.36)/1.022 #distance measured including calibration of tof for height
    distance_angled = (distance*cos(theta)) / 1.008
    #takes the measured distance while foot in motion and changes it to vertical distance
    #during trials theta was fixed now theta will be determined with second sensor

    quats.append(quat1)
    eulers.append(euler1)
    angles.append(angle)
    dists.append(distance)

#end=time.time()
#deltaT= [end-start]

# writing to csv file
file_name = "Joint_data_continuous"+ str(time.time()) + ".csv"
with open(file_name, 'w') as csvfile:
    # creating a csv writer object  
    csvwriter = csv.writer(csvfile)  
        
    # writing the fields  
    csvwriter.writerow(quats)
    csvwriter.writerow(eulers)
    csvwriter.writerow(angles)
    csvwriter.writerow(dists)
    #csvwriter.writerow(deltaT)
