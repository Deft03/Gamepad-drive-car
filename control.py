#!/usr/bin/env python3
# Tran Thanh Sang 20146412
# Thach Phi Sach 20146411
import rospy 
import RPi.GPIO as GPIO
from std_msgs.msg import String 
from std_msgs.msg import Float64

import smbus					#import SMBus module of I2C
from time import sleep   
import math

BUTTON_PIN1 = 21
BUTTON_PIN2 = 20
BUTTON_PIN3 = 19
BUTTON_PIN4 = 16
BUTTON_PIN5 = 6
BUTTON_PIN6 = 12

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


N = 2
m =0
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN1,  GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
GPIO.setup(BUTTON_PIN2, GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
GPIO.setup(BUTTON_PIN3, GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
GPIO.setup(BUTTON_PIN4, GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
GPIO.setup(BUTTON_PIN5, GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 
GPIO.setup(BUTTON_PIN6, GPIO.IN , pull_up_down = GPIO.PUD_DOWN) 

GPIO.setup(17,GPIO.OUT)
GPIO.setup(27,GPIO.OUT)

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")
GPIO.output(17,0)
if __name__=='__main__':

	rospy.init_node('transmitters')
	# banh phai 
	pub1=rospy.Publisher("/my_diffbot/right_wheel_controller/command",Float64,queue_size=10)
	# banh trai
	pub2=rospy.Publisher("/my_diffbot/left_wheel_controller/command",Float64,queue_size=10)
	rate= rospy.Rate(2)
	while not rospy.is_shutdown():
		msg=String()
		msg.data="Hi,this is " + str(GPIO.input(BUTTON_PIN1))
		
		acc_x = read_raw_data(ACCEL_XOUT_H)
		acc_y = read_raw_data(ACCEL_YOUT_H)
		acc_z = read_raw_data(ACCEL_ZOUT_H)
		
		#Read Gyroscope raw value
		gyro_x = read_raw_data(GYRO_XOUT_H)
		gyro_y = read_raw_data(GYRO_YOUT_H)
		gyro_z = read_raw_data(GYRO_ZOUT_H)
		
		#Full scale range +/- 250 degree/C as per sensitivity scale factor
		Ax = acc_x/16384.0
		Ay = acc_y/16384.0
		Az = acc_z/16384.0
		
		Gx = gyro_x/131.0
		Gy = gyro_y/131.0
		Gz = gyro_z/131.0
		

		print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	


		pitch = math.atan2(Ax,math.sqrt(Ax**2+Az**2))*180/3.14
		roll = math.atan2(Ay,math.sqrt(Ax**2+Az**2))*180/3.14

		if(GPIO.input(BUTTON_PIN6)==1):
			m+=1
			if m>2:
				m=0
		print(m)
		if m ==1:
			if(GPIO.input(BUTTON_PIN5)==1):
				N+=1
				if N == 9 :
					N=3
				print(N)
			GPIO.output(17,0)
			GPIO.output(27,1)
			if(pitch>0 and roll >0):
				# up
				print('up')
				pub1.publish(-N)
				pub2.publish(N)


			elif(pitch>0 and roll <0):
				#left
				print('left')
				pub1.publish(-(N+3))
				pub2.publish(0)

			elif(pitch<0 and roll >0): ##right

				print('right')
				pub1.publish(0)
				pub2.publish(N+3)

			elif(pitch<0 and roll <0):
				# down
				pub1.publish(N)
				pub2.publish(-N)
				print('down')
			sleep(0.5)
		

		elif m==2:
		# Tang tốc
			GPIO.output(17,1)
			GPIO.output(27,0)
			if(GPIO.input(BUTTON_PIN5)==1):
				N+=1
				if N == 9 :
					N=3
				print(N)

			if(GPIO.input(BUTTON_PIN1)==1):##Di thang 
				print('up')
				pub1.publish(N*GPIO.input(BUTTON_PIN1))
				pub2.publish(-N*GPIO.input(BUTTON_PIN1))

			elif(GPIO.input(BUTTON_PIN2)==1):
			# Di lui 
				print('down')
				pub1.publish(-N*GPIO.input(BUTTON_PIN2))
				pub2.publish(N*GPIO.input(BUTTON_PIN2))
			elif(GPIO.input(BUTTON_PIN3)==1 ):	#SANG TRAI
				print('left')
				pub1.publish(-(N*GPIO.input(BUTTON_PIN3)+3))
				pub2.publish(0)
			elif(GPIO.input(BUTTON_PIN4)==1):	#SANG PHAI
				print('right')
				pub1.publish(0)
				pub2.publish((N*GPIO.input(BUTTON_PIN4)+3))
			else :
				pub1.publish(0)
				pub2.publish(0)
			sleep(0.5)
		else:
			GPIO.output(17,0)
			GPIO.output(27,0)
			sleep(0.5)
		rospy.loginfo(msg)
		

	rospy.loginfo("Transmitter was stopped")
