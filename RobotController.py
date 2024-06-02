import serial
import inputs
import time
import math
import sys
from inputs import get_gamepad
from inputs import get_key

from approxeng.input.selectbinder import ControllerResource


import sys
import os
import subprocess
import threading

from datetime import datetime
from sys import platform

# Open serial port
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for the serial connection to initialize

#-------------------------------------
# SetEnableAll
#-------------------------------------
def SetEnableAll(value):
	ser.write(("enable M0 " + str(value)+ '\n').encode())
	ser.write(("enable M1 " + str(value)+ '\n').encode())
	ser.write(("enable M2 " + str(value)+ '\n').encode())
	ser.write(("enable M3 " + str(value)+ '\n').encode())

	ser.write(("enable S0 " + str(value)+ '\n').encode())
	ser.write(("enable S1 " + str(value)+ '\n').encode())
	ser.write(("enable S2 " + str(value)+ '\n').encode())
	ser.write(("enable S3 " + str(value)+ '\n').encode())
	ser.write(("enable S4 " + str(value)+ '\n').encode())
	ser.write(("enable S5 " + str(value)+ '\n').encode())
	ser.write(("enable S6 " + str(value)+ '\n').encode())
	ser.write(("enable S7 " + str(value)+ '\n').encode())

#-------------------------------------
# ControllerSupportApproxEngLib - Robot-specific input library
#-------------------------------------
def ControllerSupportApproxEngLib():
	last_R3 = False
	last_L3 = False
	last_left_joy_V = -1
	last_left_joy_H = -1
	last_right_joy_V = -1
	last_right_joy_H = -1

	left_joy_V = 0
	left_joy_H = 0
	right_joy_V = 0
	right_joy_H = 0

	last_P1 = -1
	last_P2 = -1
	last_P3 = -1

	P1 = 0
	P2 = 0
	P3 = 0
	while True:
		try:
			with ControllerResource() as joystick:
				print('Found a joystick and connected')
				while joystick.connected:
					presses = joystick.check_presses()
					lx, ly = joystick['l']
					rx, ry = joystick['r']
					ls = joystick['ls']
					rs = joystick['rs']

					if presses['ls']:
						SetEnableAll(0)
					if presses['rs']:
						SetEnableAll(1)
					
					P1 = (2.0/3.0)*lx+(1.0/3.0)*rx
					P2 = (-1.0/3.0)*lx+(1.0/math.sqrt(3.0))*ly+(1.0/3.0)*rx
					P3 = (-1.0/3.0)*lx-(1.0/math.sqrt(3.0))*ly+(1.0/3.0)*rx

					print("=======")
					print("P1:", P1)
					print("P2:", P2)
					print("P3:", P3)
					print("=======")

					if( abs(P1 ) < 0.15 ) :  P1 = 0
					if( abs(P2 ) < 0.15 ) :  P2 = 0
					if( abs(P3 ) < 0.15 ) :  P3 = 0

					if( abs( P1 - last_P1 ) > 0.05 ):
								last_P1 = P1
								cmd = "set M0 " + str(P1)
								ser.write((cmd + '\n').encode())
						#while ser.in_waiting > 0:
								#response = ser.readline().decode('utf-8').rstrip()
								#print(f"ESP32 Response: {response}")

					if( abs( P2 - last_P2 ) > 0.05 ):
								last_P2 = P2
								cmd = "set M1 " + str(P2)
								ser.write((cmd + '\n').encode())
						#while ser.in_waiting > 0:
								#response = ser.readline().decode('utf-8').rstrip()
								#print(f"ESP32 Response: {response}")

					if( abs( P3 - last_P3 ) > 0.05 ):
								last_P3 = P3
								cmd = "set M3 " + str(P3)
								ser.write((cmd + '\n').encode())
						#while ser.in_waiting > 0:
								#response = ser.readline().decode('utf-8').rstrip()
								#print(f"ESP32 Response: {response}")

			# Joystick disconnected...
			print('Connection to joystick lost')
		except IOError:
			# No joystick found, wait for a bit before trying again
			print('Unable to find any joysticks')
			time.sleep(1.0)


#-------------------------------------
# Controller Support - Should support all OS's
#-------------------------------------
def ControllerSupport():
	last_R3 = False
	last_L3 = False
	last_left_joy_V = -1
	last_left_joy_H = -1
	last_right_joy_V = -1
	last_right_joy_H = -1

	left_joy_V = 0
	left_joy_H = 0
	right_joy_V = 0
	right_joy_H = 0

	last_P1 = -1
	last_P2 = -1
	last_P3 = -1

	P1 = 0
	P2 = 0
	P3 = 0

	while True:
		report = get_gamepad()
		# print(report)
		if report:
			for event in report:
				print("-------")
				print(event.ev_type, event.code, event.state)
				print("-------")
				match event.code:
					case "BTN_THUMBL":
						SetEnableAll(0)
						print("L3 pressed")
					case "BTN_THUMBR":
						SetEnableAll(1)
						print("R3 pressed")
					case "ABS_X":
						left_joy_H = event.state / MAX_JOY_VAL
					case "ABS_Y":
						left_joy_V = event.state / MAX_JOY_VAL
					case "ABS_RX":
						right_joy_H = event.state / MAX_JOY_VAL
					case "ABS_RY":
						right_joy_V = event.state / MAX_JOY_VAL
					case "BTN_START":
						SetEnableAll(1)
						print("Start pressed")
					case "ABS_LZ":
						SetEnableAll(0)
						print("L3 pressed")


				P1 = (2.0/3.0)*left_joy_H+(1.0/3.0)*right_joy_H
				P2 = (-1.0/3.0)*left_joy_H+(1.0/math.sqrt(3.0))*left_joy_V+(1.0/3.0)*right_joy_H
				P3 = (-1.0/3.0)*left_joy_H-(1.0/math.sqrt(3.0))*left_joy_V+(1.0/3.0)*right_joy_H

				# # Motors are reversed
				P1 = -round(P1,3)
				P2 = -round(P2,3)
				P3 = -round(P3,3)


				print("=======")
				print("P1:", P1)
				print("P2:", P2)
				print("P3:", P3)
				print("=======")

				if( abs(P1 ) < 0.15 ) :  P1 = 0
				if( abs(P2 ) < 0.15 ) :  P2 = 0
				if( abs(P3 ) < 0.15 ) :  P3 = 0

				if( abs( P1 - last_P1 ) > 0.05 ):
							last_P1 = P1
							cmd = "set M0 " + str(P1)
							ser.write((cmd + '\n').encode())
					#while ser.in_waiting > 0:
							#response = ser.readline().decode('utf-8').rstrip()
							#print(f"ESP32 Response: {response}")

				if( abs( P2 - last_P2 ) > 0.05 ):
							last_P2 = P2
							cmd = "set M1 " + str(P2)
							ser.write((cmd + '\n').encode())
					#while ser.in_waiting > 0:
							#response = ser.readline().decode('utf-8').rstrip()
							#print(f"ESP32 Response: {response}")

				if( abs( P3 - last_P3 ) > 0.05 ):
							last_P3 = P3
							cmd = "set M3 " + str(P3)
							ser.write((cmd + '\n').encode())
					#while ser.in_waiting > 0:
							#response = ser.readline().decode('utf-8').rstrip()
							#print(f"ESP32 Response: {response}")


				print("left_joy_H", left_joy_H)
				print("left_joy_V", left_joy_V)
				print("right_joy_H", right_joy_H)
				print("right_joy_V", right_joy_V)
				print("=================")
		else:
			#state = None
			time.sleep(0.1)
			#print('Unable to get controller state')

def KeyboardSupport():
	while True:
		report = get_key()
		# print(report)
		if report:
			for event in report:
				print("-------")
				print(event.ev_type, event.code, event.state)
				print("-------")
				match event.code:
					case "BTN_THUMBL":
						SetEnableAll(0)
						print("L3 pressed")
					case "BTN_THUMBR":
						SetEnableAll(1)
						print("R3 pressed")
					case "ABS_X":
						if controller_type == "PS4":
							if event.state == 0:
								left_joy_H = event.state / MAX_JOY_VAL
						else:
							left_joy_H = event.state / MAX_JOY_VAL
					case "ABS_Y":
						left_joy_V = event.state / MAX_JOY_VAL
					case "ABS_RX":
						right_joy_H = event.state / MAX_JOY_VAL
					case "ABS_RY":
						right_joy_V = event.state / MAX_JOY_VAL
					case "BTN_START":
						SetEnableAll(1)
						print("Start pressed")
					case "ABS_LZ":
						SetEnableAll(0)
						print("L3 pressed")


				P1 = (2.0/3.0)*left_joy_H+(1.0/3.0)*right_joy_H
				P2 = (-1.0/3.0)*left_joy_H+(1.0/math.sqrt(3.0))*left_joy_V+(1.0/3.0)*right_joy_H
				P3 = (-1.0/3.0)*left_joy_H-(1.0/math.sqrt(3.0))*left_joy_V+(1.0/3.0)*right_joy_H

				# # Motors are reversed
				P1 = -round(P1,3)
				P2 = -round(P2,3)
				P3 = -round(P3,3)


				print("=======")
				print("P1:", P1)
				print("P2:", P2)
				print("P3:", P3)
				print("=======")

				if( abs(P1 ) < 0.1 ) :  P1 = 0
				if( abs(P2 ) < 0.1 ) :  P2 = 0
				if( abs(P3 ) < 0.1 ) :  P3 = 0

				if( abs( P1 - last_P1 ) > 0.05 ):
							last_P1 = P1
							cmd = "set M0 " + str(P1)
							ser.write((cmd + '\n').encode())
					#while ser.in_waiting > 0:
							#response = ser.readline().decode('utf-8').rstrip()
							#print(f"ESP32 Response: {response}")

				if( abs( P2 - last_P2 ) > 0.05 ):
							last_P2 = P2
							cmd = "set M1 " + str(P2)
							ser.write((cmd + '\n').encode())
					#while ser.in_waiting > 0:
							#response = ser.readline().decode('utf-8').rstrip()
							#print(f"ESP32 Response: {response}")

				if( abs( P3 - last_P3 ) > 0.05 ):
							last_P3 = P3
							cmd = "set M3 " + str(P3)
							ser.write((cmd + '\n').encode())
					#while ser.in_waiting > 0:
							#response = ser.readline().decode('utf-8').rstrip()
							#print(f"ESP32 Response: {response}")


				print("left_joy_H", left_joy_H)
				print("left_joy_V", left_joy_V)
				print("right_joy_H", right_joy_H)
				print("right_joy_V", right_joy_V)
				print("=================")
		else:
			#state = None
			time.sleep(0.1)
			#print('Unable to get controller state')

if len(sys.argv) > 1:
	controller_type = sys.argv[1]
	print("Setting controller to: %s" % controller_type)

if controller_type == "PS4":
	MAX_TRIG_VAL = 255
	MAX_JOY_VAL = 255
	ControllerSupport()

if controller_type == "Other":
	MAX_TRIG_VAL = math.pow(2, 8)
	MAX_JOY_VAL = math.pow(2, 15)
	ControllerSupport()

if controller_type == "approxenglib":
	ControllerSupportApproxEngLib()

if controller_type == "Keyboard":
	KeyboardSupport()
