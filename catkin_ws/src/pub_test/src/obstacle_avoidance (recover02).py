#!/usr/bin/env python
import rospy
import time
import signal
import sys
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16

try:
	import RPi.GPIO as GPIO
	import time
except RuntimeError:
	print("GPIO import error")



class ControlNode(object):
	def cb_brightness(self, msg):
		self.brightness = msg.data

	def __init__(self):
		self.node_name = rospy.get_name()

		self.car_cmd_pub = rospy.Publisher("/wheel_speed", Int16MultiArray, queue_size = 5)
		self.brightness_sub = rospy.Subscriber("/brightness", Int16, self.cb_brightness, queue_size = 1)
		

		self.motor_msg = Int16MultiArray() #motor speed array [left, right]
		self.brightness  = 0

		#switch button GPIO pin
		self.LEFT_BUTTON = 23
		self.RIGHT_BUTTON = 21
		self.MIDDLE_BUTTON = 25

		#GPIO setup
		GPIO.setwarnings(False) #ignore warnings
		GPIO.setmode(GPIO.BCM)#GPIO.BOARD GPIO or Port Pin
		GPIO.setup(self.LEFT_BUTTON , GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
		GPIO.setup(self.RIGHT_BUTTON , GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
		GPIO.setup(self.MIDDLE_BUTTON , GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
		
		self.L_prev_inp = 1
		self.R_prev_inp = 1
		self.M_prev_inp = 1

		signal.signal(signal.SIGINT, self.signal_handler)
		'''
		#switch button rising interrupt
		GPIO.add_event_detect(self.LEFT_BUTTON, GPIO.RISING, callback = self.left_touched, bouncetime = 300)
		GPIO.add_event_detect(self.RIGHT_BUTTON, GPIO.RISING, callback = self.right_touched, bouncetime = 300)
		GPIO.add_event_detect(self.MIDDLE_BUTTON, GPIO.RISING, callback = self.middle_touched, bouncetime = 300) 
		'''
				
		print("car init")
		self.stop()

                self.forward()
                rospy.sleep(8)
                self.stop()
		
                self.navigate()


	#def run(self, delay):
	#self.go_straight()
		
	def forward(self):
		self.motor_msg.data = [160, 160]
		self.cmd_publish()
		print("forward")

		#rospy.sleep(5)
		#self.navigate()

	def backward(self, delay):
		self.motor_msg.data = [-120, -120]
		self.cmd_publish()
		print("backward")

		rospy.sleep(delay)
		self.stop()

	def stop(self):
		self.motor_msg.data = [0, 0]
		self.cmd_publish()
		rospy.sleep(1)


	def turn_left(self, delay):
		self.motor_msg.data = [-100, 100]
		self.cmd_publish()

		rospy.sleep(delay)
		#self.stop()

	def turn_right(self, delay):
		self.motor_msg.data = [100, -100]
		self.cmd_publish()

		rospy.sleep(delay)
		#self.stop()

	def turn_left_slow(self, delay):
		self.motor_msg.data = [-50, 50]
		self.cmd_publish()

		rospy.sleep(delay)
		#self.stop()

	def turn_right_slow(self, delay):
		self.motor_msg.data = [50, -50]
		self.cmd_publish()

		rospy.sleep(delay)
		#self.stop()


	def left_touched(self, channel): 
		if(GPIO.input(self.RIGHT_BUTTON)): #both buttons are touched
			self.both_touched()
			return

		self.motor_msg.data = [0, -100]
		self.cmd_publish()
		print("left touched")

		while GPIO.input(self.LEFT_BUTTON):
			time.sleep(0.2)
			while GPIO.input(self.LEFT_BUTTON):
				time.sleep(0.2)

		print("left release")
		rospy.sleep(0.2)
		self.stop()
		car.navigate()
		#self.forward()

	def right_touched(self, channel): 
		if(GPIO.input(self.LEFT_BUTTON)): #both buttons are touched
			self.both_touched()
			return
		
		self.motor_msg.data = [-100, 0]
		self.cmd_publish()
		print("right touched")

		while GPIO.input(self.RIGHT_BUTTON):
			time.sleep(0.2)
			while GPIO.input(self.RIGHT_BUTTON):
				time.sleep(0.2)

		print("right release")
		rospy.sleep(0.2)
		self.stop()
		self.navigate()
		#self.forward()

	def both_touched(self):
		print("both touch")
		self.backward(3)
		self.navigate()
		#self.turn_left(3)

	def middle_touched(self, channel):
	    self.stop()
		    #find the ball and stop
		


	def get_brightness(self):
		return self.brightness

	def cmd_publish(self):
		self.car_cmd_pub.publish(self.motor_msg)
		
	def on_shutdown(self):
		self.stop()
		GPIO.cleanup()

	def navigate(self):
		print("navigating")
		rec = []
		bri = 1000
		num = 0
		for i in range (5):
			self.turn_left_slow(0.5)
		self.stop()

		for i in range (10):
			self.turn_right_slow(0.5) 
			rec.append(self.get_brightness())
			if self.get_brightness() < bri :
				bri = self.get_brightness()
				num = i
		self.stop()
		print(rec)

		for i in range(10-num):
			self.turn_left_slow(0.5)
		print("navigated")
		self.stop()
		self.forward()

	def Btncheck(self):

		L_inp = GPIO.input(self.LEFT_BUTTON)
		R_inp = GPIO.input(self.RIGHT_BUTTON)
		M_inp = GPIO.input(self.MIDDLE_BUTTON)

		if ((not self.M_prev_inp) and M_inp):
			self.middle_touched()

		elif (((not self.L_prev_inp) and L_inp) and ((not self.R_prev_inp) and R_inp)):
			print("both touched")
			self.motor_msg.data = [-100,-100]
			self.cmd_publish()
			rospy.sleep(3)
			self.stop()
			self.navigate()

		elif ((not self.L_prev_inp) and L_inp):
			print("left touched")
                        self.motor_msg.data = [-100, -70]
                        self.cmd_publish()
                        rospy.sleep(3)
                        self.stop()

			self.motor_msg.data = [40,-100]
			self.cmd_publish()
			rospy.sleep(3)
			self.stop()
			self.navigate()

		elif ((not self.R_prev_inp) and R_inp):
			print("right touched")
			self.motor_msg.data = [-70, -100]
                        self.cmd_publish()
                        rospy.sleep(2)
                        self.stop()

                        self.motor_msg.data = [-100,40]
			self.cmd_publish()
			rospy.sleep(3)
			self.stop()
			self.navigate()

		self.L_prev_inp = L_inp
		self.R_prev_inp = R_inp
		self.M_prev_inp = M_inp
		rospy.sleep(0.2)

	def signal_handler(self, signal, frame):
		print("ctrl-C")
		self.stop()
		GPIO.cleanup()
		sys.exit(0)

if __name__ == '__main__':
	rospy.init_node("control_node", anonymous = True)
	car = ControlNode()
	
	while not rospy.is_shutdown():
		car.Btncheck()	

	#rospy.spin()
	car.stop()
	GPIO.cleanup()
	rospy.on_shutdown(car.on_shutdown)
