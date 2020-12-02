#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
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

		self.car_cmd_pub = rospy.Publisher("/wheel_speed", Int8MultiArray, queue_size = 5)
		self.brightness_sub = rospy.Subscriber("/brightness", Int16, self.cb_brightness, queue_size = 1)
		

		self.motor_msg = Int8MultiArray() #motor speed array [left, right]
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

		#switch button rising interrupt
		GPIO.add_event_detect(self.LEFT_BUTTON, GPIO.RISING, callback = self.left_touched, bouncetime = 300)
		GPIO.add_event_detect(self.RIGHT_BUTTON, GPIO.RISING, callback = self.right_touched, bouncetime = 300)
		GPIO.add_event_detect(self.MIDDLE_BUTTON, GPIO.RISING, callback = self.middle_touched, bouncetime = 300) 

		#define callback
		#GPIO.add_event_callback(self.LEFT_BUTTON, callback = self.left_touched)
		#GPIO.add_event_callback(self.RIGHT_BUTTON, callback = self.right_touched)
		#GPIO.add_event_callback(self.MIDDLE_BUTTON, callback = self.middle_touched)
				
		print("car init")
		self.stop()
		self.navigate()
		self.forward()
		#rospy.sleep(3)
		#print("stop")
		#self.forward()
		#self.turn_left(2)


	#def run(self, delay):
	#	self.go_straight()
		
	def forward(self):
		self.motor_msg.data = [100, 100]
		self.cmd_publish()
		print("forward")

		#rospy.sleep(5)
		#self.navigate()

	def backward(self, delay):
		self.motor_msg.data = [-100, -100]
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
	    pass	
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

		'''		
		self.turn_left(2)
		brightness_record = [0, 0] #[brightness, way]
		
		for i in range(10):
			self.turn_right(0.1)
						#time.sleep(0.1)
			if self.get_brightness() > brightness_record[0]  :
				brightness_record[0] = self.get_brightness()
				brightness_record[1] = i
		self.turn_left(4-0.2*(1+i))
		'''		

if __name__ == '__main__':
	rospy.init_node("control_node", anonymous = True)
	car = ControlNode()
	
	while not rospy.is_shutdown():
		#brightness_record = [0, 0, 0]
		#print("dick")
		try:		
			car.stop()		
		except KeyboardInterrupt :
			break
		'''
		brightness_record[1] = self.brightness #middel

		car.turn_left(2)
		brightness_record[0] = self.brightness #left
		car.turn_right(2)

		car.turn_right(2)
		brightness_record[2] = self.brightness #right
		car.turn_left(2)

		if(brightness_record.index(max(brightness_record)) == 0): #left side have the max brightness

		else if(brightness_record.index(max(brightness_record)) == 1):

		else if(brightness_record.index(max(brightness_record)) == 2):
		'''
	#rospy.spin()
	car.stop()
	GPIO.cleanup()
	rospy.on_shutdown(car.on_shutdown)
