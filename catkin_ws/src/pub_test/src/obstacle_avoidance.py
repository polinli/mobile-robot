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
		GPIO.add_event_detect(self.LEFT_BUTTON, GPIO.RISING, callback = self.left_touched, bouncetime = 200)
		GPIO.add_event_detect(self.RIGHT_BUTTON, GPIO.RISING, callback = self.right_touched, bouncetime = 200)
		GPIO.add_event_detect(self.MIDDLE_BUTTON, GPIO.RISING, callback = self.middle_touched, bouncetime = 200) 

		#define callback
		#GPIO.add_event_callback(self.LEFT_BUTTON, callback = self.left_touched)
		#GPIO.add_event_callback(self.RIGHT_BUTTON, callback = self.right_touched)
		#GPIO.add_event_callback(self.MIDDLE_BUTTON, callback = self.middle_touched)

		self.forward()

	#def run(self, delay):
	#	self.go_straight()
		
	def forward(self):
		self.motor_msg.data = [100, 100]
		self.cmd_publish()

		#time.sleep(delay)
		#self.stop()

	def backward(self, delay):
		self.motor_msg.data = [-100, -100]
		self.cmd_publish()

		time.sleep(delay)
		self.stop()

	def stop(self):
		self.motor_msg.data = [0, 0]
		self.cmd_publish()


	def turn_left(self, delay):
		self.motor_msg.data = [-100, 100]
		self.cmd_publish()

		time.sleep(delay)
		self.stop()

	def turn_right(self, delay):
		self.motor_msg.data = [100, -100]
		self.cmd_publish()

		time.sleep(delay)
		self.stop()

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
		time.sleep(0.2)
		#self.stop()
		car.forward()

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
		time.sleep(0.2)
		#self.stop()
		car.forward()

	def both_touched(self):
		self.backward(3)
		self.turn_left(3)

	def middle_touched(self, channel):
	    pass	
            #find the ball and stop
		


	def get_brightness():
		return brightness

	def cmd_publish(self):
		self.car_cmd_pub.publish(self.motor_msg)
		
	def on_shutdown():
		GPIO.cleanup()

if __name__ == '__main__':
	rospy.init_node("control_node", anonymous = True)
	car = ControlNode()
	
	while not rospy.is_shutdown():
		brightness_record = [0, 0, 0]
		#print("dick")
		#car.forward(1)
		pass
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
	rospy.spin()
	#rospy.on_shutdown(car.on_shutdown)
	
