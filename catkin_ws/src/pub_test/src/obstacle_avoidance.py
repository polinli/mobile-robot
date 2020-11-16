#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int32

try:
	import RPi.GPIO as GPIO
	import time
except RuntimeError:
	print("GPIO import error")

class ControlNode(object):
	def __init__(self):
		self.node_name = rospy.get_name()

		self.car_cmd_pub = rospy.Publisher("/motor", Int32MultiArray, queue_size = 1)
		self.brightness_sub = rospy.Subscriber("/brightness", Int32, self.cb_brightness, queue_size = 1)
		

		self.motor_msg = Int32MultiArray() #motor speed array [left, right]
		self.brightness  = 0

		#switch button pin
		self.LEFT_BUTTON = 21
		self.RIGHT_BUTTON = 21
		self.MIDDLE_BUTTON = 21

		#GPIO setup
		GPIO.setwarnings(False) #ignore warnings
		GPIO.setmode(GPIO.BCM)#GPIO.BOARD GPIO編號或Port Pin編號
		GPIO.setup(self.LEFT_BUTTON , GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
		GPIO.setup(self.RIGHT_BUTTON , GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
		GPIO.setup(self.MIDDLE_BUTTON , GPIO.IN, pull_up_down = GPIO.PUD_DOWN)

		#switch button rising interrupt
		GPIO.add_event_detect(self.LEFT_BUTTON, GPIO.RISING)
		GPIO.add_event_detect(self.RIGHT_BUTTON, GPIO.RISING)
		GPIO.add_event_detect(self.MIDDLE_BUTTON, GPIO.RISING) 

		#define callback
		GPIO.add_event_callback(self.LEFT_BUTTON, callback = self.left_touched)
		GPIO.add_event_callback(self.RIGHT_BUTTON, callback = self.right_touched)
		GPIO.add_event_callback(self.MIDDLE_BUTTON, callback = self.middle_touched)

	#def run(self, delay):
	#	self.go_straight()
		
	def forward(self, delay):
		self.motor_msg.data = [100, 100]
		self.cmd_publish()

		time.sleep(delay)
		self.stop()

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
		if(GPIO.input(RIGHT_BUTTON)): #both buttons are touched
			self.both_touched()
			return
		
		self.motor_msg.data = [0, -100]
		self.cmd_publish()
		GPIO.wait_for_edge(LEFT_BUTTON, GPIO.FALLING)
		self.stop()

	def right_touched(self, channel): 
		if(GPIO.input(LEFT_BUTTON)): #both buttons are touched
			self.both_touched()
			return
		
		self.motor_msg.data = [-100, 0]
		self.cmd_publish()
		GPIO.wait_for_edge(RIGHT_BUTTON, GPIO.FALLING)
		self.stop()

	def both_touched(self):
		self.backward(2)
		self.turn_left(3)

	def middle_touched(self, channel):
		#find the ball and stop
		
	def cb_brightness(self, msg):
		self.brightness = msg.data

	#def get_brightness():	return brightness

	def cmd_publish():
		self.car_cmd_pub.publish(self.motor_msg)
		
	def on_shutdown():
		GPIO.cleanup()

if __name__ == '__main__':
	rospy.init_node("control_node", anonymous = True)
	car = ControlNode()
	rospy.spin()
	
	while not rospy.is_shutdown():
		car.forward(5)

		middle_brightness = self.brightness

		car.turn_left(2)
		left_brightness = self.brightness
		car.turn_right(2)

		car.turn_right(2)
		right_brightness = self.brightness
		car.turn_left(2)
		
	rospy.on_shutdown(car.on_shutdown)
	
