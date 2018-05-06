#!/usr/bin/env python
import numpy as np
import cv2

import rospy
import rospkg
import std_msgs.msg
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError

try :
	isCameraImported = True
	# Import can only work on the Raspberry	
	from picamera import PiCamera
except ImportError:
	print("[WARNING] Can't import picamera in raspicam node")
	isCameraImported = False

# Global variables
pub_image = 0
rospack = 0
cv_bridge = 0
camera = 0


def callback_load_image(data):
	""" Load an image and publish it
	"""
	global cv_bridge, pub_image, rospack
	rospy.loginfo("callback_take_photo called")
	
	# Load the image
	imagePath = rospack.get_path('construction_plan') + "/images/test.jpg"	
	image = cv2.imread(imagePath, cv2.IMREAD_COLOR)
	
	if image is None:
		rospy.loginfo("Can't open image at :")
		rospy.loginfo(imagePath)
	
	# Publish the image
	pub_image.publish(cv_bridge.cv2_to_imgmsg(image, "bgr8"))
	

def callback_take_photo(data):
	""" Take a photo with the RaspiCam and publish it
	"""
	global cv_bridge, pub_image, isCameraImported
	
	if not isCameraImported:
		rospy.loginfo("PiCamera has not been imported")
		return
		
	with picamera.PiCamera() as camera:
		camera.resolution = (320, 240)
		camera.framerate = 24
		#time.sleep(2)
		image = np.empty((240 * 320 * 3,), dtype=np.uint8)
		camera.capture(image, 'bgr8')
		image = image.reshape((240, 320, 3))
		
		pub_image.publish(cv_bridge.cv2_to_imgmsg(image, "bgr8"))
	
	

def raspicam_node():
	global cv_bridge, pub_image, rospack, isCameraImported

	# Init node
	rospy.init_node('raspicam', anonymous=True)
	rospy.loginfo("Node raspicam connected to roscore")

	pub_image = rospy.Publisher("/raspicam/image", sensor_msgs.msg.Image, queue_size=1)
	rospy.Subscriber("/raspicam/take_photo", std_msgs.msg.Float64, callback_take_photo)

	# Package manager declaration
	rospack = rospkg.RosPack()
	
	# OpenCV initialisation
	cv_bridge = CvBridge()
	
	# PiCamera intialisation
	"""if isCameraImported:
		camera = PiCamera()
		camera.resolution = (1024, 768)	
		rospy.loginfo("Raspicam has been opened")"""
		

	# Main loop    
	rospy.spin()
	

if __name__ == '__main__':
    try:
        raspicam_node()
    except rospy.ROSInterruptException:
        pass
