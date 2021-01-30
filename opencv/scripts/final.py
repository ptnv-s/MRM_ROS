#!/usr/bin/env python3

import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import matplotlib.pyplot as plt
 
def callback(data):
  print("Received an image!")
  

  br = CvBridge()
  rospy.loginfo("recieving image")
  current_frame = br.imgmsg_to_cv2(data)
  cv2.imshow("camera", current_frame)
  plt.imshow(current_frame)
  plt.title('my picture')
  plt.show() 
      
def receive_message():
  rospy.Subscriber('output', Image, callback)
  rospy.init_node('final', anonymous=True)
  # Node is subscribing to the video_frames topic
  print("Received an image!")
  #callback(Image)
  try:
      rospy.spin()
  except KeyboardInterrupt:
      print("Shutting down")
 
  
  #cv2.destroyAllWindows()
  
if __name__ == '__main__':
  receive_message()                                                      