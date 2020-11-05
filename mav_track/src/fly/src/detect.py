#! /usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
from sensor_msgs.msg import Image
from pymavlink import mavutil
import numpy as np 

class detect(object):

    def __init__(self):
        self.bridge = CvBridge()
        self.imageReciever = rospy.Subscriber('/iris/camera2/image_raw', Image, self.imageRecieverCallback)
        self.positionX = 0
        self.positionY = 0
        self.pRate = 0.08 * 10000
        self.flyFlag = 0
        self.master = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
        self.master.wait_heartbeat()
        self.adjustPosition()

    # Subscriber callback function to get position information from the reiceved image
    def imageRecieverCallback(self, data):        
        try:
            imageFrame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)
        # Set range for red color and 
        # define mask 
        red_lower = np.array([0, 100, 100], np.uint8)
        red_upper = np.array([10, 255, 255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        # Morphological Transform, Dilation 
        # for each color and bitwise_and operator
        # between imageFrame and mask determines
        # to detect only that particular color
        kernal = np.ones((5, 5), "uint8")
        # For red color 
        red_mask = cv2.dilate(red_mask, kernal)
        res_red = cv2.bitwise_and(imageFrame, imageFrame, mask = red_mask)
        # Creating contour to track red color 
        contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour)
            if(area > 200):
                x, y, w, h = cv2.boundingRect(contour)
                imageFrame = cv2.rectangle(imageFrame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(imageFrame, "Target", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))
                self.positionX = x + w/2
                self.positionY = y + h/2
                self.flyFlag = 1
                #rospy.loginfo(self.positionX)
                #rospy.loginfo(self.positionX)
            else:
                self.positionX = 0
                self.positionY = 0
                self.flyFlag = 0
        #rospy.loginfo(imageFrame.shape)
        cv2.imshow("Image window", imageFrame)
        cv2.waitKey(3) 

    def tranformCoordinates(self,x,y):
        #rospy.loginfo(x)
        x = (x/640.0) - 0.5
        y = (y/480.0) - 0.5
        nedX = y * self.pRate
        #rospy.loginfo(nedX)
        nedY = (x * self.pRate) * -1
        return nedX,nedY
    
    def adjustPosition(self):
        x = 0
        y = 0
        z = 500
        while 1: #self.positionX != 0 and self.positionY != 0:
            while self.flyFlag:
                x,y = self.tranformCoordinates(self.positionX,self.positionY)
                mode = 'LOITER'
                mode_id = self.master.mode_mapping()[mode]
                self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
                while True:
                    # Wait for ACK command
                    ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True)
                    ack_msg = ack_msg.to_dict()
                    # Check if command in the same in `set_mode`
                    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
                        continue
                    break
                rospy.loginfo("x:")
                rospy.loginfo(x)
                rospy.loginfo("y:")
                rospy.loginfo(y)
                rospy.loginfo("z:")
                rospy.loginfo(z)
                self.master.mav.manual_control_send(self.master.target_system, x, y, z,0,0)
                if abs(x) < 40 and abs(y) < 40:
                    #mode = 'GUIDED'
                    #mode_id = self.master.mode_mapping()[mode]
                    #self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
                    z = z-1;
                    break
                
            if z < 200:
                break
                
            





if __name__ == "__main__":
    rospy.init_node('detect_red', log_level=rospy.INFO)
    detect_object = detect()
    rospy.spin()