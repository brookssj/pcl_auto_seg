#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int32, Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


controllerState = 0


class cImageHandler:

    def __init__(self):
        self.point_pub = rospy.Publisher("/rgb_seg/block_location", Point, queue_size=1)
        self.rotation_pub = rospy.Publisher("/rgb_seg/block_rotation", Float32, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.stateSub = rospy.Subscriber("/control_current_state", Int32, self.state_callback)


    def state_callback(self, state):
        global controllerState
        controllerState = state.data


    def callback(self, data):
        global controllerState

        if controllerState != 4:
            return

        sGauss = 5
        sThresh = 80

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        h = hsv_image[:,:,0]
        s = hsv_image[:,:,1]
        v = hsv_image[:,:,2]
        

        sGauss = cv2.GaussianBlur( v, (sGauss, sGauss), 0 )
        #cv2.imshow("sGauss", sGauss)
        _, threshS = cv2.threshold(sGauss, sThresh, 255, cv2.THRESH_BINARY)
        threshS = self.draw_over_smallest_blobs(threshS)
    
        #lower_red = np.array([30,150,50])
        #lower_red = np.array([30,150,50])
        #upper_red = np.array([255,255,255])
    
        #mask = cv2.inRange(hsv_image, lower_red, upper_red)
        #res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
        #threshS = self.draw_over_smallest_blobs(mask)

        #cv2.imshow('frame',cv_image)
        #median = cv2.medianBlur(res,15)
        #cv2.imshow('Median Blur',median)
        #blur = cv2.GaussianBlur(res,(15,15),0)
        #cv2.imshow('Gaussian Blurring',blur)
        #cv2.imshow('mask',mask)
        #cv2.imshow('res',res)


        block = self.get_largest_blob(threshS)
        if block == None:
            print "get_largest_blob returned None"
            return

        M = cv2.moments(block)
        if M['m00'] == 0:
            return

        centroidx = int(M['m10']/M['m00'])
        centroidy = int(M['m01']/M['m00'])
        #rect = cv2.minAreaRect(block)
        #centroidx = rect[0][0]
        #centroidy = rect[0][1]

        #epsilon = 0.1*cv2.arcLength(block,True)
        #approx = cv2.approxPolyDP(block,epsilon,True)
        #cv2.imshow("approxPoly", approx)
        #rotation = rect [2]

        #cv2.imshow("RGB Image", cv_image)

        x,y,w,h = cv2.boundingRect(block)
        cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)

        # "centroid:  ", centroidx, centroidy, "\n"

        cv2.imshow("RGB Image", cv_image)
        cv2.imshow("S Thresholded", threshS)
        cv2.waitKey(3)

        p = Point(centroidx, centroidy, 0)
        self.point_pub.publish(p)
        #self.rotation_pub.publish(rotation)


    def draw_over_smallest_blobs(self, image):
        copyOfImage = image.copy()

        # We want to keep the biggest blob so long as it isn't too big.
        biggestIndex = -1
        biggestArea = 0.0
        contours, _ = cv2.findContours(copyOfImage, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if contours.__len__() > 0:
            for i in range(contours.__len__()):
                c_area = cv2.contourArea(contours[i])
                if c_area > biggestArea:
                    biggestIndex = i
                    biggestArea = c_area
                    
        if biggestIndex != -1:
            for i in range(0,contours.__len__()):
                if i != biggestIndex:
                    cv2.drawContours(image, contours, i, (0,0,0), -1, cv2.CV_AA)
        
        return image


    def get_largest_blob(self, image):
        copyOfImage = image.copy()

        biggestIndex = -1
        biggestArea = 0.0
        contours, _ = cv2.findContours(copyOfImage, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        if contours.__len__() > 0:
            for i in range(contours.__len__()):
                c_area = cv2.contourArea(contours[i])
                if c_area > biggestArea:
                    biggestIndex = i
                    biggestArea = c_area
        else:
            return None

        return contours[biggestIndex]        


def main(args):
    handler = cImageHandler()
    rospy.init_node("rgb_block_pos", anonymous=True)

    rospy.spin()
    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
