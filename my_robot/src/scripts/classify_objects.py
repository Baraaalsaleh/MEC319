#!/usr/bin/env python

if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
import sys
import rospy
import cv2 as cv
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from include.object_localization  import *
from keras.models import load_model
import numpy
#from include.object_classification  import *


class LocateObjects:

    def __init__(self, keypoint, object_found):
    
        self.set_keypoint(keypoint)

        self.set_object_found(object_found)
        
        self._t0 = time.time()
        
        self.blob_point = Point()
    
        #print (">> Publishing image to topic image_blob")
        #self.image_pub = rospy.Publisher("/blob/image_blob",Image,queue_size=1)
        #self.mask_pub = rospy.Publisher("/blob/image_mask",Image,queue_size=1)
        #print (">> Publishing position to topic point_blob")
        self.blob_sub  = rospy.Subscriber("/blob/point_blob",Point,queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/mybot/camera/image_raw",Image,self.callback)
        print ("<< Subscribed to topic /mybot/camera/image_raw")
        
    def set_keypoint(self, keypoint):
        self._keypoint = keypoint
        
    def set_object_found(self, object_found):
        self._object_found = object_found




    def callback(self,data):

        keypoint = self._keypoint
        object_found = self._object_found

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            point = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            
            #object_part = crop_object_part(img, center_coordinates, object_size)
            #result_img, processed_img, im_with_keypoints, center_coordinates, object_size = locate_object(cv_image, threshold, set_value, color, extra, min_area)
            #print("I got the image")
            #print(cv_image.shape)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(processed_img, "8UC1"))
                #self.mask_pub.publish(self.bridge.cv2_to_imgmsg(im_with_keypoints, "bgr8"))
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(result_img, "bgr8"))

            except CvBridgeError as e:
                print(e)            
                
            self.blob_point.x = center_coordinates[0]
            self.blob_point.y = center_coordinates[1]
            self.blob_pub.publish(self.blob_point) 
            


            fps = 1.0/(time.time()-self._t0)
            self._t0 = time.time()

            if object_size != 0:
                print ("Objekt wurde gefunden : Groesse des Objekts = %3d\tObjektskoordinaten: x = %3d  y= %3d\tZeit: %3d\n"%(object_size, center_coordinates[0], center_coordinates[1], self._t0))
            print("Bilderrate " + str(fps))

def main(args):

    ic = LocateObjects(threshold=100, set_value=240, color = (200,200,255), extra=150, min_area = 200)
    rospy.init_node('blob_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
