#!/usr/bin/env python

if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
import sys
import rospy
import cv2 as cv
import time
#import cntk as C

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from include.object_localization  import *
from include.object_classification  import *
from keras.models import load_model

class LocateObjects:

    def __init__(self, threshold=100, set_value=240, color = (200,200,255), extra=20, min_area = 200):
    
        self.set_threshold(threshold)
        self.set_set_value(set_value)
        self.set_color(color)
        self.set_extra(extra)
        self.set_min_area(min_area)
        self.set_last_time(time.time() - 1)
        self.set_last_coordinates((0, 0))
        self.set_velocity_calculated(False)
        self.set_belt_velocity(0)
        self.set_classes(["Rote Zylinder", "Gruene Zylinder", "Blaue Zylinder", "Rote Wuerfel", "Gruene Wuerfel", "Blaue Wuerfel"])
        
        
        self._t0 = time.time()
        
        self.blob_point = Point()
    
        print (">> Publishing image to topic image_blob")
        self.image_pub = rospy.Publisher("/blob/image_blob",Image,queue_size=1)
        self.mask_pub = rospy.Publisher("/blob/image_mask",Image,queue_size=1)
        print (">> Publishing position to topic point_blob")
        self.blob_pub  = rospy.Publisher("/blob/point_blob",Point,queue_size=1)
        self.class_pub  = rospy.Publisher("/blob/class",String,queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/mybot/camera/image_raw",Image,self.callback)
        print ("<< Subscribed to topic /mybot/camera/image_raw")
        
    def set_threshold(self, threshold):
        self._threshold = threshold
        
    def set_set_value(self, set_value):
        self._set_value = set_value

    def set_color(self, color):
        self._color = color

    def set_extra(self, extra):
        self._extra = extra

    def set_min_area(self, min_area):
        self._min_area = min_area
        
    def set_last_time(self, last_time):
        self._last_time = last_time
    
    def set_velocity_calculated(self, velocity_calculated):
        self._velocity_calculated = velocity_calculated

    def set_last_coordinates(self, last_coordinates):
        self._last_coordinates = last_coordinates
    
    def set_belt_velocity(self, belt_velocity):
        self._belt_velocity = belt_velocity

    def set_classes(self, classes):
        self._classes = classes



    def callback(self,data):

        threshold = self._threshold
        set_value = self._set_value
        color = self._color
        extra = self._extra
        min_area = self._min_area


        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            
            result_img, processed_img, im_with_keypoints, object_part, center_coordinates, object_size = locate_object(cv_image, threshold, set_value, color, extra, min_area)
            #print("I got the image")
            #print(cv_image.shape)
            #print("Extra is :" + str(extra))
            try:
                #self.image_pub.publish(self.bridge.cv2_to_imgmsg(processed_img, "8UC1"))
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(im_with_keypoints, "bgr8"))
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(result_img, "bgr8"))

            except CvBridgeError as e:
                print(e)            
                
            self.blob_point.x = center_coordinates[0]
            self.blob_point.y = center_coordinates[1]
            self.blob_pub.publish(self.blob_point) 
            
            
            
            fps = 1.0/(time.time()-self._t0)
            #print("Bilderrate " + str(fps))

            if object_size != 0 and (time.time() - self._last_time) > 1.5:
                #model = load_model('/home/baraa-ubuntu18/Desktop/project/LasttestAccIs_9697.h5')
                model = load_model('/home/baraa-ubuntu18/Desktop/project/model10.h5')
                center_real_coordinates, y_real_size = get_real_coordinates(center_coordinates, object_size, cv_image.shape, camera_coordinates=(-3, 1.5, 1.5))

                
                #print(center_coordinates)
                #print(object_size)
                #print(cv_image.shape)
                print ("Objekt wurde gefunden : Groesse des Objekts = %3d\tObjektskoordinaten: x = %3d  y= %3d\tZeit: %3d\n"%(object_size, center_coordinates[0], center_coordinates[1], self._t0))
                print("Die reale Koordinaten des Objekts sind:\tx:  %.3f meter\ty:  %.3f meter\tz:  %.3f meter\tGroesse des Objekts (Diameter) = %.3f"%(center_real_coordinates[0], center_real_coordinates[1], center_real_coordinates[2], y_real_size))
                class_number = classify(object_part, model)
                #cv.imshow("Something", object_part)
                #cv.waitKey(0)
                #cv.destroyAllWindows()
                print("\n\t>>>>>>>>>>>>Das Objekt ist: " + str(self._classes[class_number[0]]) + "\n\t>>>>>>>>>>>>Die Klasse des Objekts ist: " + str(class_number) + "\n\n")
                self._belt_velocity = (3 - (center_real_coordinates[0] - self._last_coordinates[0]) ) / (time.time() - self._last_time)

                print("Geschwindigkeit des Fliessbandes ist: " + str(self._belt_velocity) + " m/s  (x-Achse)")
                self._last_coordinates = center_real_coordinates
                self._last_time = time.time()
                #self._velocity_calculated = False

                
            
            """
            if (not self._velocity_calculated) and ((time.time() - self._last_time) > 0.05) and object_size != 0:
                center_real_coordinates, y_real_size = get_real_coordinates(center_coordinates, object_size, cv_image.shape)
                self._belt_velocity = (center_real_coordinates[0] - self._last_coordinates[0]) / ((time.time() - self._last_time))
                #print(center_real_coordinates)
                #print(self._last_coordinates)
                print("Geschwindigkeit des Fliessbandes ist: " + str(self._belt_velocity) + " m/s  (x-Achse)")
                self._velocity_calculated = True
            """






            
                #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Classify Object from object_part here<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


            self._t0 = time.time()

            

def main(args):

    ic = LocateObjects(threshold=100, set_value=240, color = (200,200,255), extra=5, min_area = 200)
    rospy.init_node('blob_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)