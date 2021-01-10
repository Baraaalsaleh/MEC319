import cv2 as cv
import numpy as np
from keras.models import load_model
import tensorflow as tf
import time


def apply_sobel(img, scale = 1, delta = 0, ddepth = 3, ksize=3, weight=1, gamma=0):

    search_region = int(img.shape[1]/10)

    gray = cv.cvtColor(img[0:len(img[0]), int((len(img[1])/2) - search_region):int((len(img[1])/2) + search_region)], cv.COLOR_BGR2GRAY)

    grad_x = cv.Sobel(gray, ddepth, 1, 0, ksize=ksize, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)
    grad_y = cv.Sobel(gray, ddepth, 0, 1, ksize=ksize, scale=scale, delta=delta, borderType=cv.BORDER_DEFAULT)

    abs_grad_x = cv.convertScaleAbs(grad_x)
    abs_grad_y = cv.convertScaleAbs(grad_y)

    grad = cv.addWeighted(abs_grad_x, weight, abs_grad_y, weight, gamma)

    #print("apply_sobel\n\t" + str(grad.shape))

    return grad

def process_grad(grad, threshold = 100, set_value = 240):

    threshold_matrix = threshold*np.ones(grad.shape)

    processed_img = grad - threshold_matrix

    processed_img = ((processed_img > 0).astype(int))*set_value

    processed_img = processed_img.astype(np.uint8)

    #print("process_grad\n\t" + str(processed_img.shape))

    return processed_img



def find_search_window(processed_img, set_value = 240):
    min_y = 0
    max_y = 0
    min_x = 0
    max_x = processed_img.shape[1]

    found = False
    for i in range(processed_img.shape[0]):
        if processed_img[i][0] == set_value and processed_img[i][int(len(processed_img[1])/2)] == set_value:
            if found and i < processed_img.shape[0]/2:
                min_y = i + 1
            elif found:
                max_y = i - 1
                break
            else:
                found = True 

    search_window = [min_y, max_y, min_x, max_x]

    #print("find_search_window\n\t" + str(search_window))
    return search_window
    


def add_mask(processed_img, img, set_value = 240):
    result_img = img.copy()

    search_region = int(img.shape[1]/10)

    for i in range(len(processed_img)):
        for j in range(len(processed_img[0])):

            if processed_img[i][j] == set_value:

                result_img[i][j + int((len(img[1])/2) - search_region)][0] = 50
                result_img[i][j + int((len(img[1])/2) - search_region)][1] = 200
                result_img[i][j + int((len(img[1])/2) - search_region)][2] = 200

    #print("result_img\n\t" + str(result_img.shape))

    return result_img



def find_object(processed_img, img, search_window, set_value = 240, min_area = 200, color = (200,200,255)):
    min_y = search_window[0]
    max_y = search_window[1]
    min_x = search_window[2]
    max_x = search_window[3]

    params = cv.SimpleBlobDetector_Params()

    params.minThreshold = set_value - 20
    params.maxThreshold = set_value + 20

    params.filterByArea = True
    params.minArea = min_area

    detector = cv.SimpleBlobDetector_create(params)

    search_region = int(img.shape[1]/10)

    keypoints = detector.detect(processed_img[min_y:max_y, min_x:max_x])

    for  kp in keypoints:
        kp.pt = (int(kp.pt[0]) + int((len(img[1])/2) - search_region) , min_y + int(kp.pt[1]))
    
    im_with_keypoints = cv.drawKeypoints(img, keypoints, np.array([]), color, cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    #print("find_object\n\t" + str(im_with_keypoints.shape) + "\n\t")

    return im_with_keypoints,  keypoints

def crop_object_part(img, coordinates, size, extra=20):
    x1 = int(coordinates[0] - ((size/2) + extra))
    x2 = int(coordinates[0] + ((size/2) + extra))
    

    y1 = int(coordinates[1] - ((size/2) + extra))
    y2 = int(coordinates[1] + ((size/2) + extra))

    I = img[y1:y2, x1:x2]
    #print("\n\n")
    #print("crop_object_part:\n\tCoordinates" + str(coordinates))
    #print("\tSize: " + str(size))
    #print((x1, x2, y1, y2))
    #print(I.shape)
    #print(coordinates[0])
    #print(coordinates[1])
    #print(extra)
    #print("\n\n")

    return I

def locate_object(img, threshold=100, set_value=240, color = (200,200,255), extra=20, min_area = 200):
    tic = time.time()
    grad = apply_sobel(img, weight=2.5)

    processed_img = process_grad(grad, threshold, set_value)

    search_window = find_search_window(processed_img, set_value)

    im_with_keypoints, kp = find_object(processed_img, img, search_window, set_value, min_area, color)

    center_coordinates = (0, 0)
    object_size = 0

    if len(kp) != 0:
        center_coordinates = kp[0].pt
        object_size = kp[0].size

    result_img = add_mask(processed_img, img)

    object_part = crop_object_part(img, center_coordinates, object_size, extra)
    toc = time.time()

    #print("locate_object dauert: " + str(toc - tic))

    return result_img, processed_img, im_with_keypoints, object_part, center_coordinates, object_size

def get_real_coordinates(keypoint_coordinates, object_size_in_px, img_size, fliessband_hight=0.65, camera_coordinates=(-3, 1.5, 1.5), camera_horizontal_fov=1.1):
    
    pixel_size_x = camera_horizontal_fov/img_size[1]
    pixel_size_y = camera_horizontal_fov/img_size[0]
    #print("px size: %3d"%(pixel_size))
    y_size = pixel_size_y * object_size_in_px

    camera_frame_x_min = camera_coordinates[0] + (camera_horizontal_fov/2)
    camera_frame_y_min = camera_coordinates[1] - (camera_horizontal_fov/2)
    camera_frame_z_min = fliessband_hight

    camera_frame_zero_coordinates = (camera_frame_x_min, camera_frame_y_min, camera_frame_z_min)

    object_center_coordinates_x = camera_frame_x_min - (keypoint_coordinates[0]*pixel_size_x)

    object_center_coordinates_y = camera_frame_y_min + (keypoint_coordinates[1]*pixel_size_y)

    center_coordinates = (object_center_coordinates_x, object_center_coordinates_y, fliessband_hight)

    #print("get_real_coordinates:\n\t" + str((camera_horizontal_fov, camera_coordinates, camera_frame_x_min, camera_frame_y_min)))

    return center_coordinates, y_size

def classify(img, model, img_shape=(-1, 32, 32, 3)):
    #model1 = load_model('/home/baraa-ubuntu18/Desktop/project/LasttestAccIs_9697.h5')
    tic = time.time()
    image = cv.resize(img, (img_shape[1],img_shape[2]))
    image = image/255.0
    image = image.reshape(img_shape)

    #my_prediction = np.array(model.predict([image]))
    object_class_nummer = model.predict_classes(image)

    toc = time.time()
    #print("classify dauert: " + str(toc - tic))

    #my_prediction = np.amax(my_prediction)
    return object_class_nummer




if __name__=="__main__":

    SOURCE = 'camera'
    
    if SOURCE == 'video':
        cap = cv.VideoCapture(0)
        while(True):
            # Capture frame-by-frame
            ret, frame = cap.read()
            
            #-- Detect keypoints
            result_img, object_part, center_coordinates, object_size   = locate_object(threshold, set_value, color, extra, min_area)

            #-- click ENTER on the image window to proceed
            cv.imshow(result_img)

            #-- press q to quit
            if cv.waitKey(1) & 0xFF == ord('q'):
                break
        
    else:
        #-- Read image list from file:
        image_list = []
        image_list.append(cv.imread("blob.jpg"))
        #image_list.append(cv2.imread("blob2.jpg"))
        #image_list.append(cv2.imread("blob3.jpg"))

        for image in image_list:
            #-- Detect keypoints
            result_img, object_part, center_coordinates, object_size   = locate_object(threshold, set_value, color, extra, min_area)
            
            cv.imshow("Object", object_part)
            cv.waitKey(0)            
            
            cv.imshow(result_img)
            #-- enter to proceed
            cv.waitKey(0)
        
            image    = draw_frame(image)
            cv.imshow("Frame", image)
            cv.waitKey(0)