#!/usr/bin/env python3
#-*-coding:utf-8-*-
##############################################
# The MIT License (MIT)
# Copyright (c) 2017 Kevin Walchko
# see LICENSE for full details
##############################################
# moves the roomba through a simple sequence

import pycreate2
import time
import serial
import sys, glob # for listing serial ports
import numpy as  np
import cv2
import math
import struct
import serial





class AU_Research_Irobot:

    # OBJECT INFORMATION

    KNOWN_DISTANCE = 35.0
    FLAG_WIDTH = 3.0
    OBSTACLE_WIDTH = 18.4
    callbackKeyUp = False
    callbackKeyDown = False
    callbackKeyLeft = False
    callbackKeyRight = False
    callbackKeyLastDriveCommand = ''


    def __init__(self):
        # Create a Create2 Bot
       
        bot = pycreate2.Create2(self.onConnect())
        #star robot
        bot.start()
        #initate safe mode
        bot.safe()
        
        self.initBot(bot)
   
    
   
    def object_detection(self, img_to_detect, focal_Length_flag, focal_Length_obstacle):
        
        img_height = img_to_detect.shape[0]
        img_width = img_to_detect.shape[1]
        
       
        
        # convert to blob to pass into model
        img_blob = cv2.dnn.blobFromImage(img_to_detect, 0.003922, (320, 320), swapRB=True, crop=False)
        
        # set of 80 class labels 
        class_labels = ["flag","obstacle"]
        
        #Declare List of colors as an array
        #Green, Blue, Red, cyan, yellow, purple
        #Split based on ',' and for every split, change type to int
        #convert that to a numpy array to apply color mask to the image numpy array
        class_colors = ["0,255,0","0,0,255"]
        class_colors = [np.array(every_color.split(",")).astype("int") for every_color in class_colors]
        class_colors = np.array(class_colors)
        class_colors = np.tile(class_colors,(16,1))
        
        # Loading pretrained model 
        # input preprocessed blob into model and pass through the model
        # obtain the detection predictions by the model using forward() method
        yolo_model = cv2.dnn.readNetFromDarknet('D:/AU_LabResearch/code/darknet/aug_datas/models/au_lab_yolov4.cfg','D:/AU_LabResearch/code/darknet/aug_datas/models/au_lab_yolov4_best.weights')
        
        # Get all layers from the yolo network
        # Loop and find the last layer (output layer) of the yolo network 
        yolo_layers = yolo_model.getLayerNames()
        yolo_output_layer = [yolo_layers[yolo_layer - 1] for yolo_layer in yolo_model.getUnconnectedOutLayers()]
        
        # input preprocessed blob into model and pass through the model
        yolo_model.setInput(img_blob)
        # obtain the detection layers by forwarding through till the output layer
        obj_detection_layers = yolo_model.forward(yolo_output_layer)
        
        
        ############## NMS Change 1 ###############
        # initialization for non-max suppression (NMS)
        # declare list for [class id], [box center, width & height[], [confidences]
        class_ids_list = []
        boxes_list = []
        confidences_list = []
        distance = [] 
        ############## NMS Change 1 END ###########
        
        
        # loop over each of the layer outputs
        for object_detection_layer in obj_detection_layers:
        	# loop over the detections
            
            
            for object_detection in object_detection_layer:
                
                # obj_detections[1 to 4] => will have the two center points, box width and box height
                # obj_detections[5] => will have scores for all objects within bounding box
                all_scores = object_detection[5:]
                predicted_class_id = np.argmax(all_scores)
                prediction_confidence = all_scores[predicted_class_id]
            
            
            
                
                # take only predictions with confidence more than 60%
                if prediction_confidence > 0.90:
                    #get the predicted label
                    predicted_class_label = class_labels[predicted_class_id]
                    #obtain the bounding box co-oridnates for actual image from resized image size
                    bounding_box = object_detection[0:4] * np.array([img_width, img_height, img_width, img_height])
                    (box_center_x_pt, box_center_y_pt, box_width, box_height) = bounding_box.astype("int")
                    start_x_pt = int(box_center_x_pt - (box_width / 2))
                    start_y_pt = int(box_center_y_pt - (box_height / 2))
                    
                    ############## NMS Change 2 ###############
                    #save class id, start x, y, width & height, confidences in a list for nms processing
                    #make sure to pass confidence as float and width and height as integers
                    class_ids_list.append(predicted_class_id)
                    confidences_list.append(float(prediction_confidence))
                    boxes_list.append([start_x_pt, start_y_pt, int(box_width), int(box_height)])
                    ############## NMS Change 2 END ###########
                    
        ############## NMS Change 3 ###############
        # Applying the NMS will return only the selected max value ids while suppressing the non maximum (weak) overlapping bounding boxes      
        # Non-Maxima Suppression confidence set as 0.5 & max_suppression threhold for NMS as 0.4 (adjust and try for better perfomance)
        max_value_ids = cv2.dnn.NMSBoxes(boxes_list, confidences_list, 0.5, 0.4)
        
        #Apply Logic
            
        # loop through the final set of detections remaining after NMS and draw bounding box and write text
        for max_valueid in max_value_ids:
                    max_class_id = max_valueid
                    box = boxes_list[max_class_id]
                    start_x_pt = box[0]
                    start_y_pt = box[1]
                    box_width = box[2]
                    box_height = box[3]
                    
                    #get the predicted class id and label
                    predicted_class_id = class_ids_list[max_class_id]
                    predicted_class_label = class_labels[predicted_class_id]
                    prediction_confidence = confidences_list[max_class_id]
                ############## NMS Change 3 END ########### 
                           
                    end_x_pt = start_x_pt + box_width
                    end_y_pt = start_y_pt + box_height
                    
                    #get a random mask color from the numpy array of colors
                    box_color = class_colors[predicted_class_id]
                    
                    #convert the color numpy array as a list and apply to text and box
                    box_color = [int(c) for c in box_color]
                    
                    
                  
            
                    #control code
                    
                    if predicted_class_label == 'flag':
                        distance = self.findDistance(FLAG_WIDTH, focal_Length_flag, box_width)
                    elif predicted_class_label == 'obstacle':
                        distance = self.findDistance(OBSTACLE_WIDTH, focal_Length_obstacle, box_width)
                        
                    
                    
                    
                    
                    
                   # print the prediction in console
                    predicted_class_label = "{}: {:.2f}%, distance: {}".format(predicted_class_label, prediction_confidence * 100, distance)
                    print("predicted object {}".format(predicted_class_label))
                    print(f'box width: {box_width}')
                    print(f'box height: {box_height}')
                    print(f'start_x: {start_x_pt}')
                    print(f'start_y: {start_y_pt}')
                    print(f'end_x_pt: {end_x_pt}')
                    print(f'end_y_pt: {end_y_pt}')
                    
                    
                    # draw rectangle and text in the image
                    cv2.rectangle(img_to_detect, (start_x_pt, start_y_pt), (end_x_pt, end_y_pt), box_color, 1)
                    cv2.putText(img_to_detect, predicted_class_label, (start_x_pt, start_y_pt-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1)
                    
        
        
        
        return max_value_ids, class_ids_list, boxes_list, confidences_list, img_to_detect, class_labels, class_colors
    
    def initBot(self, bot):
                
        #test drive
        cnt = 0
        radian = 0
        angle = 0
        while True:
            # Packet 100 contains all sensor data.
            bot.drive_direct(-30, 30)
            
            
            sensor = bot.get_sensors()
            print(f'sensor left: {sensor.encoder_counts_left}')
            print(f'sensor right: {sensor.encoder_counts_right}')
            print(f'distance: {sensor.distance}' )
            
            angle = sensor.angle/0.324056
            print(f'angle :{angle}')
            
            time.sleep(.01)

        cnt += 1
        
        
    
    def sendCommandRaw(self, command):
        global connection

        try:
            if connection is not None:
                assert isinstance(command, bytes), 'Command must be of type bytes'
                connection.write(command)
                connection.flush()
            else:
                print("Not connected.")
        except serial.SerialException:
            print("Lost connection")
    
            connection = None

        
    def driveCommand(self, velocity, rotation):
        
            vr = int(velocity + (rotation / 2))
            vl = int(velocity - (rotation / 2))
            
            cmd = struct.pack(">Bhh", 145, vr, vl)
            if cmd != self.callbackKeyLastDriveCommand:
                self.sendCommandRaw(cmd)
                self.callbackKeyLastDriveCommand = cmd                
            
            
    def findFocal(self, measured_distance, item_width, width_from_camera):
      
          focal_length = (measured_distance * width_from_camera)/item_width
          print(f'focal_length: {focal_length}')
          return focal_length

    def findDistance(self, item_width, focal_length, width_from_camera):
      
          distance = (item_width * focal_length)/width_from_camera
          
          print(f'distance: {distance}')
          
          return distance
    
    def onConnect(self):
        
        ports = self.getSerialPorts()
        print('Port List\n' + '\n'.join(ports))
        print("enter port to connect:") 
        port_name = str(input())
        return port_name
    
    
    def getSerialPorts(self):
        
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i + 1) for i in range(256)]

        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this is to exclude your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')

        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')

        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result    



if __name__ == "__main__":
    app = AU_Research_Irobot()
    
    

