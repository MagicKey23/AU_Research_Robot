#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 27 May 2015
# updated 1 April 2020 for Python3

###########################################################################
# Copyright (c) 2015-2020 iRobot Corporation#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
#   Neither the name of iRobot Corporation nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###########################################################################

from tkinter import *
import tkinter.messagebox
import tkinter.simpledialog
import time
import struct
import sys
import glob  # for listing serial ports
import numpy as np
import cv2
import math

try:
    import serial
except ImportError:
    tkinter.messagebox.showerror('Import error', 'Please install pyserial.')
    raise

connection = None

TEXTWIDTH = 40  # window width, in characters
TEXTHEIGHT = 16  # window height, in lines

KNOWN_DISTANCE = 42.0
FLAG_WIDTH = 3.0
OBSTACLE_WIDTH = 18.4

VELOCITYCHANGE = 30
ROTATIONCHANGE = 70  # 800

helpText = """\
Supported Keys:
P\tPassive
S\tSafe
F\tFull
C\tClean
D\tDock
R\tReset
Space\tBeep
Arrows\tMotion
A\tAutonomous
Q\tStop Autonomous
If nothing happens after you connect, try pressing 'P' and then 'S' to get into safe mode.
"""


class TetheredDriveApp(Tk):
    # static variables for keyboard callback -- I know, this is icky
    callbackKeyUp = False
    callbackKeyDown = False
    callbackKeyLeft = False
    callbackKeyRight = False
    callbackKeyLastDriveCommand = ''
    runAutonomous = False
    motionChange = False
    rotation360 = 5.2  # number of second to rotate
    rottation90 = 1.3
    success = False
    isCenter = False

    def __init__(self):
        Tk.__init__(self)
        self.title("iRobot Create 2 Tethered Drive")
        self.option_add('*tearOff', FALSE)

        self.menubar = Menu()
        self.configure(menu=self.menubar)

        createMenu = Menu(self.menubar, tearoff=False)
        self.menubar.add_cascade(label="Create", menu=createMenu)

        createMenu.add_command(label="Connect", command=self.onConnect)
        createMenu.add_command(label="Help", command=self.onHelp)
        createMenu.add_command(label="Quit", command=self.onQuit)

        self.text = Text(self, height=TEXTHEIGHT, width=TEXTWIDTH, wrap=WORD)
        self.scroll = Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=self.scroll.set)
        self.text.pack(side=LEFT, fill=BOTH, expand=True)
        self.scroll.pack(side=RIGHT, fill=Y)

        self.text.insert(END, helpText)

        self.bind("<Key>", self.callbackKey)
        self.bind("<KeyRelease>", self.callbackKey)

    def findFocal(self, measured_distance, item_width, width_from_camera):

        focal_length = (measured_distance * width_from_camera)/item_width
        print(f'focal_length: {focal_length}')
        return focal_length

    def findDistance(self, item_width, focal_length, width_from_camera):

        distance = (item_width * focal_length)/width_from_camera

        print(f'distance: {distance}')

        return distance

    # sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send

    def sendCommandASCII(self, command):
        cmd = bytes([int(v) for v in command.split()])
        self.sendCommandRaw(cmd)

    # sendCommandRaw takes a string interpreted as a byte array
    def sendCommandRaw(self, command):
        global connection

        try:
            if connection is not None:
                assert isinstance(
                    command, bytes), 'Command must be of type bytes'
                connection.write(command)
                connection.flush()
            else:
                print("Not connected.")
        except serial.SerialException:
            print("Lost connection")

            connection = None

        seq = ' '.join([str(c) for c in command])
        self.text.insert(END, ' '.join([str(c) for c in command]))
        self.text.insert(END, '\n')
        self.text.see(END)

    # getDecodedBytes returns a n-byte value decoded using a format string.
    # Whether it blocks is based on how the connection was set up.
    def getDecodedBytes(self, n, fmt):
        global connection

        try:
            return struct.unpack(fmt, connection.read(n))[0]
        except serial.SerialException:
            print("Lost connection")
            tkinter.messagebox.showinfo(
                'Uh-oh', "Lost connection to the robot!")
            connection = None
            return None
        except struct.error:
            print("Got unexpected data from serial port.")
            return None

    # get8Unsigned returns an 8-bit unsigned value.
    def get8Unsigned(self):
        return getDecodedBytes(1, "B")

    # get8Signed returns an 8-bit signed value.
    def get8Signed(self):
        return getDecodedBytes(1, "b")

    # get16Unsigned returns a 16-bit unsigned value.
    def get16Unsigned(self):
        return getDecodedBytes(2, ">H")

    # get16Signed returns a 16-bit signed value.
    def get16Signed(self):
        return getDecodedBytes(2, ">h")

    def driveCommand(self, velocity, rotation):

        vr = int(velocity + (rotation / 2))
        vl = int(velocity - (rotation / 2))

        cmd = struct.pack(">Bhh", 145, vr, vl)
        if cmd != self.callbackKeyLastDriveCommand:
            self.sendCommandRaw(cmd)
            self.callbackKeyLastDriveCommand = cmd

    def object_detection(self, img_to_detect, focal_Length_flag, focal_Length_obstacle):

        img_height = img_to_detect.shape[0]
        img_width = img_to_detect.shape[1]

        # convert to blob to pass into model
        img_blob = cv2.dnn.blobFromImage(
            img_to_detect, 0.003922, (320, 320), swapRB=True, crop=False)

        # set of 80 class labels
        class_labels = ["flag", "obstacle"]

        # Declare List of colors as an array
        #Green, Blue, Red, cyan, yellow, purple
        # Split based on ',' and for every split, change type to int
        # convert that to a numpy array to apply color mask to the image numpy array
        class_colors = ["0,255,0", "0,0,255"]
        class_colors = [np.array(every_color.split(",")).astype(
            "int") for every_color in class_colors]
        class_colors = np.array(class_colors)
        class_colors = np.tile(class_colors, (16, 1))

        # Loading pretrained model
        # input preprocessed blob into model and pass through the model
        # obtain the detection predictions by the model using forward() method
        yolo_model = cv2.dnn.readNetFromDarknet('D:/AU_LabResearch/code/darknet/aug_datas/models/au_lab_yolov4.cfg',
                                                'D:/AU_LabResearch/code/darknet/aug_datas/models/au_lab_yolov4_best.weights')

        # Get all layers from the yolo network
        # Loop and find the last layer (output layer) of the yolo network
        yolo_layers = yolo_model.getLayerNames()
        yolo_output_layer = [yolo_layers[yolo_layer - 1]
                             for yolo_layer in yolo_model.getUnconnectedOutLayers()]

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
                if prediction_confidence > 0.70:
                    # get the predicted label
                    predicted_class_label = class_labels[predicted_class_id]
                    # obtain the bounding box co-oridnates for actual image from resized image size
                    bounding_box = object_detection[0:4] * np.array(
                        [img_width, img_height, img_width, img_height])
                    (box_center_x_pt, box_center_y_pt, box_width,
                     box_height) = bounding_box.astype("int")
                    start_x_pt = int(box_center_x_pt - (box_width / 2))
                    start_y_pt = int(box_center_y_pt - (box_height / 2))

                    ############## NMS Change 2 ###############
                    # save class id, start x, y, width & height, confidences in a list for nms processing
                    # make sure to pass confidence as float and width and height as integers
                    class_ids_list.append(predicted_class_id)
                    confidences_list.append(float(prediction_confidence))
                    boxes_list.append(
                        [start_x_pt, start_y_pt, int(box_width), int(box_height)])
                    ############## NMS Change 2 END ###########

        ############## NMS Change 3 ###############
        # Applying the NMS will return only the selected max value ids while suppressing the non maximum (weak) overlapping bounding boxes
        # Non-Maxima Suppression confidence set as 0.5 & max_suppression threhold for NMS as 0.4 (adjust and try for better perfomance)
        max_value_ids = cv2.dnn.NMSBoxes(
            boxes_list, confidences_list, 0.5, 0.4)

        # Apply Logic

        # loop through the final set of detections remaining after NMS and draw bounding box and write text
        for max_valueid in max_value_ids:
            max_class_id = max_valueid
            box = boxes_list[max_class_id]
            start_x_pt = box[0]
            start_y_pt = box[1]
            box_width = box[2]
            box_height = box[3]

            # get the predicted class id and label
            predicted_class_id = class_ids_list[max_class_id]
            predicted_class_label = class_labels[predicted_class_id]
            prediction_confidence = confidences_list[max_class_id]
            ############## NMS Change 3 END ###########

            end_x_pt = start_x_pt + box_width
            end_y_pt = start_y_pt + box_height

            # get a random mask color from the numpy array of colors
            box_color = class_colors[predicted_class_id]

            # convert the color numpy array as a list and apply to text and box
            box_color = [int(c) for c in box_color]

            # control code

            if predicted_class_label == 'flag':
                distance = self.findDistance(
                    FLAG_WIDTH, focal_Length_flag, box_width)
            elif predicted_class_label == 'obstacle':
                distance = self.findDistance(
                    OBSTACLE_WIDTH, focal_Length_obstacle, box_width)

           # print the prediction in console
            predicted_class_label = "{}: {:.2f}%, distance: {}".format(
                predicted_class_label, prediction_confidence * 100, distance)
            print("predicted object {}".format(predicted_class_label))
            print(f'box width: {box_width}')
            print(f'box height: {box_height}')
            print(f'start_x: {start_x_pt}')
            print(f'start_y: {start_y_pt}')
            print(f'end_x_pt: {end_x_pt}')
            print(f'end_y_pt: {end_y_pt}')

            # draw rectangle and text in the image
            cv2.rectangle(img_to_detect, (start_x_pt, start_y_pt),
                          (end_x_pt, end_y_pt), box_color, 1)
            cv2.putText(img_to_detect, predicted_class_label, (start_x_pt,
                        start_y_pt-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 1)

        return max_value_ids, class_ids_list, boxes_list, confidences_list, img_to_detect, class_labels, class_colors


    def reCentering(self, start_x):
        if start_x >= 300:
           self.applyRotationRight()
        elif start_x < 300:
           self.applyRotationLeft()
        

    def applyRotationLeft(self):
        rotation = 0
        rotation += ROTATIONCHANGE
        self.driveCommand(10, rotation)

    def applyRotationRight(self):
        rotation = 0
        rotation -= ROTATIONCHANGE
        self.driveCommand(10, rotation)

    def driveForward(self):
        velocity = 0
        velocity += VELOCITYCHANGE
        self.driveCommand(velocity, 0)

    def driveBackward(self):
        velocity = 0
        velocity -= VELOCITYCHANGE
        self.driveCommand(velocity, 0)

    def obstacle_avoidance(self, start_x):

        print("in obstacle avoidance")

        if start_x >= 230:

            print("apply left to right")
            # Left to Right
            start_time = time.time()
            while (time.time() - start_time) <= 4:
                self.driveCommand(0, 50)
            self.driveCommand(0, 0)

            # go Straight
            start_time = time.time()
            while (time.time() - start_time) <= 7:
                self.driveCommand(100, 0)
            self.driveCommand(0, 0)

            # rotate left
            start_time = time.time()
            while (time.time() - start_time) <= 6:
                self.driveCommand(0, -50)
            self.driveCommand(0, 0)
            # go Straight
            start_time = time.time()
            while (time.time() - start_time) <= 8:
                self.driveCommand(100, 0)
            self.driveCommand(0, 0)

            return "RIGHT", False

        elif start_x < 230:

            # right to left

            start_time = time.time()
            while (time.time() - start_time) <= 4:
                self.driveCommand(0, -50)
            self.driveCommand(0, 0)

            start_time = time.time()
            while (time.time() - start_time) <= 7:
                self.driveCommand(100, 0)  # to the right
            self.driveCommand(0, 0)

            start_time = time.time()
            while (time.time() - start_time) <= 6:
                self.driveCommand(0, 50)
            self.driveCommand(0, 0)

            start_time = time.time()
            while (time.time() - start_time) <= 8:
                self.driveCommand(100, 0)  # to the right
            self.driveCommand(0, 0)

            return "LEFT", False

        return "LEFT", True

    def go_to_flag(self, start_x, end_y):
        # maintain center point and go to the flag
        direction_scan = "LEFT"
        if  end_y >= 480:
            print("stop the robot")
            self.driveCommand(0, 0)
            return True, "LEFT"
        if start_x >= 300:
            direction_scan = "RIGHT"
        elif start_x < 300:
            direction_scan = "LEFT"
        self.driveForward()
        
        return False, direction_scan



    def checkLeft(self, x_obstacles, x_flags):
        # closest_obstacle is an index
        closest_obstacle = 0
        flag_on_left = False

        for index, x_obstacle in enumerate(x_obstacles):
            if x_flags[0] < x_obstacle:
                flag_on_left = True
                if x_obstacle < x_obstacles[closest_obstacle]:
                    closest_obstacle = index
            else:
                flag_on_left = False

        return flag_on_left, closest_obstacle

    def checkRight(self, x_obstacles, x_flags):
        closest_obstacle = 0
        flag_on_right = False

        for index, x_obstacle in enumerate(x_obstacles):
            if x_flags[0] > x_obstacle:
                flag_on_right = True
                if x_obstacle > x_obstacles[closest_obstacle]:
                    closest_obstacle = index
            else:
                flag_on_right = False

        return flag_on_right, closest_obstacle

    def checkCenter(self, x_obstacles, x_flags):
        closest_obstacle_left = 0
        closest_obstacle_right = 0

        flag_center = False

        for index, x_obstacle in enumerate(x_obstacles):
            # Find left side of the flag
            if x_flags[0] > x_obstacle:
                if x_obstacle > x_obstacles[closest_obstacle_left]:
                    closest_obstacle_left = index
            # Find right side of the flag
            elif x_flags[0] < x_obstacle:
                if x_obstacle < x_obstacles[cloest_obstacle_right]:
                    closest_obstacle_right = index

        if x_flags[0] > closest_obstacle_left and x_flags[0] < closest_obstacle_right:
            flag_center = True
        else:
            flag_center = False

        return flag_center, closest_obstacle_left, closest_obstacle_right

    def calculateGap(self, x_obstacles, end_x_obstacles, x_flags, end_x_flags, direction, obstacle_close_left, obstacle_close_right):
        #calculate gap based on the direction of the flag
        if direction == "left":
            return x_obstacles[obstacle_close_right] - end_x_flags
        elif direction == "right":
            return x_flags - end_x_obstacles[obstacle_close_left]
        elif direction == "center":
            return x_obstalces[obstacle_close_right]- end_x_obstacles[obstacle_close_left]
        
        return 0

    def multipleObject(self, max_value_ids, class_labels, class_ids_list, boxes_list, focal_Length_flag, focal_Length_obstacle, flag_Found):
        print("in multiple")

        flag_indexes = []
        distance_flags = []
        x_flags = []
        y_flags = []
        box_width_flags = []
        box_height_flags = []
        end_x_flags = []
        end_y_flags = []

        obstacle_indexes = []
        distance_obstacles = []
        x_obstacles = []
        y_obstacles = []
        box_width_obstacles = []
        box_height_obstacles = []
        end_x_obstacles = []
        end_y_obstacles = []

        flagClose = False
        flag_Found = flag_Found
        
        destination_reach = False
        direction_scan = "LEFT"
        
        # find the flag in the max value ID
        for object_index in max_value_ids:
            # search for flag and store index in its own list
            predicted_data = class_ids_list[object_index]

            if class_labels[predicted_data] == "flag":
                flag_indexes.append(object_index)
            elif class_labels[predicted_data] == "obstacle":
                obstacle_indexes.append(object_index)
        # if flag index exist, compare the distance

        # get distance of the object
        if len(flag_indexes) > 0:

            for flag_index in flag_indexes:
                data = boxes_list[flag_index]
                start_x = data[0]
                start_y = data[1]
                box_width = data[2]
                box_height = data[3]
                end_x = start_x + box_width
                end_y = start_y + box_height
                distance = self.findDistance(
                    FLAG_WIDTH, focal_Length_flag, box_width)
                distance_flags.append(distance)
                x_flags.append(start_x)
                y_flags.append(start_y)
                end_y_flags.append(end_y)
                end_x_flags.append(end_x)
                box_width_flags.append(box_width)
                box_height_flags.append(box_height)

        if len(obstacle_indexes) > 0:

            for obstacle_index in obstacle_indexes:
                data = boxes_list[obstacle_index]
                start_x = data[0]
                start_y = data[1]
                box_width = data[2]
                box_height = data[3]
                
                end_x = start_x + box_width
                end_y = start_y + box_height
               
                distance = distance = self.findDistance(
                    OBSTACLE_WIDTH, focal_Length_obstacle, box_width)
                distance_obstacles.append(distance)
                x_obstacles.append(start_x)
                y_obstacles.append(start_y)
                end_x_obstacles.append(end_x)
                end_y_obstacles.append(end_y)
                
                box_width_obstacles.append(box_width)
                box_height_obstacles.append(box_height)

        # check flag

        if flag_indexes:
            # check distance
            for distance_obstacle in distance_obstacles:
                if distance_flags[0] < distance_obstacle:
                    print("flag close true")
                    flagClose = True
                    flag_Found = True
                else:
                    flagClose = False
                    print("flag close false")

            # go to the flag
            if flagClose:
                
                destination_reach, direction_scan = self.go_to_flag(x_flags[0], end_y_flags[0])
                self.reCentering(x_flags[0])                
            else:
                # grab position of the flag
                #RETURN TRUE VALUE WHERE THE FLAG IS LOCATED AT 
                #OBSTACLE CLOSE RETURN INDEX VALUE
                flag_at_left, obstacle_close_right = self.checkLeft(x_obstacles, x_flags)
                
                if not flag_at_left:
                    flag_at_right, obstacle_close_left = self.checkRight(
                        x_obstacles, x_flags)
                    
                    if not flag_at_right:
                        flag_at_center, obstacle_close_left, obstacle_close_right = self.checkCenter(
                            x_obstacles, x_flags)
                        

                flag_position = box_width_flags[0]

                # check the space if possible to travel

                # 3 scenario

                # Two Item

                # check left

                if flag_at_left:
                    # perform left functionality
                    gap = self.calculateGap(
                        x_obstacles, end_x_obstacles, x_flags[0], end_x_flags[0], "left", 0, obstacle_close_right)
                    if gap > 65:
                        
                        destination_reach, direction_scan = self.go_to_flag(x_flags[0], end_y_flags[0])
                        self.reCentering(x_flags[0])
                    #else:
                        # drive to the left
                # check right                        
                elif flag_at_right:
                    # perform right functionality
                    gap = self.calculateGap(
                        x_obstacles, end_x_obstacles, x_flags[0], end_x_flags[0], "right", obstacle_close_left, 0)
                    if gap > 83:
                        destination_reach, direction_scan = self.go_to_flag(x_flags[0], end_y_flags[0])
                        self.reCentering(x_flags[0])
                    #else:
                        # drive to the right
                        
                # check center
                elif flag_at_center:
                    # perform center functionality
                    gap = self.calculateGap(
                        x_obstacles, end_x_obstacles, x_flags[0], end_x_flags[0], "center", obstacle_close_left, obstacle_close_right)
                    if gap > 145:
                        destination_reach, direction_scan = self.go_to_flag(x_flags[0], end_y_flags[0])
                        self.reCentering(x_flags[0])
                    #else:
                        
        return direction_scan, destination_reach, flag_Found
                    

    def singleObject(self, max_value_ids, class_labels, class_ids_list, boxes_list, focal_Length_flag, focal_Length_obstacle):

        value_id = max_value_ids[0]

        predicted_data = class_ids_list[value_id]

        data = boxes_list[value_id]

        box_width = data[2]
        start_x = data[0]
        start_y = data[1]
        box_height = data[3]
        end_y = box_height + start_y
        end_x = box_width + start_x
        distance = 0
        if class_labels[predicted_data] == "flag":
            distance = self.findDistance(
                FLAG_WIDTH, focal_Length_flag, box_width)
        elif class_labels[predicted_data] == "obstacle":
            distance = self.findDistance(
                OBSTACLE_WIDTH, focal_Length_obstacle, box_width)

        return class_labels[predicted_data], distance, start_x, box_width, end_x, end_y

    def Test(self):

        data = self.sendCommandASCII('148 2 29 13')

    def initAuto(self):

        # get camera
        get_camera = cv2.VideoCapture(0)

        # rotation
        rotationTick = 0

        # focal length and distance for #obstacle
        focal_Length_obstacle = self.findFocal(
            KNOWN_DISTANCE, OBSTACLE_WIDTH, 190)
        # focal length and distance for #flag
        focal_Length_flag = self.findFocal(KNOWN_DISTANCE, FLAG_WIDTH, 32)

        # flag found
        flag_Found = False
        # destination
        destination_reach = False

        # state
        direction_scan = "LEFT"
        # obstacle avoidance state
        obstacle_state = False
        # Close to the obstacle
        isClose = False

        while True:

            # Get frame
            ret, current_frame = get_camera.read()

            max_value_ids, class_ids_list, boxes_list, confidences_list, img_to_detect, class_labels, class_colors = self.object_detection(
                current_frame, focal_Length_flag, focal_Length_obstacle)

            # Perform Scan
            print(len(max_value_ids))

            if not destination_reach:

                if len(class_ids_list) == 0:
                    # rotate robot if seeing nothing

                    if direction_scan == "LEFT":
                        self.applyRotationLeft()
                    elif direction_scan == "RIGHT":
                        self.applyRotationRight()

                elif len(max_value_ids) > 1:
                    # check for flag and obstacle
                    
                    direction_scan, destination_reach, flag_Found = self.multipleObject(max_value_ids, class_labels, class_ids_list,
                                        boxes_list, focal_Length_flag, focal_Length_obstacle, flag_Found)
                
                    
                    
                elif len(max_value_ids) == 1:
                    # check for flag or obstacle
                    resultLabel, distance, start_x, box_width, end_x, end_y = self.singleObject(
                        max_value_ids, class_labels, class_ids_list, boxes_list, focal_Length_flag, focal_Length_obstacle)

                    # going to the flag:

                    if resultLabel == "flag" and distance >= 12 or flag_Found:
                        # go to flag
                        print("go to the flag")
                        flag_Found = True
                        destination_reach, direction_scan = self.go_to_flag(start_x, end_y)


                    # obstacle avoidance

                    if resultLabel == "obstacle" and rotationTick > 500 and not flag_Found:
                        print(distance)
                        print("do thing")
                        # maintain center point
                        if distance >= 25:
                            if start_x >= 230:
                                print("go right")
                                obstacle_state = "RIGHT"

                            elif start_x < 230:
                                print("go left")
                                obstacle_state = "LEFT"

                            print("go forward")
                            self.driveForward()

                        elif distance < 25 or isClose:
                            isClose = True
                            direction_scan, isClose = self.obstacle_avoidance(
                                start_x)
                            rotationTick = 0

                    elif not flag_Found and rotationTick < 500:

                        if direction_scan == "LEFT":
                            self.applyRotationLeft()
                        elif direction_scan == "RIGHT":
                            self.applyRotationRight()
                else:
                    print("detect nothing")

            # check distance

            # go to certain distance before apply change direction

            # change direction based on x position

            rotationTick += 1
            print(rotationTick)

            cv2.imshow("Detection Output", img_to_detect)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.runAutonomous = False
                self.motionChange = False
                break

        get_camera.release()
        cv2.destroyAllWindows()

    # A handler for keyboard events. Feel free to add more!

    def callbackKey(self, event):
        k = event.keysym.upper()

        if event.type == '2':  # KeyPress; need to figure out how to get constant
            if k == 'P':   # Passive
                self.sendCommandASCII('128')
            elif k == 'S':  # Safe
                self.sendCommandASCII('131')
            elif k == 'F':  # Full
                self.sendCommandASCII('132')
            elif k == 'C':  # Clean
                self.sendCommandASCII('135')
            elif k == 'D':  # Dock
                self.sendCommandASCII('143')
            elif k == 'SPACE':  # Beep
                self.sendCommandASCII('140 3 1 64 16 141 3')
            elif k == 'R':  # Reset
                self.sendCommandASCII('7')
            elif k == 'M':

                # [137] [Velocity high byte] [Velocity low byte] [Radius high byte] [Radius low byte]
                # To drive in reverse at a velocity of -200 mm/s while turning at a radius of 500mm, send the following
                # serial byte sequence:
                #[137] [255] [56] [1] [244]
                # Desired value -> two’s complement and convert to hex -> split into 2 bytes -> convert to decimal
                # Velocity = -200 = 0xFF38 = [0xFF] [0x38] = [255] [56]
                # Radius = 500 = 0x01F4 = [0x01] [0xF4] = [1] [244]
                # self.sendCommandASCII('137 0 200 1 244') #turning command 100  mm/s, 500mm #30 distance x < 10 boxwidth:130-140
                # -100 and -500mm

                #self.sendCommandASCII('137 0 100 156 12')

                # right to left

                start_time = time.time()
                while (time.time() - start_time) <= 4:
                    self.driveCommand(100, -10)  # to the right
                self.driveCommand(0, 0)
                start_time = time.time()
                while (time.time() - start_time) <= 6:
                    self.driveCommand(100, 40)  # to the left
                self.driveCommand(0, 0)

                # left to right

                start_time = time.time()
                while (time.time() - start_time) <= 4:
                    self.driveCommand(100, 10)
                self.driveCommand(0, 0)
                start_time = time.time()
                while (time.time() - start_time) <= 6:
                    self.driveCommand(100, -40)
                self.driveCommand(0, 0)

            elif k == 'N':
                self.Test()
            elif k == 'UP':
                self.callbackKeyUp = True
                self.motionChange = True
            elif k == 'DOWN':
                self.callbackKeyDown = True
                self.motionChange = True
            elif k == 'LEFT':
                self.callbackKeyLeft = True
                self.motionChange = True
            elif k == 'RIGHT':
                self.callbackKeyRight = True
                self.motionChange = True
            elif k == 'A':  # run autonomous mode
                self.runAutonomous = True
                self.motionChange = True
                k == 'Q'
            elif k == 'B':  # Quit autonomous mode
                self.runAutonomous = False
            else:
                print("not handled", repr(k))

        elif event.type == '3':  # KeyRelease; need to figure out how to get constant
            if k == 'UP':
                self.callbackKeyUp = False
                self.motionChange = True
            elif k == 'DOWN':
                self.callbackKeyDown = False
                self.motionChange = True
            elif k == 'LEFT':
                self.callbackKeyLeft = False
                self.motionChange = True
            elif k == 'RIGHT':
                self.callbackKeyRight = False
                self.motionChange = True
            elif k == 'Q':
                self.runAutonomous = False
                self.motionChange = True

        # Autonomous Running

        while self.motionChange and self.runAutonomous:
            self.initAuto()

        # start streaming sensor

        # regular running
        if self.motionChange == True:
            velocity = 0
            velocity += VELOCITYCHANGE if self.callbackKeyUp is True else 0
            velocity -= VELOCITYCHANGE if self.callbackKeyDown is True else 0
            rotation = 0
            rotation += ROTATIONCHANGE if self.callbackKeyLeft is True else 0
            rotation -= ROTATIONCHANGE if self.callbackKeyRight is True else 0
            # compute left and right wheel velocities
            vr = int(velocity + (rotation/2))
            vl = int(velocity - (rotation/2))

            # create drive command
            cmd = struct.pack(">Bhh", 145, vr, vl)
            if cmd != self.callbackKeyLastDriveCommand:
                self.sendCommandRaw(cmd)
                self.callbackKeyLastDriveCommand = cmd

    def onConnect(self):
        global connection

        if connection is not None:
            tkinter.messagebox.showinfo('Oops', "You're already connected!")
            return

        try:
            ports = self.getSerialPorts()
            port = tkinter.simpledialog.askstring(
                'Port?', 'Enter COM port to open.\nAvailable options:\n' + '\n'.join(ports))
        except EnvironmentError:
            port = tkinter.simpledialog.askstring(
                'Port?', 'Enter COM port to open.')

        if port is not None:
            print("Trying " + str(port) + "... ")
            try:
                connection = serial.Serial(port, baudrate=115200, timeout=1)
                print("Connected!")
                tkinter.messagebox.showinfo(
                    'Connected', "Connection succeeded!")
            except:
                print("Failed.")
                tkinter.messagebox.showinfo(
                    'Failed', "Sorry, couldn't connect to " + str(port))

    def onHelp(self):
        tkinter.messagebox.showinfo('Help', helpText)

    def onQuit(self):
        if tkinter.messagebox.askyesno('Really?', 'Are you sure you want to quit?'):
            self.destroy()

    def getSerialPorts(self):
        """Lists serial ports
        From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of available serial ports
        """
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i + 1) for i in range(256)]

        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this is to exclude your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')

        elif sys.platform.startswith('dacarwin'):
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
    app = TetheredDriveApp()
    app.mainloop()
