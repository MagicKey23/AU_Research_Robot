import cv2
import os

# Delcare where to save the pictures
saving_path = 'D:/dataset'

# Open the video file
cap = cv2.VideoCapture('C:/Users/knguyen/Pictures/Camera Roll/box.mp4')
# Loop through the video
file_number = 301
while(cap.isOpened()):

    # Get a frame
    ret,frame = cap.read()
    if ret == False:
        break
    # Save the frame to the path
    file_name = f'Irobot_data{file_number}.jpg'
    cv2.imwrite(os.path.join(saving_path, file_name), frame)
    file_number += 1

    print(f'Saved file: {file_name}')

    # Skip 5 frames. First number is inclusive, second number is exclusive
    for x in range(1,6):
        ret, frame = cap.read()

cap.release()
cv2.destroyAllWindows()