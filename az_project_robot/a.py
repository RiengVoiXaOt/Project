import cv2
from queue import Queue
from threading import Event
from src.vision.color_detection import color_detection_loop 

videostream = cv2.VideoCapture(0)
frame_queue = Queue(maxsize=1)
stop_event = Event()
center_x, center_y = 320, 240  # Trung tâm khung hình (giả sử kích thước là 640x480)
min_box_area = 500
while True:
    color_detection_loop(videostream, center_x, center_y, min_box_area, stop_event, frame_queue, max_fps=10)
