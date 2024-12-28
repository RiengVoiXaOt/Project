import cv2
import time
import threading
from queue import Queue
from src.utils.image_utils import (
    process_frame,
    analyze_contours,
    display_info,
    calculate_fps
)
from src.hardware.servos import ServoControl, MIN_ANGLE, MAX_ANGLE, DEFAULT_ANGLE
from src.utils.control_utils import set_motors_direction
from src.vision.video_stream import VideoStream
from time import sleep
from src.vision.color_detection import color_detection_loop
from threading import Thread, Event, Lock
from src.hardware.servos import ServoControl

a = ServoControl(channel=0)
a.move_to_angle(90)

def main():
    # Khởi tạo VideoStream
    videostream = VideoStream(resolution=(640, 480), framerate=30).start()
    time.sleep(1)

    # Tạo hàng đợi và sự kiện dừng
    frame_queue = Queue(maxsize=1)
    stop_event = threading.Event()

    # Tạo và bắt đầu luồng cho nhận diện màu sắc
    color_thread = Thread(target=color_detection_loop, args=(videostream, 320, 240, 1000, stop_event, frame_queue, 10), daemon=True)
    color_thread.start()

    # Hiển thị kết quả trong thread chính
    while not stop_event.is_set():
        if not frame_queue.empty():
            # Lấy dữ liệu từ hàng đợi
            frame, mask_red, mask_yellow, status_red, deviation_x_red,deviation_y_red, status_yellow,deviation_x_yellow,deviation_y_yellow,contours_yellow,contours_red,_,_,_,_,_,_,_ = (frame_queue.get())

            # Hiển thị các khung hình
            if frame is not None:
                cv2.imshow("Red Mask", frame)

            # Nếu phát hiện đối tượng
            if status_yellow:
                if abs(deviation_x_yellow) or abs(deviation_y_yellow) < 180:
                    set_motors_direction('rotate_left', 0.1, 0.1, 0)
                if deviation_y_yellow > 30 and deviation_x_yellow == 0:
                    set_motors_direction('rotate_right', 0.1, 0.1, 0)
            if not status_yellow:
                set_motors_direction('go_forward', 0.1, 0.1, 0)
        # Dừng chương trình nếu nhấn phím 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            set_motors_direction('stop', 0, 0, 0)
            stop_event.set()
            break

    # Đợi cho các luồng hoàn thành
    color_thread.join()
    videostream.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()