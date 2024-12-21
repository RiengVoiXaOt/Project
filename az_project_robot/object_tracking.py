import os
import cv2
import numpy as np
import time
from threading import Thread, Event
from time import sleep
from queue import Queue
from tensorflow.lite.python.interpreter import Interpreter
from src.hardware.ultrasonic import UltrasonicSensors
from src.vision.video_stream import VideoStream
from src.hardware.servos import ServoControl, MIN_ANGLE, MAX_ANGLE, DEFAULT_ANGLE
from src.utils.control_utils import set_motors_direction
import warnings
from src.vision.object_detection import object_detection_loop
from src.utils.control_utils import search_for_object, rotate_robot, go_right_or_left
warnings.filterwarnings("ignore", category=UserWarning, module='cv2')

# Định nghĩa các hằng số và thông số toàn cụcs
MODEL_NAME = '/home/az/Desktop/Project/az_project_robot/models'
GRAPH_NAME = 'detect.tflite'
LABELMAP_NAME = 'labelmap.txt'
MIN_CONF_THRESHOLD = 0.7
IM_WIDTH, IM_HEIGHT = 640, 480
CENTER_X, CENTER_Y = IM_WIDTH // 2, IM_HEIGHT // 2
CWD_PATH = os.getcwd()
PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, GRAPH_NAME)
PATH_TO_LABELS = os.path.join(CWD_PATH, MODEL_NAME, LABELMAP_NAME)

ultrasonic_sensors=UltrasonicSensors()
vx = 0.2
vy = 0.2
# Hàm nhận diện đối tượng
lost_time = 0  # Biến lưu trữ thời gian mất dấu đối tượng liên tục
MAX_LOST_TIME = 200  # Thời gian tối đa (trong số khung hình) được phép mất dấu (2 giây với 30 FPS)
not_tracking_time = 0  # Thời gian không theo dõi (tính bằng giây)
TRACKING_TIMEOUT = 10  # Thời gian tối đa không theo dõi trước khi tìm kiếm lại

# Hàm chính
def main():
    stop_event = Event()
    frame_queue = Queue(maxsize=1)
    videostream = VideoStream(resolution=(IM_WIDTH, IM_HEIGHT), framerate=30)
    videostream.start()  # Đảm bảo rằng luồng video đã bắt đầu
    time.sleep(1)

    # Khởi tạo góc cho servo
    target_angle_1 = DEFAULT_ANGLE
    target_angle_2 = DEFAULT_ANGLE

    front_distance = ultrasonic_sensors.get_distance("front")
    left_distance = ultrasonic_sensors.get_distance("left")
    right_distance = ultrasonic_sensors.get_distance("right")
    try:
        object_thread = Thread(target=object_detection_loop, args=(videostream, stop_event, frame_queue, "Water"), daemon=True)
        object_thread.start()

        servo_1, servo_2 = ServoControl(channel=1), ServoControl(channel=0)
        is_tracking = False
        lost_time = 0  # Biến tính thời gian khi mất đối tượng
        search_count = 0  # Đếm số lần quay tìm kiếm

        while not stop_event.is_set():
            if not frame_queue.empty():
                status, deviation_x, deviation_y, frame = frame_queue.get()
                if frame is not None:
                    cv2.imshow("Object Detection", frame)
                else:
                    print("Frame is None, skipping...")  # Thông báo nếu khung hình là None

                # In giá trị góc của servo
                print(f"Servo 1 Angle: {target_angle_1}, Servo 2 Angle: {target_angle_2}")

                if status:
                    is_tracking = True
                    lost_time = 0  # Reset thời gian khi tìm thấy đối tượng
                    servo_1.tracking_servo_bottom(deviation_x, target_angle_1)
                    servo_2.tracking_servo_bottom(deviation_y, target_angle_2)
                    if front_distance > 15:
                    # Kiểm tra điều kiện di chuyển
                        if  51 < target_angle_1 < 69 and abs(deviation_x) < 10:
                            set_motors_direction('go_forward', vx, vy, 0)
                            is_moving = True
                            is_docking = True
                        else:
                            print("Servo 1 chưa ổn định, chờ thêm.")
                    elif front_distance < 15:
                        set_motors_direction('stop', vx, vy, 0)
                    # Kiểm tra và quay robot nếu cần
                    if abs(deviation_x) > 40:
                        if right_distance > 15 and left_distance > 15:
                            go_right_or_left(target_angle_1, vx, vy, DEFAULT_ANGLE, 4)
                        elif 20 > abs(deviation_x) < 40:
                            rotate_robot(target_angle_1, vx, vy, DEFAULT_ANGLE, 4)

                if not status:
                    lost_time += 1
                    if lost_time > MAX_LOST_TIME:
                        print("Mất dấu lâu hơn, bắt đầu tìm kiếm lại.")
                        lost_time = 0
                        # Gọi hàm tìm kiếm đối tượng và cập nhật góc nếu phát hiện
                        new_angle_1, new_angle_2 = search_for_object(servo_1, servo_2, frame_queue)
                        if new_angle_1 is not None and new_angle_2 is not None:
                            target_angle_1 = new_angle_1
                            target_angle_2 = new_angle_2

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    stop_event.set()
                    break

        object_thread.join()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        stop_event.set()
        videostream.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()