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
from src.utils.image_utils import (
    load_labels, load_model, get_model_details, detect_objects, draw_detections,
    preprocess_frame, calculate_fps, analyze_detection
)
import warnings
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
def object_detection_loop(videostream, stop_event, frame_queue, target_label):
    labels = load_labels(PATH_TO_LABELS)
    interpreter = load_model(PATH_TO_CKPT)
    input_details, output_details, height, width, floating_model = get_model_details(interpreter)
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()

    while not stop_event.is_set():
        t1 = cv2.getTickCount()
        frame = videostream.read()

        if frame is None:
            print("Cannot read frame from videostream.")
            break

        input_data = preprocess_frame(frame, width, height, floating_model)
        detections = detect_objects(interpreter, input_data, input_details, output_details, MIN_CONF_THRESHOLD, IM_WIDTH, IM_HEIGHT)
        status, deviation_x, deviation_y, *_ = analyze_detection(detections, target_label, labels, IM_WIDTH, IM_HEIGHT, CENTER_X, CENTER_Y)
        frame = draw_detections(frame, detections, labels, MIN_CONF_THRESHOLD, IM_WIDTH, IM_HEIGHT)

        # Vẽ hai đường thẳng qua trung tâm
        center_color = (0, 255, 0)  # Màu xanh lá
        cv2.line(frame, (CENTER_X, 0), (CENTER_X, IM_HEIGHT), center_color, 1)  # Đường thẳng đứng qua trung tâm
        cv2.line(frame, (0, CENTER_Y), (IM_WIDTH, CENTER_Y), center_color, 1)  # Đường ngang qua trung tâm

        # In giá trị deviation_x và deviation_y lên khung hình
        deviation_text = f"Deviation X: {deviation_x:.2f}, Deviation Y: {deviation_y:.2f}"
        cv2.putText(frame, deviation_text, (10, IM_HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Hiển thị FPS
        frame_rate_calc = calculate_fps(t1, cv2.getTickCount(), freq)
        cv2.putText(frame, f'FPS: {frame_rate_calc:.2f}', (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2, cv2.LINE_AA)

        if not frame_queue.full():
            frame_queue.put((status, deviation_x, deviation_y, frame))
# Hàm tìm kiếm đối tượng
def search_for_object(servo_1, servo_2, frame_queue, num_turns=4, step_angle=30):
    """
    Hàm tìm kiếm đối tượng bằng cách quay servo xung quanh từ 0 đến 120 độ.
    
    Args:
        servo_1: Servo điều khiển góc quay theo chiều ngang.
        servo_2: Servo điều khiển góc quay theo chiều dọc.
        frame_queue: Hàng đợi chứa khung hình để kiểm tra phát hiện đối tượng.
        num_turns: Số vòng quay tối đa để tìm kiếm.
        step_angle: Góc thay đổi mỗi lần quay servo (có thể điều chỉnh).
        
    Returns:
        (target_angle_1, target_angle_2) nếu đối tượng được phát hiện, (None, None) nếu không.
    """
    # Khởi tạo góc quay
    target_angle_1 = 0  # Góc khởi đầu cho servo 1
    target_angle_2 = 80  # Góc khởi đầu cho servo 2

    MAX_ANGLE = 120  # Giới hạn góc tối đa
    MIN_ANGLE = 0    # Giới hạn góc tối thiểu

    for turn in range(num_turns):
        print(f"Vòng tìm kiếm {turn + 1}/{num_turns}")

        # Quay servo đến góc hiện tại
        servo_1.move_to_angle(target_angle_1)
        servo_2.move_to_angle(target_angle_2)
        sleep(1)  # Chờ một chút để servo ổn định

        # Kiểm tra có phát hiện đối tượng không từ frame_queue
        if not frame_queue.empty():
            status, _, _, _ = frame_queue.get()  # Lấy thông tin từ hàng đợi

            # Nếu có phát hiện vật thể, dừng việc quay và trả về góc hiện tại
            if status:
                print("Đối tượng đã được phát hiện.")
                return target_angle_1, target_angle_2  # Trả về góc của servo

        # Cập nhật góc quay
        target_angle_1 += step_angle

        # Kiểm tra xem góc có vượt quá giới hạn không
        if target_angle_1 > MAX_ANGLE:
            target_angle_1 = MAX_ANGLE  # Giới hạn ở góc tối đa
        elif target_angle_1 < MIN_ANGLE:
            target_angle_1 = MIN_ANGLE  # Giới hạn ở góc tối thiểu

    print("Không phát hiện được đối tượng trong vòng tìm kiếm.")
    return None, None  # Không tìm thấy đối tượng

# Điều khiển robot quay
def rotate_robot(target_angle, vx, vy):
    if target_angle < DEFAULT_ANGLE - 2:
        set_motors_direction('rotate_right', vx, vy, 1)
        sleep(0.1)
        set_motors_direction('stop', vx, vy, 0)
    elif target_angle > DEFAULT_ANGLE + 2:
        set_motors_direction('rotate_left', vx, vy, 1)
        sleep(0.1)
        set_motors_direction('stop', vx, vy, 0)
def go_right_or_left(target_angle, vx, vy):
    if target_angle < DEFAULT_ANGLE - 2:
        set_motors_direction('go_right', vx, vy, 1)
        sleep(0.1)
        set_motors_direction('stop', vx, vy, 0)
    elif target_angle > DEFAULT_ANGLE + 2:
        set_motors_direction('go_left', vx, vy, 1)
        sleep(0.1)
        set_motors_direction('stop', vx, vy, 0)
lost_time = 0  # Biến lưu trữ thời gian mất dấu đối tượng liên tục
MAX_LOST_TIME = 60  # Thời gian tối đa (trong số khung hình) được phép mất dấu (2 giây với 30 FPS)
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

                    if deviation_x < -10 and target_angle_1 < MAX_ANGLE:
                        target_angle_1 += 1
                        servo_1.move_to_angle(target_angle_1)
                    elif deviation_x > 10 and target_angle_1 > MIN_ANGLE:
                        target_angle_1 -= 1
                        servo_1.move_to_angle(target_angle_1)

                    if deviation_y < -10 and target_angle_2 > MIN_ANGLE:
                        target_angle_2 -= 1
                        servo_2.move_to_angle(target_angle_2)
                    elif deviation_y > 10 and target_angle_2 < MAX_ANGLE:
                        target_angle_2 += 1
                        servo_2.move_to_angle(target_angle_2)

                    # Kiểm tra điều kiện di chuyển
                    elif  51 < target_angle_1 < 69 and abs(deviation_x) < 10:
                        if front_distance < 15:
                            set_motors_direction('stop', vx, vy, 0)
                        else:
                            set_motors_direction('go_forward', vx, vy, 0)
                            is_moving = True
                            is_docking = True
                    else:
                        print("Servo 1 chưa ổn định, chờ thêm.")

                    # Kiểm tra và quay robot nếu cần
                    if abs(deviation_x) > 40:
                        if right_distance > 15 and left_distance > 15:
                            go_right_or_left(target_angle_1, vx, vy)
                        elif 20 > abs(deviation_x) < 40:
                            rotate_robot(target_angle_1, vx, vy)

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