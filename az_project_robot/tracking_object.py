import os
import cv2
import time
import threading
from queue import Queue
from src.utils.image_utils import (
    load_labels,
    load_model,
    get_model_details,
    detect_objects,
    draw_detections,
    preprocess_frame,
    calculate_fps,
    analyze_detection,
    display_info_object
)
from src.hardware.servos import ServoControl, MIN_ANGLE, MAX_ANGLE, DEFAULT_ANGLE
from src.utils.control_utils import set_motors_direction
from src.vision.video_stream import VideoStream
from time import sleep

def tracking_with_object_detection(videostream, stop_event, frame_queue, target_label):
    """
    Cơ chế tracking đối tượng với object detection.
    """
    MODEL_NAME = '/home/az/Desktop/Project/az_project_robot/models'
    GRAPH_NAME = 'detect.tflite'
    LABELMAP_NAME = 'labelmap.txt'
    min_conf_threshold = 0.7
    imW, imH = 640, 480
    center_x, center_y = imW // 2, imH // 2
    min_box_area = 5000

    CWD_PATH = os.getcwd()
    PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, GRAPH_NAME)
    PATH_TO_LABELS = os.path.join(CWD_PATH, MODEL_NAME, LABELMAP_NAME)

    labels = load_labels(PATH_TO_LABELS)
    interpreter = load_model(PATH_TO_CKPT)
    input_details, output_details, height, width, floating_model = get_model_details(interpreter)

    while not stop_event.is_set():
        frame = videostream.read()
        if frame is None:
            print("Cannot read frame from videostream.")
            break

        input_data = preprocess_frame(frame, width, height, floating_model)
        detections = detect_objects(interpreter, input_data, input_details, output_details, min_conf_threshold, imW, imH)

        # Phân tích phát hiện đối tượng
        status, deviation_x, deviation_y, x, y, w, h = analyze_detection(
            detections, target_label, labels, imW, imH, center_x, center_y, min_box_area
        )

        # Vẽ hình chữ nhật quanh đối tượng phát hiện
        if status:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Hiển thị thông tin
        display_info_object(frame, fps=0, status=status, deviations=(deviation_x, deviation_y), center_x=center_x, center_y=center_y, detections=detections, labels=labels)

        # Gửi kết quả qua hàng đợi
        if not frame_queue.full():
            frame_queue.put((None, frame))

def rotate_robot(target_angle):
    if target_angle < DEFAULT_ANGLE - 5:
        set_motors_direction('rotate_right', 0.1, 0, 1)
        sleep(0.1)
        set_motors_direction('stop', 0, 0, 0)
    elif target_angle > DEFAULT_ANGLE + 5:
        set_motors_direction('rotate_left', 0.1, 0, 1)
        sleep(0.1)
        set_motors_direction('stop', 0, 0, 0)
    else:
        print("Servo đã ổn định, không cần quay xe.")

def check_for_object(mask, min_box_area=500):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_box_area:
            return True
    return False

def main():
    videostream = VideoStream(resolution=(640, 480), framerate=30).start()
    time.sleep(1)

    MIN_BOX_AREA = 500
    CENTER_X = 320
    CENTER_Y = 240

    frame_queue = Queue(maxsize=1)
    stop_event = threading.Event()

    tracking_thread = threading.Thread(
        target=tracking_with_object_detection, 
        args=(videostream, stop_event, frame_queue, "target_label"), 
        daemon=True
    )
    tracking_thread.start()

    servo_1 = ServoControl(channel=1)
    servo_2 = ServoControl(channel=0)

    target_angle_1 = DEFAULT_ANGLE
    target_angle_2 = DEFAULT_ANGLE

    is_tracking = False
    is_moving = False

    # Khởi tạo servo
    servo_1 = ServoControl(channel=1)
    servo_2 = ServoControl(channel=0)
    
    servo_angle_history_1 = []
    servo_angle_history_2 = []
    MAX_HISTORY = 10
    
    while not stop_event.is_set():
        if not frame_queue.empty():
            _, frame = frame_queue.get()

            if frame is not None:
                cv2.imshow("Object Detection", frame)

            # Tạm giả định có dữ liệu phát hiện (phần này thay bằng kết quả thực tế từ tracking)
            detection_status = True  # Đây là trạng thái phát hiện mục tiêu
            deviation_x, deviation_y = 10, 5  # Độ lệch mục tiêu

            if detection_status:
                if not is_tracking:
                    print("Bắt đầu theo dõi vật thể.")
                    is_tracking = True

                if deviation_x < -20:
                    target_angle_1 += 1
                elif deviation_x > 20:
                    target_angle_1 -= 1

                if deviation_y < -20:
                    target_angle_2 -= 1
                elif deviation_y > 20:
                    target_angle_2 += 1

                target_angle_1 = max(MIN_ANGLE, min(MAX_ANGLE, target_angle_1))
                target_angle_2 = max(MIN_ANGLE, min(MAX_ANGLE, target_angle_2))

                servo_1.move_to_angle(target_angle_1)
                servo_2.move_to_angle(target_angle_2)

                 # Lưu trữ lịch sử góc servo
                servo_angle_history_1.append(target_angle_1)
                servo_angle_history_2.append(target_angle_2)
                if len(servo_angle_history_1) > MAX_HISTORY:
                    servo_angle_history_1.pop(0)
                if len(servo_angle_history_2) > MAX_HISTORY:
                    servo_angle_history_2.pop(0)
                    
                if len(servo_angle_history_1) >= 3:
                    last_three_angles = servo_angle_history_1[-3:]
                    if all(59 <= angle <= 61 for angle in last_three_angles):
                        print("Servo 1 ổn định, robot bắt đầu di chuyển!")
                        set_motors_direction('go_forward', 0.4, 0, 0)

                # Kiểm tra và quay robot nếu cần
                if abs(deviation_x) < 20:
                    rotate_robot(target_angle_1)
            else:
                if is_tracking:
                    print("Không phát hiện vật thể, bắt đầu tìm kiếm...")
                    is_tracking = False

                if is_moving:
                    print("Dừng di chuyển để kiểm tra...")
                    set_motors_direction('stop', 0, 0, 0)
                    is_moving = False

        if cv2.waitKey(1) & 0xFF == ord('q'):
            servo_1.move_to_angle(60)
            servo_2.move_to_angle(60)
            set_motors_direction('stop', 0, 0, 0)
            stop_event.set()
            break

    tracking_thread.join()
    videostream.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
