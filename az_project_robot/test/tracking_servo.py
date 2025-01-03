import cv2
import numpy as np
from time import sleep, time
from src.utils.image_utils import process_frame, calculate_fps, analyze_contours, display_info, detection_callback
from src.hardware.servos import ServoControl
from src.robot.modes import Modes
from src.utils.control_utils import set_motors_direction

# Cấu hình góc servo
DEFAULT_ANGLE = 60
MIN_ANGLE = 10
MAX_ANGLE = 120
servo = ServoControl(channel=1)

# Khởi tạo đối tượng điều khiển robot
robot = Modes()

# Cấu hình camera
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Độ rộng khung hình
cap.set(4, 480)  # Độ cao khung hình

# Cấu hình phân tích contour
MIN_BOX_AREA = 500
CENTER_X = 320
CENTER_Y = 240

# Biến lưu trạng thái và tính FPS
prev_time = time()
is_tracking = False
is_moving = False
is_docking = False

# Lưu trữ lịch sử góc servo
servo_angle_history = []  # Lưu trữ góc servo gần đây
MAX_HISTORY = 10  # Số lượng giá trị cần kiểm tra

def rotate_robot(target_angle):
    if target_angle < DEFAULT_ANGLE - 5:
        set_motors_direction('rotate_right', 0.2, 0, 1)
    elif target_angle > DEFAULT_ANGLE + 5:
        set_motors_direction('rotate_left', -0.2, 0, 1)
    else:
        print("Servo đã ổn định, không cần quay xe.")

def servo_move_to_search(servo, cap, min_angle, max_angle, step=1, delay=0.05):
    """
    Quay servo từ min tới max và sau đó từ max tới min.
    Nếu phát hiện vật thể màu đỏ trong quá trình này, dừng lại và tiếp tục tracking.
    
    :param servo: Đối tượng điều khiển servo.
    :param cap: Đối tượng camera.
    :param min_angle: Góc tối thiểu của servo.
    :param max_angle: Góc tối đa của servo.
    :param step: Bước quay của servo.
    :param delay: Thời gian tạm dừng giữa các bước quay.
    """
    # Quay từ min tới max
    for angle in range(min_angle, max_angle + 1, step):
        servo.move_to_angle(angle)
        sleep(delay)
        
        # Kiểm tra xem có phát hiện vật hay không
        _, frame = cap.read()
        if frame is None:
            print("Không thể đọc khung hình!")
            break

        mask_red, _ = process_frame(frame)
        status, _, _, _, _, _, _, _ = analyze_contours(
            mask_red, CENTER_X, CENTER_Y, MIN_BOX_AREA, "red"
        )
        
        if status:  # Nếu phát hiện vật thể
            print("Phát hiện vật thể, dừng quay servo.")
            return  # Dừng lại nếu phát hiện vật thể

    # Quay từ max tới min
    for angle in range(max_angle, min_angle - 1, -step):
        servo.move_to_angle(angle)
        sleep(delay)
        
        # Kiểm tra xem có phát hiện vật hay không
        _, frame = cap.read()
        if frame is None:
            print("Không thể đọc khung hình!")
            break

        mask_red, _ = process_frame(frame)
        status, _, _, _, _, _, _, _ = analyze_contours(
            mask_red, CENTER_X, CENTER_Y, MIN_BOX_AREA, "red"
        )
        
        if status:  # Nếu phát hiện vật thể
            print("Phát hiện vật thể, dừng quay servo.")
            return  # Dừng lại nếu phát hiện vật thể

try:
    target_angle = DEFAULT_ANGLE  # Góc mặc định

    while True:
        # Đọc khung hình từ camera
        _, frame = cap.read()
        if frame is None:
            print("Không thể đọc khung hình!")
            break

        # Xử lý khung hình
        mask_red, _ = process_frame(frame)
        status, deviation_x, deviation_y, contours, x, y, w, h = analyze_contours(
            mask_red, CENTER_X, CENTER_Y, MIN_BOX_AREA, "red"
        )

        # Tính FPS
        current_time = time()
        fps = calculate_fps(prev_time, current_time, 1)
        prev_time = current_time

        # Hiển thị thông tin lên khung hình
        statuses = [status]
        deviations = [(deviation_x, deviation_y)]
        display_info(frame, fps, statuses, deviations, CENTER_X, CENTER_Y)

        if status:  # Nếu phát hiện đối tượng
            is_tracking = True

            # Điều chỉnh góc servo để theo dõi vật thể
            if deviation_x < -10:
                target_angle += 1
                if target_angle <= MAX_ANGLE:
                    servo.move_to_angle(target_angle)
                    servo_angle_history.append(target_angle)
                    if len(servo_angle_history) > MAX_HISTORY:
                        servo_angle_history.pop(0)
                    sleep(0.05)

            elif deviation_x > 10:
                target_angle -= 1
                if target_angle >= MIN_ANGLE:
                    servo.move_to_angle(target_angle)
                    servo_angle_history.append(target_angle)
                    if len(servo_angle_history) > MAX_HISTORY:
                        servo_angle_history.pop(0)
                    sleep(0.05)

            # Kiểm tra điều kiện di chuyển
            if abs(deviation_x) < 10 and not is_docking:
                if len(servo_angle_history) == MAX_HISTORY and all(57 <= angle <= 63 for angle in servo_angle_history):
                    print("Servo ổn định, robot bắt đầu di chuyển!")
                    set_motors_direction('go_forward', 0.4, 0, 0)
                    is_docking = True
                else:
                    print("Servo chưa ổn định, chờ thêm.")

            # Kiểm tra và quay robot nếu cần
            if abs(deviation_x) < 15:
                rotate_robot(target_angle)

        else:
            is_tracking = False
            if is_docking or is_moving:
                set_motors_direction('stop', 0, 0, 0)
                servo_move_to_search(servo, cap, MIN_ANGLE, MAX_ANGLE)
                is_docking = False

        # Kiểm tra cảm biến siêu âm
        front_distance = robot.ultrasonic_sensors.get_distance("front")
        if front_distance <= 15:
            set_motors_direction('stop', 0, 0, 0)
            print("Xe đã tới gần vật, dừng lại.")

        # Đưa servo về góc mặc định nếu không phát hiện vật
        if not status:
            if target_angle != DEFAULT_ANGLE:
                target_angle = DEFAULT_ANGLE
                servo.move_to_angle(target_angle)

        cv2.imshow("Frame", frame)
        cv2.imshow("Red Mask", mask_red)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
