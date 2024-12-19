import cv2
import numpy as np
from time import sleep, time
from src.utils.image_utils import process_frame, calculate_fps, analyze_contours, display_info
from src.hardware.servos import ServoControl
from src.robot.modes import Modes  # Import đối tượng điều khiển robot
from src.utils.control_utils import set_motors_direction

DEFAULT_ANGLE = 60  # Góc mặc định
MIN_ANGLE = 10  # Giới hạn góc nhỏ nhất
MAX_ANGLE = 120
servo = ServoControl(channel=1)

# Khởi tạo đối tượng điều khiển xe
robot = Modes()  # Thay đổi theo thông số thực tế của xe

# Cấu hình camera
cap = cv2.VideoCapture(0)
cap.set(3, 640)
cap.set(4, 480)

# Thông số phân tích contour
MIN_BOX_AREA = 500
CENTER_X = 320
CENTER_Y = 240  # Tâm khung hình theo chiều dọc

# Biến tính FPS
prev_time = time()

# Biến kiểm tra trạng thái xe
is_tracking = False
is_moving = False  # Trạng thái di chuyển của xe
is_docking = False  # Trạng thái xe đang chạy tới vật

# Hàm điều khiển quay robot
def rotate_robot(target_angle):
    if target_angle < DEFAULT_ANGLE - 5:
        # Quay xe sang phải nếu góc servo < 60
        print("Quay xe sang phải!")
        set_motors_direction('rotate_right', 3, 0, 1)  # Xoay sang phải
    elif target_angle > DEFAULT_ANGLE + 5:
        # Quay xe sang trái nếu góc servo > 60
        print("Quay xe sang trái!")
        set_motors_direction('rotate_left', -3, 0, 1)  # Xoay sang trái
    else:
        print("Servo đã ổn định, không cần quay xe.")

try:
    target_angle = DEFAULT_ANGLE  # Khởi tạo góc servo

    while True:
        _, frame = cap.read()
        if frame is None:
            print("Không thể đọc khung hình!")
            break

        mask_red, _ = process_frame(frame)
        status, deviation_x, deviation_y, contours, x, y, w, h = analyze_contours(
            mask_red, CENTER_X, CENTER_Y, MIN_BOX_AREA, "red"
        )

        # Tính FPS
        current_time = time()
        fps = calculate_fps(prev_time, current_time, 1)
        prev_time = current_time

        # Hiển thị thông tin lên khung hình
        statuses = [status]  # Danh sách trạng thái
        deviations = [(deviation_x, deviation_y)]  # Danh sách độ lệch
        display_info(frame, fps, statuses, deviations, CENTER_X, CENTER_Y)

        # Kiểm tra và theo dõi vật thể
        if status:  # Nếu có đối tượng
            is_tracking = True  # Đánh dấu là đang theo dõi vật thể

            # Điều chỉnh góc servo để theo dõi vật thể
            if deviation_x < -10:  # Nếu đối tượng lệch sang trái
                target_angle += 1  # Cộng 1 độ
                if target_angle <= MAX_ANGLE:  # Đảm bảo không vượt quá giới hạn
                    servo.move_to_angle(target_angle)  # Di chuyển servo đến góc mới
                    sleep(0.05)  # Đợi 50ms để servo di chuyển

            elif deviation_x > 10:  # Nếu đối tượng lệch sang phải
                target_angle -= 1  # Trừ 1 độ
                if target_angle >= MIN_ANGLE:  # Đảm bảo không vượt quá giới hạn
                    servo.move_to_angle(target_angle)  # Di chuyển servo đến góc mới
                    sleep(0.05)  # Đợi 50ms để servo di chuyển

            # Điều kiện để xe dừng xoay và chạy thẳng về phía vật
            if abs(deviation_x) < 5 and not is_docking:
                # Xe đã ổn định, bắt đầu chạy thẳng về phía vật
                print("Xe dừng xoay, bắt đầu chạy thẳng về phía vật!")
                set_motors_direction('stop', 0, 0, 0)
                set_motors_direction('go_forward', 5, 0, 0)  # Di chuyển xe về phía vật
                is_docking = True  # Đánh dấu là xe đang chạy về vật

            # Kiểm tra và quay xe khi servo đã ổn định tracking
            if abs(deviation_x) < 15 and is_tracking:
                rotate_robot(target_angle)  # Quay robot dựa trên góc servo

        else:
            is_tracking = False  # Nếu không có đối tượng, dừng tracking

            # Khi không tracking được vật, dừng xe và quay servo tìm lại
            if is_docking or is_moving:
                robot.stop_robot()  # Dừng xe khi mất dấu vật
                servo.servo_move_to_search()  # Quay servo từ min tới max tìm vật
                is_docking = False  # Đặt lại trạng thái docking

        # Kiểm tra cảm biến siêu âm trước để dừng xe khi tới gần vật
        front_distance = robot.ultrasonic_sensors.get_distance("front")
        if front_distance <= 15:  # Nếu khoảng cách <= 15cm
            robot.stop_robot()  # Dừng xe khi tới gần vật
            print("Xe đã tới gần vật, dừng lại.")

        # Nếu không có đối tượng, reset servo về góc mặc định
        if not status:
            if target_angle != DEFAULT_ANGLE:  # Nếu góc không phải là góc mặc định
                target_angle = DEFAULT_ANGLE  # Đặt lại góc về mặc định
                servo.move_to_angle(target_angle)

        cv2.imshow("Frame", frame)
        cv2.imshow("Red Mask", mask_red)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()



