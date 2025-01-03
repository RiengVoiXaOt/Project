import cv2
import numpy as np
from time import sleep, time
from src.hardware.servos import ServoControl, DEFAULT_ANGLE, MAX_ANGLE, MIN_ANGLE
from src.utils.control_utils import set_motors_direction
from src.utils.image_utils import (
    process_frame,
    analyze_contours,
    display_info,
    calculate_fps
)

# Khởi tạo servo
servo_1 = ServoControl(channel=1)  # Servo 1
servo_2 = ServoControl(channel=0)  # Servo 2

# Cấu hình camera
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Độ rộng khung hình
cap.set(4, 480)  # Độ cao khung hình

# Các biến để theo dõi trạng thái
prev_time = time()
is_tracking = False
is_docking = False
is_line_following = False

# Lưu trữ lịch sử góc servo
servo_angle_history_1 = []  # Lưu trữ góc servo 1
MAX_HISTORY = 10  # Số lượng giá trị cần kiểm tra

def rotate_robot(target_angle):
    if target_angle < DEFAULT_ANGLE - 5:
        set_motors_direction('rotate_right', 0.1, 0, 1)
        sleep(0.1)
        set_motors_direction('stop', 0, 0, 0)
    elif target_angle > DEFAULT_ANGLE + 5:
        set_motors_direction('rotate_left', 0.1, 0, 1)
        sleep(0.1)
        set_motors_direction('stop', 0, 0, 0)

def follow_line(mask_red, frame):
    contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Tìm contour lớn nhất
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        if M["m00"] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            print("CX: {}, CY: {}".format(cx, cy))

            # Vẽ điểm trung tâm
            cv2.circle(frame, (cx, cy), 5, (255, 255, 255), -1)

            # Điều khiển robot dựa trên vị trí của CX
            if cx < 250:  # Nếu nằm bên trái
                print("Quay trái")
                set_motors_direction('rotate_left', 0.1, 0, 0)

            elif cx > 390:  # Nếu nằm bên phải
                print("Quay phải")
                set_motors_direction('rotate_right', 0.1, 0, 0)
            else:  # Nếu nằm ở giữa
                print("Đi thẳng")
                set_motors_direction('go_forward', 0.1, 0, 0)

            # Vẽ contour
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
        else:
            print("Không tính được tâm")
            set_motors_direction('stop', 0, 0, 0)
    else:
        print("Không thấy đường line")
        set_motors_direction('stop', 0, 0, 0)

try:
    target_angle_1 = DEFAULT_ANGLE  # Góc mặc định cho servo 1
    target_angle_2 = DEFAULT_ANGLE  # Góc định sẵn cho servo 2

    while True:
        # Đọc khung hình từ camera
        ret, frame = cap.read()
        if not ret:
            print("Không thể đọc khung hình!")
            break

        # Xử lý khung hình
        mask_red, mask_black = process_frame(frame)

        # Phân tích đường line đỏ
        status_red, deviation_x_red, deviation_y_red, _, _, _, _, _ = analyze_contours(
            mask_red, 320, 240, 500, "red"
        )

        # Hiển thị khung hình
        current_time = time()
        fps = calculate_fps(prev_time, current_time, 1)
        prev_time = current_time

        statuses = [status_red]
        deviations = [(deviation_x_red, deviation_y_red)]
        display_info(frame, fps, statuses, deviations, 320, 240)

        if status_red and not is_line_following:  # Nếu phát hiện line đỏ khi chưa bám line
            is_tracking = True

            # Điều chỉnh góc servo 1 để theo dõi line đỏ
            if deviation_x_red < -20:
                target_angle_1 += 1
                if target_angle_1 <= MAX_ANGLE:
                    servo_1.move_to_angle(target_angle_1)
                    servo_angle_history_1.append(target_angle_1)
                    if len(servo_angle_history_1) > MAX_HISTORY:
                        servo_angle_history_1.pop(0)
                    sleep(0.05)

            elif deviation_x_red > 20:
                target_angle_1 -= 1
                if target_angle_1 >= MIN_ANGLE:
                    servo_1.move_to_angle(target_angle_1)
                    servo_angle_history_1.append(target_angle_1)
                    if len(servo_angle_history_1) > MAX_HISTORY:
                        servo_angle_history_1.pop(0)
                    sleep(0.05)

            # Kiểm tra điều kiện di chuyển
            if len(servo_angle_history_1) >= 3:
                last_three_angles = servo_angle_history_1[-3:]  # Lấy 3 góc gần nhất
                if all(59 <= angle <= 61 for angle in last_three_angles):  # Kiểm tra khoảng
                    print("Servo 1 ổn định, robot bắt đầu di chuyển!")
                    servo_2.move_to_angle(target_angle_2)
                    is_line_following = True

        elif is_line_following:  # Khi đã sẵn sàng bám line
            follow_line(mask_red, frame)

        else:  # Không phát hiện line đỏ
            is_tracking = False
            is_line_following = False
            set_motors_direction('stop', 0, 0, 0)
            if target_angle_1 != DEFAULT_ANGLE:
                target_angle_1 = DEFAULT_ANGLE
                servo_1.move_to_angle(target_angle_1)
            if target_angle_2 != 120:
                target_angle_2 = 120
                servo_2.move_to_angle(target_angle_2)

        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
