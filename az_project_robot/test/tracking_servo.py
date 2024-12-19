import cv2
import numpy as np
from time import sleep, time
from src.utils.image_utils import process_frame, calculate_fps, analyze_contours, display_info
from src.hardware.servos import ServoControl

DEFAULT_ANGLE = 60  # Góc mặc định
MIN_ANGLE = 10  # Giới hạn góc nhỏ nhất
MAX_ANGLE = 120
servo = ServoControl(channel=1)

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

        # Tính góc mới từ độ lệch
        if status:  # Nếu có đối tượng
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

        # Nếu không có đối tượng, reset servo về góc mặc định
        else:
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
