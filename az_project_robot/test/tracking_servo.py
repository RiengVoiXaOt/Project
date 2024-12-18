import cv2
import numpy as np
from gpiozero import AngularServo
from time import sleep, time
from src.utils.image_utils import process_frame, calculate_fps, analyze_contours, display_info

# Cấu hình servo sử dụng AngularServo
servo_x = AngularServo(15, min_angle=-90, max_angle=90, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servo_y = AngularServo(14, min_angle=-90, max_angle=90, min_pulse_width=0.5/1000, max_pulse_width=2.4/1000)

# Đặt góc ban đầu
servo_x.angle = 0  # Góc trung tâm
servo_y.angle = 0
sleep(1)

# Cấu hình camera
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Chiều rộng khung hình
cap.set(4, 480)  # Chiều cao khung hình

# Thông số cho việc phân tích contour
MIN_BOX_AREA = 500  # Diện tích tối thiểu để phát hiện đối tượng
CENTER_X = 320
CENTER_Y = 240

# Bộ lọc trung bình động cho góc servo
filtered_x_angle = 0
filtered_y_angle = 0
previous_deviation_x = 0
previous_deviation_y = 0
sensitivity_x = 0.1
sensitivity_y = 0.1

try:
    prev_time = time()
    frame_count = 0

    while True:
        _, frame = cap.read()
        if frame is None:
            print("Không thể đọc khung hình!")
            break

        # Xử lý khung hình để tạo mask
        mask_red, mask_black = process_frame(frame)

        # Phân tích contour cho màu đỏ
        status, deviation_x, deviation_y, contours, x, y, w, h = analyze_contours(mask_red, CENTER_X, CENTER_Y, MIN_BOX_AREA, "red")

        # Theo dõi vật thể màu đỏ
        if status:
            # Tính toán độ nhạy
            dx = deviation_x - previous_deviation_x
            dy = deviation_y - previous_deviation_y
            sensitivity_x = min(0.5, max(0.05, abs(dx) / 100))
            sensitivity_y = min(0.5, max(0.05, abs(dy) / 100))

            # Lọc góc servo bằng trung bình động
            filtered_x_angle = 0.8 * filtered_x_angle + 0.2 * (servo_x.angle + deviation_x * sensitivity_x)
            filtered_y_angle = 0.8 * filtered_y_angle + 0.2 * (servo_y.angle + deviation_y * sensitivity_y)

            # Cập nhật góc servo
            servo_x.angle = max(-90, min(90, filtered_x_angle))
            servo_y.angle = max(-90, min(90, filtered_y_angle))

            # Lưu giá trị độ lệch trước đó
            previous_deviation_x = deviation_x
            previous_deviation_y = deviation_y

        # Tính toán FPS
        frame_count += 1
        current_time = time()
        if current_time - prev_time >= 1:  # Mỗi giây
            fps = calculate_fps(prev_time, current_time, frame_count)
            print(f"FPS: {fps:.2f}")
            prev_time = current_time
            frame_count = 0

        # Vẽ thông tin lên frame
        display_info(frame, 30, [status], [(deviation_x, deviation_y)], CENTER_X, CENTER_Y)  # Giả sử FPS là 30
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.imshow("Frame", frame)
        cv2.imshow("Red Mask", mask_red)
        cv2.imshow("Black Mask", mask_black)

        # Nhấn 'q' để thoát
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
