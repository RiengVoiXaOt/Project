import cv2
import numpy as np
from gpiozero import AngularServo
from time import sleep, time

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

# Thông số khởi tạo
MIN_BOX_AREA = 500
TARGET_BOX_AREA = MIN_BOX_AREA
CENTER_X = 320
CENTER_Y = 240
low_red = np.array([161, 155, 84])
high_red = np.array([179, 255, 255])

# Bộ lọc trung bình động
filtered_x_angle = 0
filtered_y_angle = 0

# Giá trị cho điều chỉnh tự động
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

        # Chuyển đổi khung hình sang HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Tạo mask màu đỏ
        mask_red = cv2.inRange(hsv_frame, low_red, high_red)

        # Tìm contour
        contours, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

        status = False
        deviation_x = 0
        deviation_y = 0

        if contours:
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                area = w * h
                if area >= MIN_BOX_AREA:
                    x_medium = int((x + x + w) / 2)
                    y_medium = int((y + y + h) / 2)

                    deviation_x = x_medium - CENTER_X
                    deviation_y = y_medium - CENTER_Y

                    # Trung bình động diện tích để tự động điều chỉnh MIN_BOX_AREA
                    TARGET_BOX_AREA = 0.9 * TARGET_BOX_AREA + 0.1 * area
                    MIN_BOX_AREA = max(100, int(0.7 * TARGET_BOX_AREA))

                    status = True
                    break

        if status:
            # Tự động điều chỉnh độ nhạy
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

        # Vẽ thông tin lên frame
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.line(frame, (CENTER_X, 0), (CENTER_X, 480), (255, 0, 0), 1)
        cv2.line(frame, (0, CENTER_Y), (640, CENTER_Y), (255, 0, 0), 1)

        # Hiển thị frame và mask
        cv2.imshow("Frame", frame)
        cv2.imshow("Red Mask", mask_red)

        # Nhấn 'q' để thoát
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
