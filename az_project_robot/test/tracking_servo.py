import cv2
import numpy as np
from gpiozero import AngularServo
from time import sleep
from src.utils.image_utils import analyze_contours, display_info

# Cấu hình servo sử dụng AngularServo
servo_x = AngularServo(15, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.5/1000)
servo_y = AngularServo(14, min_angle=0, max_angle=180, min_pulse_width=0.5/1000, max_pulse_width=2.4/1000)

# Đặt góc ban đầu
servo_x.angle = 90  # Góc trung tâm
servo_y.angle = 90
sleep(1)

# Cấu hình camera
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Chiều rộng khung hình
cap.set(4, 480)  # Chiều cao khung hình

# Giới hạn góc di chuyển
X_MIN_ANGLE = 0
X_MAX_ANGLE = 180
Y_MIN_ANGLE = 0
Y_MAX_ANGLE = 180

# Khởi tạo trạng thái góc hiện tại
current_x_angle = 90
current_y_angle = 90

# Thông số cho việc phân tích contour
MIN_BOX_AREA = 500  # Diện tích tối thiểu để phát hiện đối tượng
CENTER_X = 320
CENTER_Y = 240

try:
    while True:
        _, frame = cap.read()
        if frame is None:
            print("Không thể đọc khung hình!")
            break

        # Chuyển đổi sang không gian màu HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Lọc màu đỏ
        low_red = np.array([161, 155, 84])
        high_red = np.array([179, 255, 255])
        red_mask = cv2.inRange(hsv_frame, low_red, high_red)

        # Phân tích contour
        status, deviation_x, deviation_y, contours, x, y, w, h = analyze_contours(red_mask, CENTER_X, CENTER_Y, MIN_BOX_AREA, "red")

        # Theo dõi vật thể màu đỏ
        if status:
            # Tính góc mới
            move_x_angle = current_x_angle - (deviation_x * 0.1)  # Hệ số 0.1 để điều chỉnh độ nhạy
            move_y_angle = current_y_angle - (deviation_y * 0.1)

            # Giới hạn góc di chuyển
            move_x_angle = max(X_MIN_ANGLE, min(X_MAX_ANGLE, move_x_angle))
            move_y_angle = max(Y_MIN_ANGLE, min(Y_MAX_ANGLE, move_y_angle))

            # Cập nhật góc servo
            servo_x.angle = move_x_angle
            servo_y.angle = move_y_angle

            # Lưu trạng thái góc hiện tại
            current_x_angle = move_x_angle
            current_y_angle = move_y_angle

        # Vẽ thông tin lên frame
        display_info(frame, 30, [status], [(deviation_x, deviation_y)], CENTER_X, CENTER_Y)  # Giả sử FPS là 30
        cv2.imshow("Frame", frame)
        cv2.imshow("Red Mask", red_mask)

        # Nhấn 'q' để thoát
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
