import cv2
import numpy as np
from src.utils.control_utils import set_motors_direction

# Khởi tạo webcam
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

if not camera.isOpened():
    print("Error: Could not open the webcam.")
    exit()

while True:
    # Lấy khung hình từ webcam
    ret, frame = camera.read()
    if not ret:
        print("Error: Could not read a frame from the webcam.")
        break

    # Chuyển đổi khung hình sang không gian màu HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Định nghĩa dải màu đỏ trong không gian HSV
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # Tạo mặt nạ cho màu đỏ
    red_mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = red_mask1 | red_mask2

    # Tìm và xử lý các contour
    contours, _ = cv2.findContours(red_mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Lấy contour lớn nhất (giả sử là line đỏ)
        largest_contour = max(contours, key=cv2.contourArea)
        blackbox = cv2.minAreaRect(largest_contour)
        (x_min, y_min), (w_min, h_min), ang = blackbox

        # Tính toán góc và sai số
        if ang < -45:
            ang = 90 + ang
        if w_min < h_min and ang > 0:
            ang = (90 - ang) * -1
        if w_min > h_min and ang < 0:
            ang = 90 + ang

        setpoint = 320  # Trung tâm ngang của khung hình
        error = int(x_min - setpoint)
        ang = int(ang)

        # Vẽ hình chữ nhật và hiển thị góc và sai số
        box = cv2.boxPoints(blackbox)
        box = np.intp(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)
        cv2.putText(frame, f"Angle: {ang}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, f"Error: {error}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.line(frame, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)

        # Xác định hướng di chuyển
        
    set_motors_direction("go_left", 0.1, 0.1, 0)

    # Hiển thị khung hình
    cv2.imshow("Red Line Tracking", frame)

    # Dừng vòng lặp khi nhấn 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng tài nguyên
camera.release()
cv2.destroyAllWindows()
