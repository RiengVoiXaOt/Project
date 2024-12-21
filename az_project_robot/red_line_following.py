import cv2
import numpy as np
from time import sleep, time
from src.hardware.servos import ServoControl
from src.utils.control_utils import set_motors_direction

# Khởi tạo servo
servo_1 = ServoControl(channel=1)  # Servo 1
servo_2 = ServoControl(channel=0)  # Servo 2
DEFAULT_ANGLE = 100  # Góc mặc định
MIN_ANGLE = 0  # Giới hạn góc nhỏ nhất
MAX_ANGLE = 120

# Cấu hình camera
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Độ rộng khung hình
cap.set(4, 480)  # Độ cao khung hình

# Các biến để theo dõi trạng thái
prev_time = time()
is_tracking = False

try:
    while True:
        # Đọc khung hình từ camera
        ret, frame = cap.read()
        if not ret:
            print("Không thể đọc khung hình!")
            break

        # Chuyển đổi khung hình sang không gian màu HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Định nghĩa khoảng màu đỏ trong không gian HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Tạo mặt nạ cho màu đỏ
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.add(mask1, mask2)

        # Tìm contour trong mặt nạ
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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
                if cx < 50:  # Nếu nằm bên trái
                    print("Quay trái")
                    set_motors_direction('rotate_left', 0.1, 0, 0)
                    
                elif cx > 350:  # Nếu nằm bên phải
                    print("Quay phải")
                    set_motors_direction('rotate_right', 0.1, 0, 0)
                else:  # Nếu nằm ở giữa
                    print("Đi thẳng")
                    set_motors_direction('go_forward', 0.1, 0, 0)

            # Vẽ contour
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
        else:
            print("Không thấy đường line")
            set_motors_direction('stop', 0, 0, 0)

        # Hiển thị khung hình
        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Chương trình đã dừng.")
finally:
    cap.release()
    cv2.destroyAllWindows()
