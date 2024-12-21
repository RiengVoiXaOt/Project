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


try:
    angle_1 = DEFAULT_ANGLE  # Góc mặc định cho servo 1
    angle_2 = 90  # Góc mặc định cho servo 2
    servo_2.move_to_angle(angle_2)
    while True:
        # Đọc khung hình từ camera
        ret, frame = cap.read()
        if not ret:
            print("Không thể đọc khung hình!")
            break

        # Chuyển đổi khung hình sang không gian màu HSV
        mask_red, _ = process_frame(frame)
        # Tìm contour trong mặt nạ
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
                if cx < 100:  # Nếu nằm bên trái
                    print("Quay trái")
                    set_motors_direction('rotate_left', 0.1, 0, 0)
                    
                elif cx > 450:  # Nếu nằm bên phải
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
