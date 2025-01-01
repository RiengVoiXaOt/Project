import cv2
import numpy as np
from time import sleep
from src.utils.control_utils import set_motors_direction

# Khởi tạo camera
cap = cv2.VideoCapture(0)

# Trung tâm màn hình


# Ngưỡng diện tích nhỏ nhất của line đỏ để nhận diện
min_box_area = 1000

# Hàm nhận diện màu đỏ
def process_frame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Mask màu đỏ
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask1_red = cv2.inRange(hsv, lower_red, upper_red)
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask2_red = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = mask1_red + mask2_red    

    return mask_red

# Phân tích contours của đối tượng
def analyze_contours(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    status = "No Red Line Detected"
    deviation_x = deviation_y = 0  # Khởi tạo mặc định
    x = y = w = h = 0  # Khởi tạo mặc định
    status = False
    center_x = 320  # giả sử camera có độ phân giải 640x480
    center_y = 240
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_box_area:  # Nếu diện tích lớn hơn ngưỡng tối thiểu
            x, y, w, h = cv2.boundingRect(contour)
            obj_center_x = x + w // 2
            obj_center_y = y + h // 2
            deviation_x = obj_center_x - center_x
            deviation_y = obj_center_y - center_y
            status = True  # Đối tượng đã được phát hiện
            break  # Chỉ xử lý contour đầu tiên phù hợp

    return status, deviation_x, deviation_y, contours, x, y, w, h

# Hàm hiển thị thông tin lên frame
def display_info(frame, status, deviation_x, deviation_y):
    cv2.putText(frame, f"Status: {status}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(frame, f"x: {deviation_x}, y: {deviation_y}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

# Hàm bám line đỏ
def red_line_following(contours):
    frame_center_x = 640 // 2  # Tâm khung hình ngang
    frame_center_y = 480 // 2  # Tâm khung hình dọc

    if contours:
        # Lấy đường viền lớn nhất
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)

        if M["m00"] != 0:
            # Tọa độ trọng tâm của đường viền
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Tính độ lệch so với tâm khung hình
            deviation_x = cx - frame_center_x
            deviation_y = cy - frame_center_y

            # Điều chỉnh theo hướng robot tiếp cận
            if cy > frame_center_y:  # Đường đỏ xuất hiện ở phía trước
                if abs(deviation_x) > 150:  # Độ lệch lớn khỏi trung tâm
                    if deviation_x < 0:  # Đường lệch về bên trái
                        set_motors_direction('rotate_left', 0.1, 0, 0)
                    else:  # Đường lệch về bên phải
                        set_motors_direction('rotate_right', 0.1, 0, 0)
                else:  # Đường gần trung tâm
                    set_motors_direction('go_forward', 0.1, 0, 0)
            else:  # Đường đỏ xuất hiện phía trên (robot cần xoay để căn chỉnh)
                if deviation_x < 0:
                    set_motors_direction('rotate_left', 0.1, 0, 0)
                else:
                    set_motors_direction('rotate_right', 0.1, 0, 0)

        else:
            # Nếu không xác định được trọng tâm, robot tìm đường
            set_motors_direction('rotate_left',0.1, 0, 0)
    else:
        # Không có đường đỏ trong khung hình
        set_motors_direction('go_forward', 0.1, 0, 0)



# Hàm quay đầu khi không còn thấy line đỏ


# Vòng lặp chính
while True:
    ret, frame = cap.read()
    if not ret:
        print("Không thể lấy frame từ camera.")
        break

    # Xử lý frame và nhận diện màu đỏ
    mask_red = process_frame(frame)

    # Phân tích các contours
    status, deviation_x, deviation_y, contours, x, y, w, h = analyze_contours(mask_red)

    # Hiển thị thông tin lên frame
    display_info(frame, status, deviation_x, deviation_y)

    # Bám theo line đỏ
    red_line_following(contours)

    # Nếu không tìm thấy line đỏ, quay đầu
    # Hiển thị frame
    cv2.imshow("Red Line Detection", mask_red)

    # Dừng khi nhấn phím 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng camera và đóng cửa sổ
cap.release()
cv2.destroyAllWindows()
