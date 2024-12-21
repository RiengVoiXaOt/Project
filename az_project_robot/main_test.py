import cv2
import numpy as np
from time import sleep, time
from src.utils.image_utils import process_frame, calculate_fps, analyze_contours, display_info, detection_callback
from src.hardware.servos import ServoControl, MIN_ANGLE, MAX_ANGLE, DEFAULT_ANGLE
from src.utils.control_utils import set_motors_direction
# from src.hardware.ultrasonic import UltrasonicSensors

servo_1 = ServoControl(channel=1)  # Servo 1
servo_2 = ServoControl(channel=0)  # Servo 2

# Cấu hình camera
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # Độ rộng khung hình
cap.set(4, 480)  # Độ cao khung hình
# Ultrasonic = UltrasonicSensors()
# Cấu hình phân tích contour
MIN_BOX_AREA = 500
CENTER_X = 320
CENTER_Y = 240

# Biến lưu trạng thái và tính FPS
prev_time = time()
is_tracking = False
is_moving = False
is_docking = False

# Lưu trữ lịch sử góc servo
servo_angle_history_1 = []  # Lưu trữ góc servo 1
servo_angle_history_2 = []  # Lưu trữ góc servo 2
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
    else:
        print("Servo đã ổn định, không cần quay xe.")

try:
    target_angle_1 = DEFAULT_ANGLE  # Góc mặc định cho servo 1
    target_angle_2 = DEFAULT_ANGLE  # Góc mặc định cho servo 2

    # Vòng lặp chờ cho đến khi có ít nhất 3 giá trị trong servo_angle_history_1
    while len(servo_angle_history_1) < 3:
        # Đọc khung hình từ camera
        _, frame = cap.read()
        if frame is None:
            print("Không thể đọc khung hình!")
            break

        # Xử lý khung hình
        mask_red, _ = process_frame(frame)
        status, deviation_x, deviation_y, contours, x, y, w, h = analyze_contours(
            mask_red, CENTER_X, CENTER_Y, MIN_BOX_AREA, "red"
        )

        # Điều chỉnh góc servo 1 và lưu trữ vào lịch sử
        servo_1.move_to_angle(target_angle_1)
        servo_angle_history_1.append(target_angle_1)
        if len(servo_angle_history_1) > MAX_HISTORY:
            servo_angle_history_1.pop(0)

        # Hiển thị thông tin lên khung hình
        cv2.putText(frame, f'Servo 1 Angle: {target_angle_1}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Bắt đầu vòng lặp chính khi có đủ 3 giá trị
    while True:
        # Đọc khung hình từ camera
        _, frame = cap.read()
        if frame is None:
            print("Không thể đọc khung hình!")
            break

        # Xử lý khung hình
        mask_red, _ = process_frame(frame)
        status, deviation_x, deviation_y, contours, x, y, w, h = analyze_contours(
            mask_red, CENTER_X, CENTER_Y, MIN_BOX_AREA, "red"
        )

        # Tính FPS
        current_time = time()
        fps = calculate_fps(prev_time, current_time, 1)
        prev_time = current_time

        # Hiển thị thông tin lên khung hình
        statuses = [status]
        deviations = [(deviation_x, deviation_y)]
        display_info(frame, fps, statuses, deviations, CENTER_X, CENTER_Y)

        # Hiển thị góc hiện tại của servo 1 và servo 2
        cv2.putText(frame, f'Servo 1 Angle: {target_angle_1}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        cv2.putText(frame, f'Servo 2 Angle: {target_angle_2}', (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

        if status:  # Nếu phát hiện đối tượng
            is_tracking = True

            # Điều chỉnh góc servo 1 để theo dõi vật thể
            if deviation_x < -20:
                target_angle_1 += 1
                if target_angle_1 <= MAX_ANGLE:
                    servo_1.move_to_angle(target_angle_1)
                    servo_angle_history_1.append(target_angle_1)
                    if len(servo_angle_history_1) > MAX_HISTORY:
                        servo_angle_history_1.pop(0)
                    sleep(0.05)

            elif deviation_x > 20:
                target_angle_1 -= 1
                if target_angle_1 >= MIN_ANGLE:
                    servo_1.move_to_angle(target_angle_1)
                    servo_angle_history_1.append(target_angle_1)
                    if len(servo_angle_history_1) > MAX_HISTORY:
                        servo_angle_history_1.pop(0)
                    sleep(0.05)

            # Điều chỉnh góc servo 2 để theo dõi vật thể
            if deviation_y < -20:
                target_angle_2 -= 1
                if target_angle_2 <= MAX_ANGLE:
                    servo_2.move_to_angle(target_angle_2)
                    servo_angle_history_2.append(target_angle_2)
                    if len(servo_angle_history_2) > MAX_HISTORY:
                        servo_angle_history_2.pop(0)
                    sleep(0.05)

            elif deviation_y > 20:
                target_angle_2 += 1
                if target_angle_2 >= MIN_ANGLE:
                    servo_2.move_to_angle(target_angle_2)
                    servo_angle_history_2.append(target_angle_2)
                    if len(servo_angle_history_2) > MAX_HISTORY:
                        servo_angle_history_2.pop(0)
                    sleep(0.05)

            # Kiểm tra điều kiện di chuyển
            if len(servo_angle_history_1) >= 3:
                last_three_angles = servo_angle_history_1[-3:]  # Lấy 3 góc gần nhất
                if all(59 <= angle <= 61 for angle in last_three_angles):  # Kiểm tra khoảng
                    average_angle = sum(last_three_angles) / len(last_three_angles)  # Tính trung bình
                    print(f"Trung bình góc: {average_angle:.2f}")
                    print("Servo 1 ổn định, robot bắt đầu di chuyển!")
                    set_motors_direction('go_forward', 0.4, 0, 0)
                    is_docking = True
                else:
                    print("Servo 1 chưa ổn định, chờ thêm.")

            # Kiểm tra và quay robot nếu cần
            if abs(deviation_x) < 20:
                rotate_robot(target_angle_1)

        else:
            is_tracking = False
            if is_docking or is_moving:
                set_motors_direction('stop', 0, 0, 0)
                is_docking = False

        # Kiểm tra cảm biến siêu âm
        # front_distance = Ultrasonic.get_distance("front")
        # if front_distance <= 15:
        #     set_motors_direction('stop', 0, 0, 0)
        #     sleep(5)
        #     print("Xe đã tới gần vật, dừng lại.")
        #     print(front_distance)

        # Đưa servo về góc mặc định nếu không phát hiện vật
        if not status:
            if target_angle_1 != DEFAULT_ANGLE:
                target_angle_1 = DEFAULT_ANGLE
                servo_1.move_to_angle(target_angle_1)
            if target_angle_2 != DEFAULT_ANGLE:
                target_angle_2 = DEFAULT_ANGLE
                servo_2.move_to_angle(target_angle_2)

        cv2.imshow("Frame", frame)
        cv2.imshow("Red Mask", mask_red)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
