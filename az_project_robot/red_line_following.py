import cv2
def search_for_charger(
    servo_1, servo_2, 
    deviation_x, deviation_y, 
    target_angle_1, target_angle_2, 
    vx, vy, 
    front_distance, right_distance, left_distance,
    DEFAULT_ANGLE=0,contours
):
    lost_time = 0  # Đặt lại thời gian nếu tìm thấy mục tiêu
    # Điều chỉnh servo để theo dõi mục tiêu
    servo_1.tracking_servo_bottom(deviation_x, target_angle_1)
    servo_2.tracking_servo_bottom(deviation_y, target_angle_2)

    # Kiểm tra và di chuyển về phía trước
    if front_distance > 15:
        if 51 < target_angle_1 < 69 and abs(deviation_x) < 10:
            print("Di chuyển thẳng về phía mục tiêu.")
            set_motors_direction('go_forward', vx, vy, 0)
        else:
            print("Servo chưa ổn định, chờ thêm.")
    elif front_distance <= 15:
        print("Khoảng cách phía trước quá gần, dừng lại.")
        set_motors_direction('stop', 0, 0, 0)

    # Quay robot nếu cần
    if abs(deviation_x) > 40:
        if right_distance > 15 and left_distance > 15:
            print("Quay sang phải hoặc trái.")
            go_right_or_left(target_angle_1, vx, vy, DEFAULT_ANGLE, 4)
        elif 20 <= abs(deviation_x) < 40:
            print("Quay robot để hiệu chỉnh góc.")
            rotate_robot(target_angle_1, vx, vy, DEFAULT_ANGLE, 4)

    # Xử lý khi phát hiện line đen
    if status_black:
        if abs(deviation_x_black) < 180 or abs(deviation_y_black) < 180:
            print("Quay trái để tránh line đen.")
            set_motors_direction('rotate_left', 0.1, 0.1, 0)
        elif deviation_y_black > 30 and deviation_x_black == 0:
            print("Quay phải để tránh line đen.")
            set_motors_direction('rotate_right', 0.1, 0.1, 0)
    else:
        # Tránh vật cản nếu không phát hiện line đen
        avoid_obstacles()
        red_line_following(contours)
    # Xử lý trạng thái khi phát hiện hoặc không phát hiện mục tiêu
    if status_charger:
        print("Mục tiêu 'charger' được phát hiện, di chuyển đến mục tiêu.")
        move_to_target(servo_1, servo_2, deviation_x, deviation_y, target_angle_1, target_angle_2, vx, vy, front_distance, right_distance, left_distance, status_black, status_charger)
    else:
        lost_time += 1
        print(f"Không phát hiện mục tiêu, thời gian tìm kiếm: {lost_time}s.")
        if lost_time >= 30:  # Tìm kiếm mục tiêu mới sau 30 giây
            print("Tìm kiếm mục tiêu mới.")
            new_angle_1, new_angle_2 = search_for_object(servo_1, servo_2, frame_queue)
            if new_angle_1 is not None and new_angle_2 is not None:
                target_angle_1 = new_angle_1
                target_angle_2 = new_angle_2
                print("Đã cập nhật góc mục tiêu mới.")

def red_line_following(contours):
    if contours:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if cx < 100:  
                set_motors_direction('rotate_left', 0.1, 0, 0)              
            elif cx > 450:  
                set_motors_direction('rotate_right', 0.1, 0, 0)
            else:  
                set_motors_direction('go_forward', 0.1, 0, 0)
    else:
        set_motors_direction('stop', 0, 0, 0)

