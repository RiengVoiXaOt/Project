from robot.modes import RobotModes
from vision.video_stream import VideoStream
from vision.color_detection import color_detection_loop
from vision.object_detection import object_detection_loop
from threading import Event, Thread
from queue import Queue
import cv2

def main():
    robot_modes = RobotModes()

    while True:
        command = input("Enter 'm' for manual, 'a' for automatic, or 'q' to quit: ")
        if command == "m":
            robot_modes.switch_mode()
            robot_modes.manual_control()
        elif command == "a":
            robot_modes.switch_mode()
            robot_modes.automatic_mode()
        elif command == "q":
            break

if __name__ == "__main__":
    main()



def control_rule_auto(robot, vx, vy, theta, a_n, delta):
    front_distance, left_distance, right_distance, f_state, l_state, r_state = sensor_distance()
    
    if a_n == 0:  # Chế độ tự động
        print("--------------------------------")
        print(f"Tốc độ hiện tại: {current_speed}")
        print("Tự động chạy")
        print(f"Cảm biến trước {f_state}, Cảm biến trái {l_state}, Cảm biến phải {r_state}")
        print(f"Cảm biến trước {front_distance}, Cảm biến trái {left_distance}, Cảm biến phải {right_distance}")

        # Nếu không có vật cản phía trước
        if f_state == 0:
            front_distance_history.clear()
            left_distance_history.clear()
            right_distance_history.clear()
            
            if l_state == 0 and r_state == 0:
                action = 'go_forward'
                action_msg = "đi thẳng"
            elif r_state == 1 and right_distance < 15:
                action = 'go_left'
                action_msg = "đi trái"
            elif l_state == 1 and left_distance < 15:
                action = 'go_right'
                action_msg = "đi phải"
            elif l_state == 1:
                action = 'diagonal_up_right'
                action_msg = "Đi chéo lên phải"
            elif r_state == 1:
                action = 'diagonal_up_left'
                action_msg = "Đi chéo lên trái"
            else:
                action = 'stop'
                action_msg = "dừng lại"

            set_motors_direction(robot, action, vx, vy, theta)
            print(action_msg)

        else:  # Có vật cản phía trước
            # Lưu giá trị cảm biến
            if not front_distance_history and not right_distance_history and not left_distance_history:
                front_distance_history.append(min(front_distance, 85) if front_distance < 100 else 85)
                left_distance_history.append(min(left_distance, 85) if left_distance < 100 else 85)
                right_distance_history.append(min(right_distance, 85) if right_distance < 100 else 85)

            # Nếu quá gần vật cản phía trước
            if front_distance <= 12:
                action = 'go_backward'
                action_msg = "đi lùi khi quá gần vật cản phía trước"
            elif l_state == 0 and r_state == 0 and front_distance >= 10:
                action, action_msg = rotate_based_on_side(robot, vx, vy, theta, right_distance, left_distance)
            elif l_state == 1 and r_state == 0 and front_distance >= 10:
                action = 'rotate_right'
                action_msg = "xoay phải"
            elif l_state == 0 and r_state == 1 and front_distance >= 10:
                action = 'rotate_left'
                action_msg = "xoay trái"
            elif l_state == 1 and r_state == 1 and front_distance >= 10:
                action, action_msg = rotate_based_on_side(robot, vx, vy, theta, right_distance, left_distance)

            if action:
                set_motors_direction(robot, action, vx, vy, theta)
                print(action_msg)

def rotate_based_on_side(robot, vx, vy, theta, right_distance, left_distance):
    """Xoay robot dựa trên khoảng cách bên trái và bên phải."""
    if right_distance > left_distance:  # Xoay phải
        while True:
            front_distance, left_distance, right_distance, f_state, l_state, r_state = sensor_distance()
            set_motors_direction(robot, 'rotate_right', vx, vy, theta)
            if front_distance >= right_distance_history[0] - delta:
                set_motors_direction(robot, 'stop', vx, vy, theta)
                sleep(0.3)
                break
        return 'rotate_right', "xoay phải"
    else:  # Xoay trái
        while True:
            front_distance, left_distance, right_distance, f_state, l_state, r_state = sensor_distance()
            set_motors_direction(robot, 'rotate_left', vx, vy, theta)
            if front_distance >= left_distance_history[0] - delta:
                set_motors_direction(robot, 'stop', vx, vy, theta)
                sleep(0.3)
                break
        return 'rotate_left', "xoay trái"
