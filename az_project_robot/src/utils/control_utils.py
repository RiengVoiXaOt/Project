import sys, tty, termios
from src.hardware.motors  import Motors
from src.hardware.kinematic import Kinematic
from time import sleep

direction = {
    'w': "go_forward",
    's': "go_backward",
    'a': "go_left",
    'd': "go_right",
    'q': "diagonal_up_left",
    'e': "diagonal_up_right",
    'z': "diagonal_down_left",
    'x': "diagonal_down_right",
    '1': "stop",
    '2': "rotate_left",
    '3': "rotate_right"
}

robot = Kinematic(0,0,0,0)
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        command = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return command

def set_motors_direction(command, vx, vy, theta):
    if command == 'go_forward':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = vx, 0, 0, 0
    elif command == 'go_backward':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = -vx, 0, 0, 0
    elif command == 'go_left':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = 0, vy, 0, 0
    elif command == 'go_right':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = 0, -vy, 0, 0
    elif command == 'diagonal_up_left':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = vx, vy, 0, 0
    elif command == 'diagonal_down_left':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = -vx, vy, 0, 0
    elif command == 'diagonal_up_right':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = vx, -vy, 0, 0
    elif command == 'diagonal_down_right':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = -vx, -vy, 0, 0
    elif command == 'stop':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = 0, 0, 0, 0
    elif command == 'rotate_left':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = -vx, 0, 0, 1
    elif command == 'rotate_right':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = vx, 0, 0, 1
    elif command =='stop':
        robot.vxg, robot.vyg, robot.theta_d, robot.turn = 0, 0, 0, 0
    
    motor_controller = Motors()  # Khởi tạo đối tượng Motors
    robot.balancing_velocity()
    robot.backward_kinematics()  # Cập nhật giá trị động cơ dựa trên kinematics

    # Cập nhật tốc độ cho từng động cơ thông qua đối tượng motor_controller
    motor_controller.set_speed("motor_1", max(0, min(abs(robot.v1), 1)))
    motor_controller.set_speed("motor_2", max(0, min(abs(robot.v2), 1)))
    motor_controller.set_speed("motor_3", max(0, min(abs(robot.v3), 1)))
    motor_controller.set_speed("motor_4", max(0, min(abs(robot.v4), 1)))

    # In tốc độ của động cơ để kiểm tra
    print(f"Motor speeds: {robot.v1}, {robot.v2}, {robot.v3}, {robot.v4}")
    
def search_for_object(servo_1, servo_2, frame_queue, num_turns=4, step_angle=30, start_angle_1=0, start_angle_2=60):
    """
    Hàm tìm kiếm đối tượng bằng cách quay servo xung quanh từ góc khởi đầu đến góc tối đa.

    Args:
        servo_1: Servo điều khiển góc quay theo chiều ngang.
        servo_2: Servo điều khiển góc quay theo chiều dọc.
        frame_queue: Hàng đợi chứa khung hình để kiểm tra phát hiện đối tượng.
        num_turns: Số vòng quay tối đa để tìm kiếm.
        step_angle: Góc thay đổi mỗi lần quay servo (có thể điều chỉnh).
        start_angle_1: Góc khởi đầu cho servo 1.
        start_angle_2: Góc khởi đầu cho servo 2.

    Returns:
        (target_angle_1, target_angle_2) nếu đối tượng được phát hiện, (None, None) nếu không.
    """
    # Khởi tạo góc quay
    target_angle_1 = start_angle_1
    target_angle_2 = start_angle_2

    MAX_ANGLE = 120  # Giới hạn góc tối đa
    MIN_ANGLE = 0    # Giới hạn góc tối thiểu

    for i in range(2):  # Lặp lại quá trình tìm kiếm 2 lần
        for turn in range(num_turns):
            print(f"Vòng tìm kiếm {turn + 1}/{num_turns} ở góc {target_angle_1} độ.")

            # Quay servo đến góc hiện tại
            servo_1.move_to_angle(target_angle_1)
            servo_2.move_to_angle(target_angle_2)
            sleep(1)  # Chờ một chút để servo ổn định

            # Kiểm tra có phát hiện đối tượng không từ frame_queue
            if not frame_queue.empty():
                status, _, _, _ = frame_queue.get()  # Lấy thông tin từ hàng đợi
                if status:
                    print("Đối tượng đã được phát hiện.")
                    return target_angle_1, target_angle_2  # Trả về góc của servo

            # Cập nhật góc quay
            target_angle_1 += step_angle

            # Giới hạn góc quay
            if target_angle_1 > MAX_ANGLE:
                target_angle_1 = MIN_ANGLE  # Reset về góc tối thiểu nếu vượt quá tối đa

        # Đưa servo 2 về vị trí cố định sau khi hoàn thành vòng quét
        servo_2.move_to_angle(80)
        servo_1.move_to_angle(MIN_ANGLE)  # Đưa servo 1 về góc khởi đầu

    print("Không phát hiện được đối tượng trong vòng tìm kiếm.")
    return None, None  # Không tìm thấy đối tượng

def rotate_robot(target_angle, vx, vy, DEFAULT_ANGLE, angle):
    if target_angle < DEFAULT_ANGLE - angle:
        set_motors_direction('rotate_right', vx, vy, 0)
        sleep(0.1)
        set_motors_direction('stop', vx, vy, 0)
    elif target_angle > DEFAULT_ANGLE + angle:
        set_motors_direction('rotate_left', vx, vy, 0)
        sleep(0.1)
        set_motors_direction('stop', vx, vy, 0)
def go_right_or_left(target_angle, vx, vy, DEFAULT_ANGLE, angle):
    if target_angle < DEFAULT_ANGLE - angle:
        set_motors_direction('go_right', vx, vy, 0)
        sleep(0.1)
        set_motors_direction('stop', vx, vy, 0)
    elif target_angle > DEFAULT_ANGLE + angle:
        set_motors_direction('go_left', vx, vy, 0)
        sleep(0.1)
        set_motors_direction('stop', vx, vy, 0)
