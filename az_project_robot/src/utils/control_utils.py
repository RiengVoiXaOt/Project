import sys, tty, termios
from src.hardware.motors  import Motors

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

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        command = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return command

def set_motors_direction(robot, command, vx, vy, theta):
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
    
    control(robot)

def control(robot):
    motor_controller = Motors()  # Khởi tạo đối tượng Motors

    robot.backward_kinematics()  # Cập nhật giá trị động cơ dựa trên kinematics

    # Cập nhật tốc độ cho từng động cơ thông qua đối tượng motor_controller
    motor_controller.set_speed("motor_1", max(0, min(abs(robot.v1), 1)))
    motor_controller.set_speed("motor_2", max(0, min(abs(robot.v2), 1)))
    motor_controller.set_speed("motor_3", max(0, min(abs(robot.v3), 1)))
    motor_controller.set_speed("motor_4", max(0, min(abs(robot.v4), 1)))

    # In tốc độ của động cơ để kiểm tra
    print(f"Motor speeds: {robot.v1}, {robot.v2}, {robot.v3}, {robot.v4}")
