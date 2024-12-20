from time import sleep, time
from src.hardware.relay import RelayControl
from src.hardware.servos import ServoControl
from src.hardware.motors import Motors
from src.hardware.ultrasonic import UltrasonicSensors
from src.utils.control_utils import getch, direction, set_motors_direction

# Khởi tạo các thành phần phần cứng của robot
relay_control = RelayControl(5)  # Điều khiển relay cho tưới cây
top_servo = ServoControl(channel=0)  # Servo trên
bottom_servo = ServoControl(channel=1)  # Servo dưới
motors = Motors()  # Điều khiển động cơ

# Các biến trạng thái
speed_factor = 5
speed_step = 0.04309596457  # Tốc độ mỗi bước (10% tốc độ tối đa)
vx = speed_factor * speed_step
vy = speed_factor * speed_step
theta = 0
state = "stopped"
is_watering = False

# Các hàm điều khiển
def update_state(new_state):
    global state
    state = new_state
    print(f"Trạng thái hiện tại: {state}")

def increase_speed(command):
    global speed_factor, vx, vy
    if command in ['+', '=']:
        speed_factor = min(speed_factor + 1, 10)
        vx = speed_factor * speed_step
        vy = speed_factor * speed_step
        update_state(f"speed increased to {speed_factor * 10}%")
        print(f"Tốc độ tăng lên {speed_factor * 10}%")

def decrease_speed(command):
    global speed_factor, vx, vy
    #if command in ['-', '_']:
    speed_factor = max(speed_factor - 1, 0)
    vx = speed_factor * speed_step
    vy = speed_factor * speed_step
    update_state(f"speed decreased to {speed_factor * 10}%")
    print(f"Tốc độ giảm xuống {speed_factor * 10}%")

def move_robot(command):
    if command in direction:
        current_direction = direction[command]
        set_motors_direction(current_direction, vx, vy, theta)
        update_state(f"moving {current_direction}")
    else:
        update_state("invalid command")
        print("Lệnh không hợp lệ. Vui lòng thử lại.")

def toggle_relay():
    relay_control.run_relay_for_duration()
    update_state("watering activated")

def control_servo(command):
    if command == '7':
        bottom_servo.move_up()
        update_state("bottom servo moved up")
    elif command == '8':
        bottom_servo.move_down()
        update_state("bottom servo moved down")
    elif command == '9':
        top_servo.move_up()
        update_state("top servo moved up")
    elif command == '0':
        top_servo.move_down()
        update_state("top servo moved down")

def stop_robot():
    top_servo.reset()
    bottom_servo.reset()
    set_motors_direction('stop', 0, 0, 0)
    update_state("manual control exited")
    print("Exiting manual control.")

def manual_control(command):
    """Chế độ điều khiển thủ công bằng bàn phím."""
    while True:
        print("Enter command (w/a/s/d/q/e/z/x/1/2/3 to move, r to toggle relay, p to quit): ")
        command = getch()  # Nhận lệnh từ bàn phím
        if command == 'p':
            stop_robot()
            break
        elif command == '+':
            increase_speed(command)
        elif command  == '-':
            decrease_speed(command)
        elif command in direction:
            move_robot(command)
        elif command == 'r':
            toggle_relay()
        elif command in ['7', '8', '9', '0']:
            control_servo(command)
        else:
            update_state("invalid command")
            print("Lệnh không hợp lệ. Vui lòng thử lại.")
