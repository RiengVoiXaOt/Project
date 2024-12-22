from src.vision.object_detection import object_detection_loop
from src.vision.color_detection import color_detection_loop
from src.hardware.battery import BatteryMonitor
from src.vision.video_stream import VideoStream
from threading import Thread, Event, Lock
from queue import Queue
from time import sleep
import cv2
from datetime import datetime
from src.utils.control_utils import set_motors_direction, go_right_or_left, rotate_robot
from src.robot.modes import Modes
from src.hardware.servos import ServoControl, DEFAULT_ANGLE
from src.hardware.relay import RelayControl
from src.hardware.ultrasonic import UltrasonicSensors

OPERATION_START_TIME = datetime.now().replace(hour=12, minute=40,second=0)
now = datetime.now()
relay_control = RelayControl(5)
battery = BatteryMonitor()
_, _, _, battery_percentage, _ = battery.read_battery_status()
a = Modes()
ultrasonic_sensors=UltrasonicSensors()
servo_1 = ServoControl(channel=1)
servo_2 = ServoControl(channel=0)
vx =0.1, vy =0.1
def move_to_target(
    servo_1, servo_2, 
    deviation_x, deviation_y, 
    target_angle_1, target_angle_2, 
    vx, vy, relay_control,
    front_distance, right_distance, left_distance,
    DEFAULT_ANGLE
):
    """
    Điều khiển robot di chuyển đến vị trí mục tiêu.
    """
    # Điều chỉnh servo để theo dõi mục tiêu
    servo_1.tracking_servo_bottom(deviation_x, target_angle_1)
    servo_2.tracking_servo_bottom(deviation_y, target_angle_2)

    # Kiểm tra khoảng cách phía trước để xác định hành động
    if front_distance > 15:
        # Di chuyển thẳng nếu servo ổn định
        if 51 < target_angle_1 < 69 and abs(deviation_x) < 10:
            print("Di chuyển thẳng về phía mục tiêu.")
            set_motors_direction('go_forward', vx, vy, 0)
        else:
            print("Servo chưa ổn định, chờ thêm.")
    else:
        # Dừng robot nếu khoảng cách phía trước quá gần
        print("Khoảng cách phía trước quá gần, dừng lại.")
        set_motors_direction('stop', 0, 0, 0)
        relay_control.run_relay_for_duration()

    # Kiểm tra và quay robot nếu cần
    if abs(deviation_x) > 40:
        if right_distance > 15 and left_distance > 15:
            print("Quay robot để điều chỉnh góc nhìn.")
            go_right_or_left(target_angle_1, vx, vy, DEFAULT_ANGLE, 4)
        elif 20 <= abs(deviation_x) < 40:
            print("Hiệu chỉnh góc robot.")
            rotate_robot(target_angle_1, vx, vy, DEFAULT_ANGLE, 4)

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

def left_charger():
    set_motors_direction('go_backward', 0.1, 0.1, 0)
    sleep(1)
    set_motors_direction('rotate_left', 0.1, 0.1, 0)
    sleep(1)
def rest_in_charger(servo_1, servo_2, DEFAULT_ANGLE):
    servo_1.move_to_angle = DEFAULT_ANGLE
    servo_2.move_to_angle = DEFAULT_ANGLE
    set_motors_direction('stop', 0, 0, 0)
def handle_black_line(status_black, deviation_x_black, deviation_y_black):
    if status_black:
        if abs(deviation_x_black) < 180 or abs(deviation_y_black) < 180:
            set_motors_direction('rotate_left', 0.1, 0.1, 0)
        if deviation_y_black > 30 and deviation_x_black == 0:
            set_motors_direction('rotate_right', 0.1, 0.1, 0)
# Định dạng thời gian theo ý muốn
def Auto():
    stop_event = Event()
    frame_queue = Queue(maxsize=1)
    mission = False
    try:
        # Khởi tạo VideoStream
        videostream = VideoStream(resolution=(640, 480), framerate=30).start()
        sleep(1)

        # Bắt đầu nhận diện màu sắc trong một thread riêng với giới hạn FPS
        color_thread = Thread(target=color_detection_loop, args=(videostream, 320, 240, 1000, stop_event, frame_queue, 10), daemon=True)
        color_thread.start()

        # Bắt đầu nhận diện đối tượng trong một thread riêng
        object_thread = Thread(target=object_detection_loop, args=(videostream, stop_event, frame_queue), daemon=True)
        object_thread.start()
        while not stop_event.is_set():
            if not frame_queue.empty():
                # Lấy dữ liệu từ hàng đợi
                (frame_color, mask_red, mask_black, status_red, deviation_x_red, deviation_y_red,
                status_black, deviation_x_black, deviation_y_black, contours_black, contours_red,
                status_water, status_charger, deviation_x_water, deviation_y_water,
                deviation_x_charger, deviation_y_charger, frame_object) = frame_queue.get()

                front_distance = ultrasonic_sensors.get_distance("front")
                left_distance = ultrasonic_sensors.get_distance("left")
                right_distance = ultrasonic_sensors.get_distance("right")

                # Kiểm tra xem đã đến thời điểm hoạt động chưa
                if now >= OPERATION_START_TIME:
                    print(f"Robot bắt đầu hoạt động vào {now.strftime('%Y-%m-%d %H:%M:%S')}")
                    if battery_percentage > 25:
                        if not mission:
                            rest_in_charger()  
                        else:
                            left_charger()
                            if status_black:
                                if abs(deviation_x_black) < 180 or abs(deviation_y_black) < 180:
                                    set_motors_direction('rotate_left', 0.1, 0.1, 0)
                                if deviation_y_black > 30 and deviation_x_black == 0:
                                    set_motors_direction('rotate_right', 0.1, 0.1, 0)
                            else:
                                a.avoid_obstacles()
                                if status_water:
                                    move_to_target(servo_1, servo_2, deviation_x_water, deviation_y_water, target_angle_1, target_angle_2, vx, vy, relay_control, front_distance, right_distance, left_distance, DEFAULT_ANGLE)
                                else:
                                    time += 1
                                    if time >= 60:
                                        new_angle_1, new_angle_2 = a.search_for_object(servo_1, servo_2, frame_queue)
                                        if new_angle_1 is not None and new_angle_2 is not None:
                                            target_angle_1, target_angle_2 = new_angle_1, new_angle_2
                    else:
                        if front_distance < 10 and status_charger:
                            rest_in_charger()
                        else:
                            handle_black_line(status_black, deviation_x_black, deviation_y_black)
                            if not status_black:
                                a.avoid_obstacles()
                                red_line_following(contours_red)
                                if status_water:
                                    move_to_target(servo_1, servo_2, deviation_x_charger, deviation_y_charger, target_angle_1, target_angle_2, vx, vy, relay_control, front_distance, right_distance, left_distance, DEFAULT_ANGLE)
                                else:
                                    time += 1
                                    if time >= 60:
                                        new_angle_1, new_angle_2 = a.search_for_object(servo_1, servo_2, frame_queue)
                                        if new_angle_1 is not None and new_angle_2 is not None:
                                            target_angle_1, target_angle_2 = new_angle_1, new_angle_2
                else:
                    if front_distance < 10 and status_charger:
                        rest_in_charger()
                    else:
                        handle_black_line(status_black, deviation_x_black, deviation_y_black)
                        if not status_black:
                            a.avoid_obstacles()
                            red_line_following(contours_red)
                            if status_water:
                                move_to_target(servo_1, servo_2, deviation_x_charger, deviation_y_charger, target_angle_1, target_angle_2, vx, vy, relay_control, front_distance, right_distance, left_distance, DEFAULT_ANGLE)
                            else:
                                time += 1
                                if time >= 60:
                                    new_angle_1, new_angle_2 = a.search_for_object(servo_1, servo_2, frame_queue)
                                    if new_angle_1 is not None and new_angle_2 is not None:
                                        target_angle_1, target_angle_2 = new_angle_1, new_angle_2

                    # Dừng chương trình nếu nhấn phím 'q'
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        stop_event.set()
                        break

        color_thread.join()
        object_thread.join()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        stop_event.set()
        videostream.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    Auto()  # Chạy chương trình khi thực thi script