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
import warnings

OPERATION_START_TIME = datetime.now().replace(hour=12, minute=40,second=0)
now = datetime.now()
relay_control = RelayControl(5)
battery = BatteryMonitor()
_, _, _, battery_percentage, _ = battery.read_battery_status()
a = Modes()
ultrasonic_sensors=UltrasonicSensors()
servo_1 = ServoControl(channel=1)
servo_2 = ServoControl(channel=0)

servo_1.move_to_angle(60)
servo_2.move_to_angle(90)
vx = 0.15
vy = 0.15
def search_for_object(bottom_servo, top_servo, frame_queue, num_turns = 4):
        """
        Tìm kiếm đối tượng bằng cách quay servo xung quanh từ góc khởi đầu đến góc tối đa.
        """
        print("Bắt đầu tìm kiếm vật thể...")
        
        target_angle_1 = 0
        target_angle_2 = 70

        MAX_ANGLE = 120  # Giới hạn góc tối đa
        MIN_ANGLE = 0    # Giới hạn góc tối thiểu
        for turn in range(num_turns):
            print(f"Vòng tìm kiếm {turn + 1}/{num_turns} ở góc {target_angle_1} độ.")

            # Quay servo đến góc hiện tại
            bottom_servo.move_to_angle(target_angle_1)
            top_servo.move_to_angle(target_angle_2)
            sleep(1)  # Chờ một chút để servo ổn định

            # Kiểm tra có phát hiện đối tượng không từ frame_queue
            if not frame_queue.empty():
                status, _, _, _ = frame_queue.get()  # Lấy thông tin từ hàng đợi
                if status:
                    print("Đối tượng đã được phát hiện.")
                    return target_angle_1, target_angle_2  # Trả về góc của servo

            # Cập nhật góc quay
            target_angle_1 += 30

            # Giới hạn góc quay
            if target_angle_1 > MAX_ANGLE:
                target_angle_1 = MIN_ANGLE  # Reset về góc tối thiểu nếu vượt quá tối đa

        print("Không phát hiện được đối tượng trong vòng tìm kiếm.")
        return None, None  # Không tìm thấy đối tượng
def move_to_target(
    servo_1, servo_2, 
    deviation_x, deviation_y, 
    target_angle_1, target_angle_2, 
    vx, vy, relay_control,
    front_distance, right_distance, left_distance,
    DEFAULT_ANGLE
):
    print(f"Điều khiển robot đến mục tiêu: deviation_x={deviation_x}, deviation_y={deviation_y}")
    # Điều chỉnh servo để theo dõi mục tiêu
    servo_1.tracking_servo_bottom(deviation_x, target_angle_1)
    servo_2.tracking_servo_bottom(deviation_y, target_angle_2)

    # Kiểm tra khoảng cách phía trước để xác định hành động
    if front_distance > 15:
        print(f"Khoảng cách phía trước: {front_distance}. Đủ xa để tiếp tục di chuyển.")
        if 51 < target_angle_1 < 69 and abs(deviation_x) < 10:
            print("Servo ổn định. Robot di chuyển thẳng về phía mục tiêu.")
            set_motors_direction('go_forward', vx, vy, 0)
        else:
            print("Servo chưa ổn định. Chờ thêm để điều chỉnh.")
    else:
        print(f"Khoảng cách phía trước quá gần ({front_distance}). Dừng robot.")
        set_motors_direction('stop', 0, 0, 0)
        relay_control.run_relay_for_duration()

    if abs(deviation_x) > 40:
        print(f"Deviation_x vượt quá giới hạn: {deviation_x}. Robot cần quay.")
        if right_distance > 15 and left_distance > 15:
            print("Khoảng cách hai bên đủ rộng. Robot quay để điều chỉnh góc nhìn.")
            go_right_or_left(target_angle_1, vx, vy, DEFAULT_ANGLE, 4)
        elif 20 <= abs(deviation_x) < 40:
            print("Hiệu chỉnh góc robot.")
            rotate_robot(target_angle_1, vx, vy, DEFAULT_ANGLE, 4)

def red_line_following(contours):
    print("red line ")
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
    print("Đang nghỉ ngơi tại trạm sạc")
    servo_1.move_to_angle(DEFAULT_ANGLE)
    servo_2.move_to_angle(DEFAULT_ANGLE)
    set_motors_direction('stop', 0, 0, 0)
def handle_black_line(status_black, deviation_x_black, deviation_y_black):
    if status_black:
        if abs(deviation_x_black) < 200 or abs(deviation_y_black) < 200:
            set_motors_direction('rotate_left', 0.1, 0.1, 0)
        if deviation_y_black > 30 and deviation_x_black == 0:
            set_motors_direction('rotate_right', 0.1, 0.1, 0)

def avoid_obstacles(front_distance, left_distance, right_distance, vx, vy, safe_distance=15, critical_distance=10):
    """
    Hàm tránh chướng ngại vật sử dụng cảm biến siêu âm.
    
    Parameters:
        front_distance (float): Khoảng cách phía trước.
        left_distance (float): Khoảng cách bên trái.
        right_distance (float): Khoảng cách bên phải.
        vx (float): Tốc độ theo trục x.
        vy (float): Tốc độ theo trục y.
        safe_distance (float): Khoảng cách an toàn để tránh chướng ngại vật.
        critical_distance (float): Khoảng cách nguy hiểm cần dừng ngay.
    """
    if None in (front_distance, left_distance, right_distance):
        print("Lỗi cảm biến siêu âm: Không nhận được dữ liệu.")
        set_motors_direction("stop", 0, 0, 0)
        return

    if front_distance < critical_distance:
        print(f"Khoảng cách phía trước quá nguy hiểm ({front_distance} cm). Dừng robot và lùi lại.")
        set_motors_direction("go_backward", vx, vy, 0)
        sleep(1)  # Lùi trong một giây
        set_motors_direction("stop", 0, 0, 0)
        return

    if front_distance < safe_distance:
        print(f"Khoảng cách phía trước gần ({front_distance} cm). Robot sẽ tránh sang một bên.")
        if left_distance > right_distance:
            print("Tránh sang trái.")
            set_motors_direction("go_left", vx, vy, 0)
        else:
            print("Tránh sang phải.")
            set_motors_direction("go_right", vx, vy, 0)
        sleep(1)  # Di chuyển tránh trong một giây
        set_motors_direction("stop", 0, 0, 0)
    else:
        print("Không có chướng ngại vật phía trước. Tiếp tục di chuyển.")
        set_motors_direction("go_forward", vx, vy, 0)

def handle_side_obstacles(left_state, right_state, left_distance, right_distance, vx, vy, safe_distance=15):
    """
    Xử lý chướng ngại vật bên trái và phải.

    Parameters:
        left_state (bool): Trạng thái có vật cản bên trái.
        right_state (bool): Trạng thái có vật cản bên phải.
        left_distance (float): Khoảng cách bên trái.
        right_distance (float): Khoảng cách bên phải.
        vx (float): Tốc độ theo trục x.
        vy (float): Tốc độ theo trục y.
        safe_distance (float): Khoảng cách an toàn để tránh chướng ngại vật.
    """
    if left_state and left_distance < safe_distance:
        print(f"Tránh chướng ngại vật bên trái ({left_distance} cm).")
        set_motors_direction("go_right", vx, vy, 0)
        sleep(1)
        set_motors_direction("stop", 0, 0, 0)
    elif right_state and right_distance < safe_distance:
        print(f"Tránh chướng ngại vật bên phải ({right_distance} cm).")
        set_motors_direction("go_left", vx, vy, 0)
        sleep(1)
        set_motors_direction("stop", 0, 0, 0)
    else:
        print("Không có vật cản hai bên. Tiếp tục tiến.")
        set_motors_direction("go_forward", vx, vy, 0)

def avoid_and_navigate(front_distance, left_distance, right_distance, vx, vy):
    """
    Kết hợp tránh vật cản và điều hướng robot.

    Parameters:
        front_distance (float): Khoảng cách phía trước.
        left_distance (float): Khoảng cách bên trái.
        right_distance (float): Khoảng cách bên phải.
        vx (float): Tốc độ theo trục x.
        vy (float): Tốc độ theo trục y.
    """
    print("Bắt đầu tránh vật cản và điều hướng.")
    avoid_obstacles(front_distance, left_distance, right_distance, vx, vy)

    if front_distance >= 15:
        print("Không có vật cản phía trước. Kiểm tra hai bên để điều hướng thêm.")
        left_state = left_distance < 15
        right_state = right_distance < 15
        handle_side_obstacles(left_state, right_state, left_distance, right_distance, vx, vy)
# Định dạng thời gian theo ý muốn
def Auto():
    stop_event = Event()
    frame_queue = Queue(maxsize=1)
    mission = True
    time = 0  # Khởi tạo biến time
    target_angle_1, target_angle_2 = DEFAULT_ANGLE, DEFAULT_ANGLE  # Khởi tạo góc servo

    try:
        # Khởi tạo VideoStream
        videostream = VideoStream(resolution=(640, 480), framerate=30).start()  # Gọi start() để khởi tạo đối tượng
        sleep(1)

        # Bắt đầu nhận diện màu sắc trong một thread riêng
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
                
                if frame_object is not None:
                    cv2.imshow("Color Tracking Frame", frame_object)

                front_distance = ultrasonic_sensors.get_distance("front")
                left_distance = ultrasonic_sensors.get_distance("left")
                right_distance = ultrasonic_sensors.get_distance("right")
                warnings.filterwarnings("ignore", category=UserWarning, module='cv2')
                print(f"Trạng thái pin: {battery_percentage}%.")
                print(f"Khoảng cách: front={front_distance}, left={left_distance}, right={right_distance}.")
                # Kiểm tra xem đã đến thời điểm hoạt động chưa
                if now >= OPERATION_START_TIME:
                    print(f"Đã đến giờ hoạt động. Bắt đầu vào lúc {OPERATION_START_TIME}.")
                    if battery_percentage > 25:
                        print("Pin đủ để hoạt động. Tiếp tục nhiệm vụ.")
                        if not mission:
                            if front_distance < 10 and status_charger:
                                rest_in_charger(servo_1, servo_2, DEFAULT_ANGLE)  
                            else:
                                print("Ko ở tại trạm sạc, đi tìm")
                                handle_black_line(status_black, deviation_x_black, deviation_y_black)
                                if not status_black:
                                    avoid_and_navigate(front_distance, left_distance, right_distance, vx, vy)
                                    red_line_following(contours_red)
                                    if status_water:
                                        move_to_target(servo_1, servo_2, deviation_x_charger, deviation_y_charger, target_angle_1, target_angle_2, vx, vy, relay_control, front_distance, right_distance, left_distance, DEFAULT_ANGLE)
                                    else:
                                        time += 1
                                        if time >= 6000:
                                            new_angle_1, new_angle_2 = search_for_object(servo_1, servo_2, frame_queue)
                                            if new_angle_1 is not None and new_angle_2 is not None:
                                                target_angle_1, target_angle_2 = new_angle_1, new_angle_2

                        else:
                            left_charger()
                            if status_black:
                                handle_black_line(status_black, deviation_x_black, deviation_y_black)
                            else:
                                avoid_and_navigate(front_distance, left_distance, right_distance, vx, vy)
                                if status_water:
                                    move_to_target(servo_1, servo_2, deviation_x_water, deviation_y_water, target_angle_1, target_angle_2, vx, vy, relay_control, front_distance, right_distance, left_distance, DEFAULT_ANGLE)
                                else:
                                    time += 1
                                    if time >= 6000:
                                        new_angle_1, new_angle_2 = search_for_object(servo_1, servo_2, frame_queue)
                                        if new_angle_1 is not None and new_angle_2 is not None:
                                            target_angle_1, target_angle_2 = new_angle_1, new_angle_2
                    else:
                        print("Pin yếu. Chuyển sang chế độ nghỉ hoặc quay về sạc.")
                        if front_distance < 10 and status_charger:
                            rest_in_charger(servo_1, servo_2, DEFAULT_ANGLE)
                        else:
                            handle_black_line(status_black, deviation_x_black, deviation_y_black)
                            if not status_black:
                                avoid_and_navigate(front_distance, left_distance, right_distance, vx, vy)
                                red_line_following(contours_red)
                                if status_water:
                                    move_to_target(servo_1, servo_2, deviation_x_charger, deviation_y_charger, target_angle_1, target_angle_2, vx, vy, relay_control, front_distance, right_distance, left_distance, DEFAULT_ANGLE)
                                else:
                                    time += 1
                                    if time >= 6000:
                                        new_angle_1, new_angle_2 = search_for_object(servo_1, servo_2, frame_queue)
                                        if new_angle_1 is not None and new_angle_2 is not None:
                                            target_angle_1, target_angle_2 = new_angle_1, new_angle_2

                else:
                    print("Chưa đến thời gian hoạt động. Robot đang chờ.")
                    if front_distance < 10 and status_charger:
                        rest_in_charger(servo_1, servo_2, DEFAULT_ANGLE)
                    else:
                        handle_black_line(status_black, deviation_x_black, deviation_y_black)
                        if not status_black:
                            avoid_and_navigate(front_distance, left_distance, right_distance, vx, vy)
                            red_line_following(contours_red)
                            if status_water:
                                move_to_target(servo_1, servo_2, deviation_x_charger, deviation_y_charger, target_angle_1, target_angle_2, vx, vy, relay_control, front_distance, right_distance, left_distance, DEFAULT_ANGLE)
                            else:
                                time += 1
                                if time >= 6000:
                                    new_angle_1, new_angle_2 = search_for_object(servo_1, servo_2, frame_queue)
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
        if 'videostream' in locals():
            videostream.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    Auto()  # Chạy chương trình khi thực thi script