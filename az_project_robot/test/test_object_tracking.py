import cv2
import time
import threading
from queue import Queue
from src.utils.image_utils import (
    process_frame,
    analyze_contours,
    display_info,
    calculate_fps
)
from src.hardware.servos import ServoControl, MIN_ANGLE, MAX_ANGLE, DEFAULT_ANGLE
from src.utils.control_utils import set_motors_direction
from src.vision.video_stream import VideoStream
from time import sleep

def color_detection_loop(videostream, center_x, center_y, min_box_area, stop_event, frame_queue, max_fps=10):
    frame_rate_calc = 1
    freq = cv2.getTickFrequency()
    frame_interval = 1.0 / max_fps
    last_time = time.time()

    while not stop_event.is_set():
        current_time = time.time()
        if current_time - last_time < frame_interval:
            continue

        frame = videostream.read()
        if frame is None:
            print("Cannot read from camera.")
            break

        t1 = cv2.getTickCount()

        mask_red, mask_black = process_frame(frame)
        status_red, x_r, y_r, contours_red, x_red, y_red, w_red, h_red = analyze_contours(mask_red, center_x, center_y, min_box_area, "red")
        status_black, x_b, y_b, contours_black, x_black, y_black, w_black, h_black = analyze_contours(mask_black, center_x, center_y, min_box_area, "black")

        # Gửi kết quả vào hàng đợi
        deviation_x = (x_red + w_red // 2) - center_x if contours_red else 0
        deviation_y = (y_r + h_red // 2) - center_y if contours_red else 0
        frame_queue.put((frame, mask_red, mask_black, status_red, deviation_x, deviation_y))

        display_info(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), [str(status_red), str(status_black)], [(x_r, y_r), (x_b, y_b)], center_x, center_y)

        t2 = cv2.getTickCount()
        frame_rate_calc = calculate_fps(t1, t2, freq)
        last_time = current_time

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

def check_for_object(mask, min_box_area=500):
    """
    Hàm kiểm tra xem có vật thể nào trong mask không.

    :param mask: Mask nhị phân để kiểm tra.
    :param min_box_area: Diện tích tối thiểu để coi là một vật thể.
    :return: True nếu phát hiện vật thể, False nếu không.
    """
    # Tìm các contour trong mask nhị phân
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Kiểm tra từng contour
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_box_area:
            return True  # Phát hiện vật thể

    return False  # Không phát hiện vật thể

def main():
    # Khởi tạo VideoStream
    videostream = VideoStream(resolution=(640, 480), framerate=30).start()
    time.sleep(1)

    # Cấu hình phân tích contour
    MIN_BOX_AREA = 500
    CENTER_X = 320
    CENTER_Y = 240

    # Tạo hàng đợi và sự kiện dừng
    frame_queue = Queue(maxsize=1)
    stop_event = threading.Event()

    # Tạo và bắt đầu luồng cho nhận diện màu sắc
    color_thread = threading.Thread(target=color_detection_loop, args=(videostream, CENTER_X, CENTER_Y, MIN_BOX_AREA, stop_event, frame_queue), daemon=True)
    color_thread.start()

    # Khởi tạo servo
    servo_1 = ServoControl(channel=1)
    servo_2 = ServoControl(channel=0)

    target_angle_1 = DEFAULT_ANGLE
    target_angle_2 = DEFAULT_ANGLE
    servo_angle_history_1 = []
    servo_angle_history_2 = []
    MAX_HISTORY = 10

    # Biến để theo dõi trạng thái tracking
    is_tracking = False

    # Hiển thị kết quả trong thread chính
    while not stop_event.is_set():
        if not frame_queue.empty():
            # Lấy dữ liệu từ hàng đợi
            frame, mask_red, mask_black, status_red, deviation_x, deviation_y = frame_queue.get()

            # Hiển thị các khung hình
            if mask_red is not None:
                cv2.imshow("Red Mask", mask_red)

            # Nếu phát hiện đối tượng
            if status_red:
                # Bắt đầu tracking nếu chưa tracking
                if not is_tracking:
                    print("Bắt đầu theo dõi vật thể.")
                    is_tracking = True

                # Điều chỉnh góc servo 1 để theo dõi vật thể
                if deviation_x < -20:
                    target_angle_1 += 1
                elif deviation_x > 20:
                    target_angle_1 -= 1

                if deviation_y < -20:
                    target_angle_2 -= 1
                elif deviation_y > 20:
                    target_angle_2 += 1

                # Giới hạn góc servo
                target_angle_1 = max(MIN_ANGLE, min(MAX_ANGLE, target_angle_1))
                target_angle_2 = max(MIN_ANGLE, min(MAX_ANGLE, target_angle_2))

                servo_1.move_to_angle(target_angle_1)
                servo_2.move_to_angle(target_angle_2)

                # Lưu trữ lịch sử góc servo
                servo_angle_history_1.append(target_angle_1)
                servo_angle_history_2.append(target_angle_2)
                if len(servo_angle_history_1) > MAX_HISTORY:
                    servo_angle_history_1.pop(0)
                if len(servo_angle_history_2) > MAX_HISTORY:
                    servo_angle_history_2.pop(0)
                    
                if len(servo_angle_history_1) >= 3:
                    last_three_angles = servo_angle_history_1[-3:]
                    if all(59 <= angle <= 61 for angle in last_three_angles):
                        print("Servo 1 ổn định, robot bắt đầu di chuyển!")
                        set_motors_direction('go_forward', 0.4, 0, 0)

                # Kiểm tra và quay robot nếu cần
                if abs(deviation_x) < 20:
                    rotate_robot(target_angle_1)

            else:
                # Nếu không phát hiện đối tượng và đang tracking
                if is_tracking:
                    print("Không phát hiện vật thể, bắt đầu tìm kiếm...")
                    is_tracking = False  # Đặt lại trạng thái tracking

                    # Dừng di chuyển
                    if is_moving:
                        print("Dừng di chuyển để kiểm tra...")
                        set_motors_direction('stop', 0, 0, 0)
                        is_moving = False  # Đánh dấu rằng robot không còn di chuyển

                    # Tìm kiếm vật thể bằng cách di chuyển servo 1
                    for angle in range(MIN_ANGLE, MAX_ANGLE + 1, 5):  # Quay servo 1 từ MIN_ANGLE đến MAX_ANGLE
                        servo_1.move_to_angle(angle)
                        sleep(0.1)  # Thời gian để servo di chuyển

                        # Kiểm tra lại vật thể
                        if check_for_object(mask_red, MIN_BOX_AREA):
                            print("Đã tìm thấy vật thể!")
                            is_tracking = True  # Đặt lại trạng thái tracking
                            set_motors_direction('go_forward', 0.4, 0, 0)  # Bắt đầu di chuyển lại
                            is_moving = True  # Đánh dấu rằng robot đang di chuyển
                            break  # Thoát khỏi vòng tìm kiếm

                    # Nếu không tìm thấy vật thể, quay lại từ MAX_ANGLE về MIN_ANGLE
                    if not is_tracking:
                        for angle in range(MAX_ANGLE, MIN_ANGLE - 1, -5):
                            servo_1.move_to_angle(angle)
                            sleep(0.1)  # Thời gian để servo di chuyển

                            # Kiểm tra lại vật thể
                            if check_for_object(mask_red, MIN_BOX_AREA):
                                print("Đã tìm thấy vật thể!")
                                is_tracking = True  # Đặt lại trạng thái tracking
                                set_motors_direction('go_forward', 0.4, 0, 0)  # Bắt đầu di chuyển lại
                                is_moving = True  # Đánh dấu rằng robot đang di chuyển
                                break  # Thoát khỏi vòng tìm kiếm
                            
        # Dừng chương trình nếu nhấn phím 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            servo_1.move_to_angle(60)
            servo_2.move_to_angle(60)
            set_motors_direction('stop', 0, 0, 0)
            stop_event.set()
            break

    # Đợi cho các luồng hoàn thành
    color_thread.join()
    videostream.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()