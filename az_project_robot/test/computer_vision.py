import os
import cv2
import numpy as np
import sys
import time
from threading import Thread, Event, Lock
from tensorflow.lite.python.interpreter import Interpreter
from queue import Queue
from utils import (
    process_frame,
    analyze_contours,
    display_info,
    load_labels,
    load_model,
    get_model_details,
    detect_objects,
    draw_detections,
    preprocess_frame,
    calculate_fps
)

# Lớp VideoStream giúp tạo và quản lý luồng video từ webcam
class VideoStream:
    """
    Mục đích: Quản lý luồng video từ camera, bao gồm việc khởi tạo, đọc và dừng luồng video.
    
    Chi tiết:
    - Khởi tạo kết nối với camera thông qua OpenCV.
    - Lấy và đọc khung hình từ camera.
    - Cung cấp phương thức để bắt đầu và dừng luồng video.
    """
    def __init__(self, resolution=(640, 480), framerate=30):
        self.stream = cv2.VideoCapture(0)
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.stream.set(3, resolution[0])
        self.stream.set(4, resolution[1])
        if not self.stream.isOpened():
            raise RuntimeError("Cannot open camera.")
        self.grabbed, self.frame = self.stream.read()
        self.stopped = False
        self.lock = Lock()

    def start(self):
        """
        Mục đích: Bắt đầu quá trình đọc video trong một thread riêng biệt.
        """
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        """
        Mục đích: Cập nhật khung hình video liên tục trong vòng lặp.
        """
        while not self.stopped:
            grabbed, frame = self.stream.read()
            with self.lock:
                self.grabbed, self.frame = grabbed, frame

    def read(self):
        """
        Mục đích: Đọc khung hình hiện tại từ luồng video.
        """
        with self.lock:
            return self.frame.copy()

    def stop(self):
        """
        Mục đích: Dừng luồng video và giải phóng tài nguyên.
        """
        self.stopped = True
        self.stream.release()

# Hàm chính để thực hiện nhận diện màu sắc trong video (đỏ và đen)
def color_detection_loop(videostream, center_x, center_y, min_box_area, stop_event, frame_queue, max_fps=15):
    """
    Mục đích: Phát hiện và theo dõi các đối tượng có màu đỏ và đen trong video.

    Chi tiết:
    - Đọc từng khung hình từ luồng video.
    - Tạo các mặt nạ (mask) cho màu đỏ và màu đen trong không gian màu HSV.
    - Phân tích các vùng màu sắc và vẽ các hình chữ nhật bao quanh đối tượng.
    - Giới hạn FPS để giảm tải tài nguyên xử lý.
    """
    frame_rate_calc = 1  # Khởi tạo FPS
    freq = cv2.getTickFrequency()  # Lấy tần số đồng hồ OpenCV
    frame_interval = 1.0 / max_fps  # Tính toán thời gian giữa các frame để giới hạn FPS
    last_time = time.time()

    while not stop_event.is_set():
        current_time = time.time()
        if current_time - last_time < frame_interval:
            continue  # Bỏ qua nếu chưa đủ thời gian cho frame tiếp theo

        frame = videostream.read()
        if frame is None:
            print("Cannot read from camera.")
            break

        t1 = cv2.getTickCount()  # Bắt đầu theo dõi thời gian

        # Phân tích khung hình để tạo mask cho màu đỏ và đen
        mask_red, mask_black = process_frame(frame)

        # Phân tích các contour để tìm các đối tượng trong vùng mask
        status_red, x_r, y_r, contours_red, x_red, y_red, w_red, h_red = analyze_contours(mask_red, center_x, center_y, min_box_area, "red")
        status_black, x_b, y_b, contours_black, x_black, y_black, w_black, h_black = analyze_contours(mask_black, center_x, center_y, min_box_area, "black")

        # Chuyển status thành chuỗi để hiển thị trên video
        status_red = str(status_red)
        status_black = str(status_black)
        
        # Vẽ kết quả (cập nhật vào frame)
        if contours_red:
            cv2.rectangle(frame, (x_red, y_red), (x_red + w_red, y_red + h_red), (0, 0, 255), 2)
            cv2.circle(frame, (x_red + w_red // 2, y_red + h_red // 2), 5, (0, 0, 255), -1)
        if contours_black:
            cv2.rectangle(frame, (x_black, y_black), (x_black + w_black, y_black + h_black), (255, 255, 255), 2)
            cv2.circle(frame, (x_black + w_black // 2, y_black + h_black // 2), 5, (255, 255, 255), -1)

        # Gửi kết quả qua hàng đợi để truyền cho phần hiển thị
        if not frame_queue.full():
            frame_queue.put((frame, mask_red, mask_black, None))

        # Hiển thị thông tin FPS và các kết quả trên video
        display_info(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), [status_red, status_black], [(x_r, y_r), (x_b, y_b)], center_x, center_y)
        
        t2 = cv2.getTickCount()  # Lấy thời gian kết thúc
        frame_rate_calc = calculate_fps(t1, t2, freq)  # Tính FPS
        last_time = current_time  # Cập nhật thời gian cuối cùng

# Hàm nhận diện đối tượng trong video sử dụng TensorFlow Lite
def object_detection_loop(videostream, stop_event, frame_queue):
    """
    Mục đích: Phát hiện đối tượng trong video sử dụng mô hình học sâu (TensorFlow Lite).
    
    Chi tiết:
    - Đọc khung hình từ luồng video.
    - Tiền xử lý khung hình và sử dụng mô hình TensorFlow Lite để nhận diện các đối tượng.
    - Vẽ các bounding box cho các đối tượng phát hiện được.
    """
    # Định nghĩa các tham số và đường dẫn đến mô hình
    MODEL_NAME = '/home/az/Desktop/Project/az_project_robot/models'
    GRAPH_NAME = 'detect.tflite'
    LABELMAP_NAME = 'labelmap.txt'
    min_conf_threshold = 0.7
    imW, imH = 640, 480
    CWD_PATH = os.getcwd()
    PATH_TO_CKPT = os.path.join(CWD_PATH, MODEL_NAME, GRAPH_NAME)
    PATH_TO_LABELS = os.path.join(CWD_PATH, MODEL_NAME, LABELMAP_NAME)

    # Tải nhãn và mô hình
    labels = load_labels(PATH_TO_LABELS)
    interpreter = load_model(PATH_TO_CKPT)
    input_details, output_details, height, width, floating_model = get_model_details(interpreter)
    
    frame_rate_calc = 1  # Khởi tạo FPS
    freq = cv2.getTickFrequency()  # Lấy tần số đồng hồ OpenCV

    while not stop_event.is_set():
        t1 = cv2.getTickCount()  # Bắt đầu theo dõi thời gian
        frame = videostream.read()  # Đọc khung hình từ luồng video

        if frame is None:
            print("Cannot read frame from videostream.")
            break

        # Tiền xử lý và nhận diện đối tượng
        input_data = preprocess_frame(frame, width, height, floating_model)
        detections = detect_objects(interpreter, input_data, input_details, output_details, min_conf_threshold, imW, imH)
        
        # Vẽ các đối tượng phát hiện được lên khung hình
        frame = draw_detections(frame, detections, labels, min_conf_threshold, imW, imH)
        
        # Hiển thị FPS trên video
        cv2.putText(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (255, 255, 0), 2, cv2.LINE_AA)
        
        t2 = cv2.getTickCount()  # Lấy thời gian kết thúc
        frame_rate_calc = calculate_fps(t1, t2, freq)  # Tính FPS

        # Gửi kết quả qua hàng đợi
        if not frame_queue.full():
            frame_queue.put((None, None, None, frame))  # Đẩy frame vào hàng đợi

# Hàm chính để khởi tạo và chạy các luồng
def main():
    stop_event = Event()
    frame_queue = Queue(maxsize=1)

    try:
        # Khởi tạo VideoStream
        videostream = VideoStream(resolution=(640, 480), framerate=30)
        videostream.start()
        time.sleep(1)

        # Bắt đầu nhận diện màu sắc trong một thread riêng với giới hạn FPS
        color_thread = Thread(target=color_detection_loop, args=(videostream, 320, 240, 1000, stop_event, frame_queue, 10), daemon=True)
        color_thread.start()

        # Bắt đầu nhận diện đối tượng trong một thread riêng
        object_thread = Thread(target=object_detection_loop, args=(videostream, stop_event, frame_queue), daemon=True)
        object_thread.start()

        # Hiển thị kết quả trong thread chính
        while not stop_event.is_set():
            if not frame_queue.empty():
                # Lấy dữ liệu từ hàng đợi
                frame, mask_red, mask_black, frame2 = frame_queue.get()

                # Kiểm tra nếu khung hình không phải là None
                if frame is not None:
                    cv2.imshow("Color Tracking Frame", frame)
                if mask_red is not None:
                    cv2.imshow("Red Mask", mask_red)
                if mask_black is not None:
                    cv2.imshow("Black Mask", mask_black)
                if frame2 is not None:
                    cv2.imshow("Object Detection Frame", frame2)  # Hiển thị frame từ nhận diện đối tượng

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
    main()  # Chạy chương trình khi thực thi script
