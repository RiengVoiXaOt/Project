import os
import cv2
from tensorflow.lite.python.interpreter import Interpreter # type: ignore
from queue import Queue
from src.utils.image_utils import (
    load_labels,
    load_model,
    get_model_details,
    detect_objects,
    draw_detections,
    preprocess_frame,
    calculate_fps
)
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
