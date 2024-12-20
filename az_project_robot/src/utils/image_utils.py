import cv2
import numpy as np
from tensorflow.lite.python.interpreter import Interpreter # type: ignore


def process_frame(frame):
    """
    Mục đích: Xử lý khung hình để phát hiện các đối tượng màu đỏ và đen bằng cách tạo các mask (mặt nạ) dựa trên dải màu trong không gian HSV.

    Chi tiết:
    Chuyển đổi khung hình từ không gian màu BGR sang HSV (cv2.cvtColor).
    Xác định dải màu đỏ (hai phần, vì đỏ nằm ở hai đầu phổ HSV).
    Tạo các mask cho màu đỏ (cv2.inRange) và kết hợp chúng (mask1_red + mask2_red).
    Xác định dải màu đen và tạo mask tương ứng.
    Trả về hai mask: mask_red và mask_black.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Mask màu đỏ
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask1_red = cv2.inRange(hsv, lower_red, upper_red)
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask2_red = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = mask1_red + mask2_red

    # Mask màu đen
    lower_black1 = np.array([0, 0, 0])
    upper_black1 = np.array([180, 255, 50])
    mask_black1 = cv2.inRange(hsv, lower_black1, upper_black1)

    lower_black2 = np.array([0, 0, 0])
    upper_black2 = np.array([180, 255, 80])
    mask_black2 = cv2.inRange(hsv, lower_black2, upper_black2)
    mask_black = mask_black1 + mask_black2

    return mask_red, mask_black


# Phân tích contour của đối tượng
def analyze_contours(mask, center_x, center_y, min_box_area, color):
    """
    Mục đích: Tìm contour lớn nhất trong mask, tính toán các thông tin như kích thước, tọa độ, và độ lệch so với tâm.

    Chi tiết:

    Dùng cv2.findContours để tìm các contour trên mask.
    Gán trạng thái mặc định là không tìm thấy đối tượng (No {color.capitalize()} Object Detected).
    Nếu tìm thấy contour, chọn contour lớn nhất (max(contours, key=cv2.contourArea)).
    Tính toán các thông số như tọa độ, diện tích bounding box và độ lệch so với tâm.
    Chỉ cập nhật thông tin nếu diện tích lớn hơn min_box_area.
    Trả về trạng thái, độ lệch so với tâm, các thông số bounding box, và danh sách contour.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    status = f"No {color.capitalize()} Object Detected"
    deviation_x = deviation_y = 0  # Khởi tạo mặc định
    x = y = w = h = 0  # Khởi tạo mặc định
    status = False

    for contour in contours:
        area = cv2.contourArea(contour)
        if area > min_box_area:  # Nếu diện tích lớn hơn ngưỡng tối thiểu
            x, y, w, h = cv2.boundingRect(contour)
            obj_center_x = x + w // 2
            obj_center_y = y + h // 2
            deviation_x = obj_center_x - center_x
            deviation_y = obj_center_y - center_y
            status = True  # Đối tượng đã được phát hiện
            break  # Chỉ xử lý contour đầu tiên phù hợp (có thể thay đổi tùy yêu cầu)

    return status, deviation_x, deviation_y, contours, x, y, w, h

# Hiển thị thông tin lên khung hình
def display_info(frame, fps, statuses, deviations, center_x, center_y):
    """
    Mục đích: Hiển thị các thông tin như FPS, trạng thái đối tượng, độ lệch, và vẽ các đường chia khung hình.

    Chi tiết:

    Dùng cv2.putText để hiển thị FPS.
    Lặp qua danh sách trạng thái và độ lệch để hiển thị thông tin cho mỗi loại đối tượng (đỏ/đen).
    Vẽ các đường thẳng đứng và ngang qua tâm khung hình để hỗ trợ định vị.
    """
    cv2.putText(frame, f"FPS: {fps}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    for i, (status, deviation) in enumerate(zip(statuses, deviations)):
        color = (0, 0, 255) if i == 0 else (255, 255, 255)
        
        # Đảm bảo status là chuỗi
        status_str = str(status)
        
        cv2.putText(frame, status_str, (10, 40 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(frame, f"x: {deviation[0]}, y: {deviation[1]}", (10, 80 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    cv2.line(frame, (center_x, 0), (center_x, frame.shape[0]), (0, 255, 0), 1)
    cv2.line(frame, (0, center_y), (frame.shape[1], center_y), (0, 255, 0), 1)
    
def load_labels(path_to_labels):
    """
    Mục đích: Tải danh sách nhãn từ tệp tin nhãn.

    Chi tiết:

    Mở tệp tin nhãn và đọc từng dòng, loại bỏ khoảng trắng.
    Loại bỏ dòng đầu tiên nếu nhãn đó là ??? (đại diện cho nhãn không xác định).
    Trả về danh sách nhãn.
    """
    with open(path_to_labels, 'r') as f:
        labels = [line.strip() for line in f.readlines()]  # Đọc mỗi dòng trong file và loại bỏ khoảng trắng
    if labels[0] == '???':
        del(labels[0])  # Loại bỏ nhãn không xác định
    return labels

def load_model(path_to_ckpt):
    """
    Mục đích: Tải mô hình TensorFlow Lite từ đường dẫn.

    Chi tiết:

    Sử dụng Interpreter để tải mô hình từ tệp.
    Cấp phát bộ nhớ cho các tensor bằng allocate_tensors.
    Trả về đối tượng Interpreter.
    """
    interpreter = Interpreter(model_path=path_to_ckpt)  # Tạo đối tượng Interpreter để load mô hình
    interpreter.allocate_tensors()  # Cấp phát bộ nhớ cho các tensor
    return interpreter

def get_model_details(interpreter):
    """
    Mục đích: Lấy các thông tin chi tiết của mô hình như input/output, kích thước và kiểu dữ liệu.

    Chi tiết:

    Lấy thông tin input và output từ interpreter.
    Xác định chiều cao, chiều rộng của input từ shape.
    Kiểm tra xem mô hình có sử dụng kiểu dữ liệu float32 không.
    Trả về các thông tin này.
    """
    input_details = interpreter.get_input_details()  # Lấy thông tin về các input của mô hình
    output_details = interpreter.get_output_details()  # Lấy thông tin về các output của mô hình
    height = input_details[0]['shape'][1]  # Chiều cao của input
    width = input_details[0]['shape'][2]   # Chiều rộng của input
    floating_model = (input_details[0]['dtype'] == np.float32)  # Kiểm tra xem mô hình có sử dụng kiểu dữ liệu float32 hay không
    return input_details, output_details, height, width, floating_model

def detect_objects(interpreter, input_data, input_details, output_details, min_conf_threshold, imW, imH):
    """
    Mục đích: Phát hiện các đối tượng trong khung hình và trả về thông tin như hộp chứa, lớp, và độ tin cậy.

    Chi tiết:

    Cung cấp dữ liệu đầu vào cho mô hình (set_tensor) và khởi chạy mô hình (invoke).
    Lấy kết quả từ output của mô hình.
    Duyệt qua danh sách kết quả để lọc các đối tượng có độ tin cậy lớn hơn ngưỡng (min_conf_threshold).
    Tính toán tọa độ hộp chứa (xmin, ymin, xmax, ymax).
    Trả về danh sách các đối tượng được phát hiện.
    """
    interpreter.set_tensor(input_details[0]['index'], input_data)  # Cung cấp dữ liệu vào input của mô hình
    interpreter.invoke()  # Khởi chạy mô hình
    
    outname = output_details[0]['name']  # Lấy tên output của mô hình
    if ('StatefulPartitionedCall' in outname):  # Kiểm tra nếu mô hình có sự phân chia stateful
        boxes_idx, classes_idx, scores_idx = 1, 3, 0
    else:
        boxes_idx, classes_idx, scores_idx = 0, 1, 2
    # Dùng các chỉ số đã gán từ trên để truy cập các tensor output
    boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0]  # Hộp chứa đối tượng
    classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0]  # Lớp đối tượng
    scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0]  # Độ tin cậy của đối tượng

    detections = []
    for i in range(len(scores)):  # Duyệt qua tất cả các đối tượng được phát hiện
        if scores[i] > min_conf_threshold and scores[i] <= 1.0:  # Kiểm tra độ tin cậy đối tượng
            ymin = int(max(1, (boxes[i][0] * imH)))  # Tính toán tọa độ y_min
            xmin = int(max(1, (boxes[i][1] * imW)))  # Tính toán tọa độ x_min
            ymax = int(min(imH, (boxes[i][2] * imH)))  # Tính toán tọa độ y_max
            xmax = int(min(imW, (boxes[i][3] * imW)))  # Tính toán tọa độ x_max
            detections.append((xmin, ymin, xmax, ymax, classes[i], scores[i]))  # Thêm thông tin đối tượng vào danh sách detections
    return detections


def draw_detections(frame, detections, labels, min_conf_threshold, imW, imH):
    """
    Mục đích: Vẽ các hộp chứa và nhãn lên khung hình.

    Chi tiết:

    Lặp qua danh sách các đối tượng được phát hiện.
    Vẽ hộp chứa (cv2.rectangle) và nền nhãn.
    Hiển thị nhãn với tên và độ tin cậy của đối tượng.
    """
    for xmin, ymin, xmax, ymax, class_id, score in detections:
        cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)  # Vẽ hình chữ nhật bao quanh đối tượng
        object_name = labels[int(class_id)]  # Lấy tên đối tượng từ nhãn
        label = '%s: %.2f%%' % (object_name, score * 100)  # Tạo nhãn với tên và độ tin cậy của đối tượng
        label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)  # Tính toán kích thước nhãn
        label_ymin = max(ymin, label_size[1] + 10)  # Đảm bảo nhãn không bị vướng vào hộp chứa
        cv2.rectangle(frame, (xmin, label_ymin - label_size[1] - 10),
                      (xmin + label_size[0], label_ymin + base_line - 10), (255, 255, 255), cv2.FILLED)  # Vẽ nền cho nhãn
        cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2)  # Vẽ nhãn lên khung hình
    return frame


def preprocess_frame(frame, width, height, floating_model):
    """
    Mục đích: Tiền xử lý khung hình để phù hợp với đầu vào của mô hình.

    Chi tiết:

    Chuyển đổi khung hình từ BGR sang RGB.
    Thay đổi kích thước khung hình theo yêu cầu của mô hình.
    Thêm chiều batch vào dữ liệu đầu vào (expand_dims).
    Chuẩn hóa giá trị pixel nếu mô hình sử dụng float32.
    """
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Chuyển đổi từ BGR sang RGB (mô hình TensorFlow Lite yêu cầu RGB)
    frame_resized = cv2.resize(frame_rgb, (width, height))  # Thay đổi kích thước khung hình
    input_data = np.expand_dims(frame_resized, axis=0)  # Thêm một chiều batch vào đầu dữ liệu (mô hình yêu cầu một batch)
    if floating_model:
        input_data = (np.float32(input_data) - 127.5) / 127.5  # Chuẩn hóa dữ liệu nếu mô hình sử dụng kiểu dữ liệu float32
    return input_data

def calculate_fps(t1, t2, freq):
    """
    Mục đích: Tính toán số khung hình mỗi giây (FPS).

    Chi tiết:

    Tính thời gian giữa hai khung hình ((t2 - t1) / freq).
    Lấy nghịch đảo của thời gian để tính FPS.
    """
    time1 = (t2 - t1) / freq  # Tính thời gian giữa hai khung hình
    return 1 / time1  # Trả về FPS

def detection_callback():
    """
    Hàm kiểm tra vật có được phát hiện hay không.
    Trả về True nếu phát hiện được vật, ngược lại trả về False.
    """
    global status  # Biến trạng thái phát hiện vật
    return status