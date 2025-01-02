import cv2
import time
from src.utils.image_utils import (
    process_frame,
    analyze_contours,
    display_info,
    calculate_fps
)

# Hàm chính để thực hiện nhận diện màu sắc trong video (đỏ và đen)
# def color_detection_loop(videostream, center_x, center_y, min_box_area, stop_event, frame_queue, max_fps=10):
#     """
#     Mục đích: Phát hiện và theo dõi các đối tượng có màu đỏ và đen trong video.

#     Chi tiết:
#     - Đọc từng khung hình từ luồng video.
#     - Tạo các mặt nạ (mask) cho màu đỏ và màu đen trong không gian màu HSV.
#     - Phân tích các vùng màu sắc và vẽ các hình chữ nhật bao quanh đối tượng.
#     - Giới hạn FPS để giảm tải tài nguyên xử lý.
#     """
#     frame_rate_calc = 1  # Khởi tạo FPS
#     freq = cv2.getTickFrequency()  # Lấy tần số đồng hồ OpenCV
#     frame_interval = 1.0 / max_fps  # Tính toán thời gian giữa các frame để giới hạn FPS
#     last_time = time.time()

#     while not stop_event.is_set():
#         current_time = time.time()
#         if current_time - last_time < frame_interval:
#             continue  # Bỏ qua nếu chưa đủ thời gian cho frame tiếp theo

#         frame = videostream.read()
#         if frame is None:
#             print("Cannot read from camera.")
#             break

#         t1 = cv2.getTickCount()  # Bắt đầu theo dõi thời gian

#         # Phân tích khung hình để tạo mask cho màu đỏ và đen
#         mask_red, mask_yellow = process_frame(frame)

#         # Phân tích các contour để tìm các đối tượng trong vùng mask
#         status_red, x_r, y_r, contours_red, x_red, y_red, w_red, h_red = analyze_contours(mask_red, center_x, center_y, min_box_area, "red")
#         status_yellow, x_b, y_b, contours_yellow, x_yellow, y_yellow, w_yellow, h_yellow = analyze_contours(mask_yellow, center_x, center_y, min_box_area, "yellow")

#         # Chuyển status thành chuỗi để hiển thị trên video
#         status_red = str(status_red)
#         status_yellow = str(status_yellow)
        
#         # Vẽ kết quả (cập nhật vào frame)
#         if contours_red:
#             cv2.rectangle(frame, (x_red, y_red), (x_red + w_red, y_red + h_red), (0, 0, 255), 2)
#             cv2.circle(frame, (x_red + w_red // 2, y_red + h_red // 2), 5, (0, 0, 255), -1)
#         if contours_yellow:
#             cv2.rectangle(frame, (x_yellow, y_yellow), (x_yellow + w_yellow, y_yellow + h_yellow), (255, 255, 255), 2)
#             cv2.circle(frame, (x_yellow + w_yellow // 2, y_yellow + h_yellow // 2), 5, (255, 255, 255), -1)

#         # Gửi kết quả qua hàng đợi để truyền cho phần hiển thị
#         if not frame_queue.full():
#             frame_queue.put((frame, None))

#         # Hiển thị thông tin FPS và các kết quả trên video
#         display_info(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), [status_red, status_yellow], [(x_r, y_r), (x_b, y_b)], center_x, center_y)
        
#         t2 = cv2.getTickCount()  # Lấy thời gian kết thúc
#         frame_rate_calc = calculate_fps(t1, t2, freq)  # Tính FPS
#         last_time = current_time  # Cập nhật thời gian cuối cùng


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

        mask_red, mask_yellow, mask_blue = process_frame(frame)
        status_red, x_r, y_r, contours_red, x_red, y_red, w_red, h_red = analyze_contours(mask_red, center_x, center_y, min_box_area, "red")
        status_yellow, x_y, y_y, contours_yellow, x_yellow, y_yellow, w_yellow, h_yellow = analyze_contours(mask_yellow, center_x, center_y, min_box_area, "yellow")
        status_blue, x_b, y_b, contours_blue, x_blue, y_blue, w_blue, h_blue = analyze_contours(mask_blue, center_x, center_y, min_box_area, "blue")

        # Gửi kết quả vào hàng đợi
        deviation_x_red = (x_red + w_red // 2) - center_x if contours_red else 0
        deviation_y_red = (y_r + h_red // 2) - center_y if contours_red else 0
        deviation_x_yellow = (x_yellow + w_yellow // 2) - center_x if contours_red else 0
        deviation_y_yellow = (y_yellow + h_yellow // 2) - center_y if contours_red else 0
        frame_queue.put((frame, mask_red, mask_yellow, status_red, deviation_x_red,deviation_y_red, status_yellow,deviation_x_yellow,deviation_y_yellow,contours_yellow,contours_red, status_blue,None,None,None,None,None,None,None))

        display_info(frame, 'FPS: {0:.2f}'.format(frame_rate_calc), [str(status_red), str(status_yellow)], [(x_r, y_r), (x_y, y_y)], center_x, center_y)

        t2 = cv2.getTickCount()
        frame_rate_calc = calculate_fps(t1, t2, freq)
        last_time = current_time