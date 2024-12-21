from src.vision.object_detection import object_detection_loop
from src.vision.color_detection import color_detection_loop
from src.hardware.battery import BatteryMonitor
from src.vision.video_stream import VideoStream
from threading import Thread, Event, Lock
from queue import Queue
from time import sleep
import cv2
from datetime import datetime

OPERATION_START_TIME = datetime.now().replace(hour=12, minute=40,second=0)
now = datetime.now()

battery = BatteryMonitor()
_, _, _, battery_percentage, _ = battery.read_battery_status()

# Định dạng thời gian theo ý muốn
def Auto():
    stop_event = Event()
    frame_queue = Queue(maxsize=1)
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
                frame_color, mask_red, mask_black, status_red, deviation_x_red, deviation_y_red, \
                    status_black, deviation_x_black, deviation_y_black, contours_black, contours_red, \
                    status_water, status_charger, deviation_x_water, deviation_y_water, \
                    deviation_x_charger, deviation_y_charger, frame_object = (frame_queue.get())

                # Kiểm tra xem đã đến thời điểm hoạt động chưa
                if now >= OPERATION_START_TIME:
                    print(f"Robot bắt đầu hoạt động vào {now.strftime('%Y-%m-%d %H:%M:%S')}")
                    if battery_percentage > 25 
                        if not mission :
                            rest_in_charger()  
                        else :
                            leave_charger()
                            complete_mission()
                    else:
                        if present_status == "charging":
                            rest_in_charger()
                        else:
                            search_for_charger()
                            
                else:
                    if present_status == "charging":
                        rest_in_charger()
                    else:
                        search_for_charger()
                    
            
            
            
            
            
            
            
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