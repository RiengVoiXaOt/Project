import cv2
from threading import Thread, Lock

class VideoStream:
    """
    Quản lý luồng video từ camera.
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
        Khởi chạy luồng video trong một thread riêng.
        """
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        """
        Cập nhật liên tục các khung hình từ camera.
        """
        while not self.stopped:
            grabbed, frame = self.stream.read()
            with self.lock:
                self.grabbed, self.frame = grabbed, frame

    def read(self):
        """
        Đọc khung hình hiện tại.
        """
        with self.lock:
            return self.frame.copy()

    def stop(self):
        """
        Dừng luồng video.
        """
        self.stopped = True
        self.stream.release()
