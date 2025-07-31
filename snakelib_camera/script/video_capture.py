import queue
import threading

import cv2


# bufferless VideoCapture
class VideoCapture:
    def __init__(self, cap):
        self.cap = cap

        # Thread-safe frame storage object
        self.q = queue.Queue()

        # Frame collection occurs on a different thread which only holds the most recent frame
        t = threading.Thread(target=self._reader)
        t.daemon = True
        self.running = True
        t.start()

    def _reader(self):
        """
        Read frames as soon as they are available but keep only most recent one
        """
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                break
            if not self.q.empty():
                try:
                    self.q.get_nowait()  # discard previous (unprocessed) frame
                except queue.Empty:
                    pass
            self.q.put(frame)

    def read(self):
        """
        Extract the most recent frame from the queue and return
        """
        try:
            return True, self.q.get(timeout=0.2)
        except Exception:
            return self.cap.read()

    def release(self):
        """
        Stop sending more tasks to the frame collection thread (safe way to join the thread)
        Release the OpenCV VideoCapture Object
        """
        self.running = False
        self.cap.release()
