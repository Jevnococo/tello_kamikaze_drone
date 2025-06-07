from djitellopy import Tello
import cv2
import threading
import queue
import time
# import tello_kamikaze_drone.utils.tello_const as tello_const
from . import tello_const

video_frame_queue = queue.Queue(maxsize=1)

class VideoStream(threading.Thread):
    def __init__(self, tello: Tello, video_streaming:queue.Queue):
        super().__init__()
        self.tello = tello
        self.video_streaming = video_streaming
        self.keepRecording = True

    def run(self):
        self.start_stream()
        self.get_frame()


    def start_stream(self):
        """
        Starts the video stream from the Tello drone.
        """
        self.tello.streamon()
        time.sleep(1)
        self.frame_read = self.tello.get_frame_read()
        if self.frame_read is not None:
            height, width, _ = self.frame_read.frame.shape
            self.video = cv2.VideoWriter(tello_const.ORIGINAL_VIDEO_PATH, cv2.VideoWriter_fourcc(*'XVID'),
                                    30, (width, height))

    def get_frame(self):
        """
        Retrieves a single frame from the video stream.
        """
        
        
        while self.keepRecording:
            if self.frame_read is not None:
                frame = cv2.cvtColor(self.frame_read.frame, cv2.COLOR_BGR2RGB)
                
                if self.video_streaming.full():
                    self.video_streaming.get()
                self.video_streaming.put(frame)
                time.sleep(0.1)
                cv2.imshow("Tello Stream", frame)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC退出
                    self.keepRecording = False

                self.video.write(frame)

        self.video.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print("Battery:", tello.get_battery())
    
    video_streaming = queue.Queue(maxsize=1)
    video_thread = VideoStream(tello, video_streaming)
    video_thread.start()

    try:
        while True:
            if not video_streaming.empty():
                frame = video_streaming.get()
                cv2.imshow("Tello Stream", frame)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC退出
                    break
    except KeyboardInterrupt:
        pass

    video_thread.keepRecording = False
    video_thread.join()
    tello.end()