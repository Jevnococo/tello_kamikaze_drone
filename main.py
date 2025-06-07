import queue
import threading
import numpy as np
from djitellopy import Tello
import time
import cv2
from modules.detector import yolo_detect, camera
from modules.tracker import Tracker, Laser_thread_sim
from modules.laser import Laser_thread as tello_Laser_thread
from modules.video_stream import VideoStream
from modules.exit import Exit
import modules.tello_pid_controller as tello_pid_controller
video_frame_queue = queue.Queue(maxsize=1)
target_queue = queue.Queue(maxsize=1)  # 用于存储需要跟踪的行人
laser_queue = queue.Queue(maxsize=1)

# # 创建无人机实例
tello = Tello()
# # 连接无人机
tello.connect()
print('TT_battery：', tello.get_battery())
tello.takeoff()
# 启动视频流线程
videostream = VideoStream(tello, video_frame_queue)
videostream.start()

# 用电脑摄像头模拟无人机前视摄像头
# a = threading.Thread(target=camera,args=(video_frame_queue,))
# a.start()
# 等待摄像头准备就绪
while True:
    if not video_frame_queue.empty():
        print("Camera feed is ready.")
        break

# 启动YOLO检测线程
if not video_frame_queue.empty():
    print("Starting YOLO detection thread...")
    yolo = yolo_detect(video_frame_queue, target_queue)
    yolo.daemon = True
    yolo.start()

# 等待目标队列被填充
while True:
    if not target_queue.empty():
        print("Target queue is ready.", target_queue.queue)
        break
    else:
        print("Waiting for target queue to be populated...")
        time.sleep(1)

# 启动跟踪器线程和激光线程
if not target_queue.empty():
    #模拟激光测距数据
    # laser_thread = Laser_thread_sim(laser_queue)
    #真实激光测距数据
    laser_thread = tello_Laser_thread(tello, laser_queue)
    laser_thread.daemon = True
    laser_thread.start()
    # 启动跟踪器线程
    tracker = Tracker(tello, target_queue, laser_queue)
    # 使用PID控制器进行跟踪
    # tracker = tello_pid_controller.TelloController(tello, target_queue, laser_queue)
    tracker.daemon = True
    tracker.start()
exit_event = threading.Event()
threads_to_stop = [videostream, yolo, laser_thread, tracker]
exit_thread = Exit(threads_to_stop, Tello(), exit_event)
exit_thread.daemon = True
exit_thread.start()

while True:
    time.sleep(1)  # 主线程保持运行状态
# exit_window_name = "按ESC退出程序"
# cv2.namedWindow(exit_window_name, cv2.WINDOW_NORMAL)
# cv2.resizeWindow(exit_window_name, 300, 100)
# exit_image = np.zeros((100, 300, 3), dtype=np.uint8)
# cv2.putText(exit_image, "按ESC退出程序", (50, 50), 
#             cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
# while True:
#     cv2.imshow(exit_window_name, exit_image)
#     key = cv2.waitKey(100) & 0xFF
#     if key == 27:  # ESC键
#         tracker.join()
#         laser_thread.join()
#         yolo.join()
#         a.join()
#         cv2.destroyAllWindows()
#         print("ESC键被按下,正在退出...")
#         # tello.streamoff()
#         # tello.land()
#         break
#     time.sleep(0.1)  # 避免CPU过度使用
