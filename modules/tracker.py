import queue
import threading
import numpy as np
from djitellopy import Tello
import time
from .LED import attack_sim

target_queue = queue.Queue(maxsize=1)
laser_queue = queue.Queue(maxsize=1)

class Tracker(threading.Thread):
    def __init__(self, tello: Tello, target_queue: queue.Queue, laser_queue: queue.Queue):
        super().__init__()
        self.tello = tello
        self.target_queue = target_queue
        self.laser_queue = laser_queue
        self.keepTracking = True

    def run(self):
        print("Starting Tracker thread...")
        self.track()

    def track(self):
        while self.keepTracking:
            # print("Tracking...")
            if not self.target_queue.empty():
                # 偏移量 [ x偏移, y偏移 ]
                # print("Target queue is not empty, processing targets...")
                target = self.target_queue.get()
                # print(f"Processing target: {target}")
                if target is not None:
                    # 调整无人机位置
                    yaw_velocity = int(np.clip(target[0] * 50, -50, 50))
                    ud_velocity = int(np.clip(target[1] * 50, -50, 50))
                    video_dist = target[2]
                    if self.laser_queue.empty():
                        time.sleep(0.1)  # 等待激光数据
                        # print("Laser queue is empty, waiting for data...")
                        laser_dist = 25
                    else:
                        laser_dist = self.laser_queue.get()
                    if laser_dist < 25 or video_dist > 0.7:
                        print("Laser distance too close, activating attack simulation.")
                        attack_sim(self.tello)
                        self.tello.land()  # 模拟攻击后降落  
                    fb_velocity = min(int(np.clip(65 - 40, 0, 50)), int(np.clip(50*(0.5-video_dist)/0.7, 0, 50)))  # 前后速度，目标距离40cm时速度为0，超过40cm时速度增加
                    # 输出调试信息
                    # print(f"offset: ({target[0]}, {target[1]})")
                    # print(f"Control velocities - lr: {yaw_velocity}, ud: {ud_velocity}, fb: {fb_velocity}")
                    
                    # 发送RC控制命令
                    self.tello.send_rc_control(0, fb_velocity*2, ud_velocity, yaw_velocity)
                    # RC控制使用以下输出模拟
                    # print(f"RC Control - Left/Right: 0, Forward/Backward: {fb_velocity}, Up/Down: {ud_velocity}, Yaw: {yaw_velocity}")
            # else:
            #     print("No target to track.")
            time.sleep(0.3)

class Laser_thread_sim(threading.Thread):
    def __init__(self, laser_queue: queue.Queue):
        super().__init__()
        self.laser_queue = laser_queue
        self.keepRunning = True

    def run(self):
        print("Starting Laser thread...")
        self.read_laser()

    def read_laser(self):
        while self.keepRunning:
            # 模拟激光测距数据
            laser_distance = np.random.randint(0, 100)  # 模拟30到100cm的距离
            if self.laser_queue.full():
                self.laser_queue.get()  # 清空队列中的旧数据
            self.laser_queue.put(laser_distance)
            print(f"Laser distance: {laser_distance} cm")
            time.sleep(0.5)  # 模拟激光读取间隔



if __name__ == "__main__":
    from detector import yolo_detect, camera
    video_frame_queue = queue.Queue(maxsize=1)
    target_queue = queue.Queue(maxsize=1)  # 用于存储需要跟踪的行人
    laser_queue = queue.Queue(maxsize=1)
    a = threading.Thread(target=camera,args=(video_frame_queue,))
    a.start()
    while True:
        if not video_frame_queue.empty():
            print("Camera feed is ready.")
            break
        
    if not video_frame_queue.empty():
        print("Starting YOLO detection thread...")
        yolo = yolo_detect(video_frame_queue, target_queue)
        yolo.start()
    
    while True:
        if not target_queue.empty():
            print("Target queue is ready.", target_queue.queue)
            break
        else:
            print("Waiting for target queue to be populated...")
            time.sleep(1)
    # 启动跟踪器线程
    if not target_queue.empty():
        laser_thread = Laser_thread_sim(laser_queue)
        laser_thread.start()
        tracker = Tracker(Tello(), target_queue, laser_queue)
        tracker.start()
    # yolo = yolo_detect(video_frame_queue)
    # yolo.start()