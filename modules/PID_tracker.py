import queue
import threading
import numpy as np
from djitellopy import Tello
import time
from .LED import attack_sim
from .PID_controller import PIDController
target_queue = queue.Queue(maxsize=1)
laser_queue = queue.Queue(maxsize=1)

class Tracker(threading.Thread):
    def __init__(self, tello: Tello, target_queue: queue.Queue, laser_queue: queue.Queue):
        super().__init__()
        self.tello = tello
        self.target_queue = target_queue
        self.laser_queue = laser_queue
        self.keepTracking = True
        self.target_lost_timeout = 3.0
        self.last_target_time = time.time()
        self.target_count = 0
        self.yaw_pid = PIDController(
            kp=25.0,  # 比例增益
            ki=0.1,   # 积分增益
            kd=5.0,   # 微分增益
            setpoint=0.0,  # 目标值：无偏移
            output_limits=(-50, 50)  # 输出限制
        )
        
        self.ud_pid = PIDController(
            kp=25.0,
            ki=0.1,
            kd=5.0,
            setpoint=0.0,
            output_limits=(-50, 50)
        )

        self.fb_pid = PIDController(
            kp=50.0,
            ki=0.0,
            kd=10.0,
            setpoint=0.6,  # 目标距离为0.5（标准化值）
            output_limits=(0, 100)
        )
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

                self.last_target_time = time.time()
                self.target_count += 1

                # print(f"Processing target: {target}")
                if target is not None:
                    # 调整无人机位置
                    yaw_velocity = int(self.yaw_pid.compute(target[0]))
                    ud_velocity = int(self.ud_pid.compute(target[1]))

                    video_dist = target[2]
                    self.fb_pid.setpoint = 0.6  # 期望距离
                    fb_velocity = int(self.fb_pid.compute(video_dist))
                    if self.laser_queue.empty():
                        time.sleep(0.1)  # 等待激光数据
                        # print("Laser queue is empty, waiting for data...")
                        laser_dist = 65
                    else:
                        laser_dist = self.laser_queue.get()
                    
                    # 激光测距小于25cm或视频测距大于0.6m时，模拟攻击
                    if laser_dist < 40 or video_dist > 0.6:
                        print("Laser distance too close, activating attack simulation.")
                        attack_sim(self.tello)
                        self.tello.land()  # 模拟攻击后降落  

                    # 旧前进速度计算方式
                    # fb_velocity = min(int(np.clip(laser_dist - 40, 0, 50)), int(np.clip(50*(0.5-video_dist)/0.7, 0, 50)))  # 前后速度，目标距离40cm时速度为0，超过40cm时速度增加
                    # 输出调试信息
                    # print(f"offset: ({target[0]}, {target[1]})")
                    # print(f"Control velocities - lr: {yaw_velocity}, ud: {ud_velocity}, fb: {fb_velocity}")
                    
                    # 发送RC控制命令
                    self.tello.send_rc_control(0, fb_velocity*2, ud_velocity, yaw_velocity)
                    # RC控制使用以下输出模拟
                    # print(f"RC Control - Left/Right: 0, Forward/Backward: {fb_velocity}, Up/Down: {ud_velocity}, Yaw: {yaw_velocity}")
            else:
                if time.time() - self.last_target_time > self.target_lost_timeout:
                        # 目标丢失，执行搜索行为
                        self.handle_lost_target()
            time.sleep(0.3)

    def handle_lost_target(self):
        """处理目标丢失的情况"""
        # 重置PID控制器积分项，防止积分饱和
        self.yaw_pid.reset()
        self.ud_pid.reset()
        self.fb_pid.reset()
        
        # 执行搜索模式 - 原地悬停并缓慢旋转
        self.tello.send_rc_control(0, 0, 0, 20)  # 缓慢右转
        print("目标丢失，执行搜索模式...")

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