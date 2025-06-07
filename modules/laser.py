import threading
import queue
import time
from djitellopy import Tello

laser_queue = queue.Queue(maxsize=1)
class Laser_thread(threading.Thread):
    def __init__(self, tello: Tello ,laser_queue: queue.Queue):
        super().__init__()
        self.laser_queue = laser_queue
        self.keepRunning = True
        self.tello = tello

    def run(self):
        print("Starting Laser thread...")
        self.read_laser()

    def read_laser(self):
        while self.keepRunning:
            # 模拟激光测距数据
            laser_distance = self.tello.send_read_command('EXT tof?')  # 模拟30到100cm的距离
            try:
                laser_distance = int(laser_distance.split()[1])
            except (IndexError, ValueError):
                laser_distance = 200
            if self.laser_queue.full():
                self.laser_queue.get()  # 清空队列中的旧数据
            self.laser_queue.put(laser_distance)
            print(f"Laser distance: {laser_distance} cm")
            time.sleep(0.5)  # 模拟激光读取间隔

if __name__ == "__main__":
    tello = Tello()
    tello.connect()
    print("Battery:", tello.get_battery())
    
    laser_thread = Laser_thread(tello, laser_queue)
    laser_thread.start()

    # 模拟主线程运行
    try:
        while True:
            if not laser_queue.empty():
                distance = laser_queue.get()
                print(f"Main thread received laser distance: {distance} cm")
            time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting...")
        laser_thread.keepRunning = False
        laser_thread.join()
        tello.end()