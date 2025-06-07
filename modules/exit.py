from djitellopy import Tello
import threading
import sys
import cv2
import time
import os
import msvcrt  # Windows特定库，用于非阻塞键盘输入


class Exit(threading.Thread):
    def __init__(self, threads_to_stop=None, tello=None, exit_event=None):
        super().__init__()
        self.threads_to_stop = threads_to_stop or []  # 存储所有需要停止的线程
        self.tello = tello  # Tello无人机实例，可能为None
        self.keepRunning = True
        self.daemon = True  # 设置为守护线程，这样主线程结束时它会自动结束
        self.exit_event = exit_event or threading.Event()
        self.name = "ExitThread"  # 设置线程名

    def add_thread(self, thread):
        """添加需要停止的线程"""
        if thread not in self.threads_to_stop:
            self.threads_to_stop.append(thread)

    def add_threads(self, threads):
        """添加多个需要停止的线程"""
        for thread in threads:
            self.add_thread(thread)

    def run(self):
        print("Exit线程已启动 - 在控制台输入q并按回车退出")
        self.exit_handler()

    def exit_handler(self):
        """处理退出逻辑 - 使用非阻塞输入"""
        while self.keepRunning and not self.exit_event.is_set():
            # 检查是否有键盘输入
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8').lower()
                if key == 'q':
                    print("\n'q'键被按下，正在退出程序...")
                    self.stop_all()
                    break
            
            # 短暂休息，减少CPU使用
            time.sleep(0.1)

    def stop_all(self):
        """停止所有线程并清理资源"""
        print("正在停止所有线程...")
        
        # 设置退出事件
        self.exit_event.set()
        
        # 1. 首先停止无人机（如果有）
        if self.tello is not None:
            try:
                print("正在让无人机安全降落...")
                self.tello.land()
                time.sleep(2)  # 给无人机足够的时间降落
                self.tello.streamoff()
                self.tello.end()
                print("无人机已安全降落并断开连接")
            except Exception as e:
                print(f"停止无人机时出错: {e}")
        
        # # 2. 停止所有线程
        # for thread in self.threads_to_stop:
        #     try:
        #         if thread is not None and thread.is_alive():
        #             print(f"正在停止线程: {thread.name}")
        #             if hasattr(thread, 'stop') and callable(getattr(thread, 'stop')):
        #                 thread.stop()
        #             elif hasattr(thread, 'keepRunning'):
        #                 thread.keepRunning = False
        #     except Exception as e:
        #         print(f"停止线程时出错: {e}")
        
        # # 3. 等待所有线程结束
        # timeout = 3  # 超时时间（秒）
        # for thread in self.threads_to_stop:
        #     if thread is not None and thread.is_alive():
        #         print(f"等待线程结束: {thread.name}")
        #         thread.join(timeout=timeout)
        #         if thread.is_alive():
        #             print(f"警告: 线程 {thread.name} 在超时时间内未结束")
        
        # 4. 关闭OpenCV窗口
        try:
            cv2.destroyAllWindows()
            print("已关闭所有OpenCV窗口")
        except:
            pass
        
        # 5. 设置停止标志
        self.keepRunning = False
        
        # 6. 通知主线程退出
        self.exit_event.set()