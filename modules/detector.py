import cv2
import numpy as np
from .yolo_detector import const as const
# import yolo_detector.kal_utils as kal_utils
from .yolo_detector import kal_utils
from .yolo_detector.kalman import Kalman
from .yolo_detector.v5npredicate import detect_pedestrians
import queue
import threading

video_frame_queue = queue.Queue(maxsize=1)
# 偏移量 [ x偏移比例, y偏移比例， 宽度比例， box（中心x,中心y,宽w,高h,dx,dy）] ]
target_queue = queue.Queue(maxsize=1)  # 用于存储需要跟踪的行人

class yolo_detect(threading.Thread):
    def __init__(self, video_frame_queue:queue.Queue, target_queue:queue.Queue):
        super().__init__()
        self.A = np.array([ [1, 0, 0, 0, 1, 0],
                            [0, 1, 0, 0, 0, 1],
                            [0, 0, 1, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 1] ])
        self.B = None
        self.Q = np.eye(self.A.shape[0]) * 0.1
        self.H = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0]])
        self.R = np.eye(self.H.shape[0]) * 1
        self.P = np.eye(self.A.shape[0])
        self.video_frame_queue = video_frame_queue
        self.target_queue = target_queue
    def run(self):
        print("Starting YOLO detection thread...")
        self.detect()

    def detect(self):
        # 1.载入视频
        frame = self.video_frame_queue.get(timeout=5)  # 获取第一帧以获取视频尺寸
        print("获取第一帧以获取视频尺寸")
        sz = (frame.shape[1], frame.shape[0])  # 视频帧的宽和高
        # 初始化视频写入器
        video_writer = cv2.VideoWriter(const.VIDEO_OUTPUT_PATH, cv2.VideoWriter_fourcc(*'XVID'), const.FPS, sz, True)
        
        # 2. 逐帧观测并且滤波
        state_list = []  # 单帧目标状态信息，存kalman对象
        frame_cnt = 1
        
        while True:
            try:
                frame = self.video_frame_queue.get(timeout=5)  # 获取下一帧图像
            except queue.Empty:
                print("No more frames to process.")
                pass
            
            meas_list_frame = detect_pedestrians(frame, conf_thres=const.CONF_THRES, weights_path=const.WEIGHTS_PATH, yolov5_repo_path=const.yolov5_repo_path)    
            
            # 预测
            for target in state_list:
                target.predict()
                # print(target.X_prior)
            # 关联
            mea_list = [kal_utils.box2meas(mea) for mea in meas_list_frame]
            state_rem_list, mea_rem_list, match_list = Kalman.association(state_list, mea_list)
            # 状态没匹配上的，更新一下，如果触发终止就删除
            state_del = list()
            for idx in state_rem_list:
                status, _, _ = state_list[idx].update()
                if not status:
                    state_del.append(idx)
            state_list = [state_list[i] for i in range(len(state_list)) if i not in state_del]
            # 量测没匹配上的，作为新生目标进行航迹起始
            for idx in mea_rem_list:
                state_list.append(Kalman(self.A, self.B, self.H, self.Q, self.R, kal_utils.mea2state(mea_list[idx]), self.P))
                
            # -----------------------------------------------可视化-----------------------------------
            # 显示所有mea到图像上
            for mea in meas_list_frame:
                cv2.rectangle(frame, tuple(mea[:2]), tuple(mea[2:]), const.COLOR_MEA, thickness=1)
            # 显示所有的state到图像上
            for kalman in state_list:
                pos = kal_utils.state2box(kalman.X_posterior)
                cv2.rectangle(frame, tuple(pos[:2]), tuple(pos[2:]), const.COLOR_STA, thickness=2)
            # 将匹配关系画出来
            for item in match_list:
                cv2.line(frame, tuple(item[0][:2]), tuple(item[1][:2]), const.COLOR_MATCH, 3)
                
            # 计算图像中心
            center_x = frame.shape[1] // 2
            center_y = frame.shape[0] // 2
            
            # 找到距离中心最近的行人
            min_dist = float(0)
            closest_kalman = None
            
            for kalman in state_list:
                if len(kalman.track) > 0:
                    # 获取当前位置（最后一个轨迹点）
                    current_pos = kalman.track[-1]
                    # print("当前行人位置：", current_pos)
                    X_prior = kalman.X_prior
                    # print(X_prior)
                    if X_prior is None:
                        continue
                    # 计算与图像中心的距离
                    dist = X_prior[2]
                    
                    if dist > min_dist:
                        min_dist = dist
                        closest_kalman = kalman
            
            # 只绘制距离中心最近的行人的轨迹
            if closest_kalman is not None:
            
                # 计算偏移量 [ x偏移比例, y偏移比例， 宽度比例， box（中心x,中心y,宽w,高h,dx,dy）] ]
                bias = [ (closest_kalman.X_prior[0] - center_x)/center_x,
                         (center_y - closest_kalman.X_prior[1])/center_y,
                          closest_kalman.X_prior[2]/(2*center_x),
                          closest_kalman.X_prior ]
                if self.target_queue.full():
                    self.target_queue.get_nowait()
                    self.target_queue.put_nowait(bias)
                else:
                    self.target_queue.put_nowait(bias)
                tracks_list = closest_kalman.track
                for idx in range(len(tracks_list) - 1):
                    last_frame = tracks_list[idx]
                    cur_frame = tracks_list[idx + 1]
                    # 使用不同的颜色和粗细来突出显示最近的行人轨迹
                    cv2.line(frame, last_frame, cur_frame, const.RED, 3)  # 使用红色粗线
                
                # 可选：在最近行人上方添加标识
                pos = kal_utils.state2box(closest_kalman.X_posterior)
                cv2.putText(frame, "TRACKED", 
                        (pos[0], pos[1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, const.RED, 2)
            
            cv2.putText(frame, str(frame_cnt), (0, 50), color=const.RED, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1.5)
            cv2.imshow('Demo', frame)
            cv2.imwrite("./image/{}.jpg".format(frame_cnt), frame)
            video_writer.write(frame)
            cv2.waitKey(100)  # 显示 1000 ms 即 1s 后消失
            frame_cnt += 1

        cv2.destroyAllWindows()
        video_writer.release()

def camera(video_frame_queue: queue.Queue):
    cap = cv2.VideoCapture(0)  # 使用摄像头
    if not cap.isOpened():
        print("Error: Could not open video.")
        exit()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break
        
        if video_frame_queue.full():
            video_frame_queue.get_nowait()
        video_frame_queue.put_nowait(frame)
        
        cv2.imshow('Camera Feed', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    import yolo_detector.const as const
    # video_frame_queue = queue.Queue(maxsize=1)
    # target_queue = queue.Queue(maxsize=1)  # 用于存储需要跟踪的行人
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
    if not target_queue.empty():
        print("Target queue is ready.", target_queue.queue)
    # yolo = yolo_detect(video_frame_queue)
    # yolo.start()
    