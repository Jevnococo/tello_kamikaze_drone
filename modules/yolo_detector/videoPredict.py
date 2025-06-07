import cv2
import numpy as np
import kal_utils
import const
import measure
from kalman import Kalman
from v5npredicate import detect_pedestrians_offline
import queue
# --------------------------------Kalman参数---------------------------------------
# 状态转移矩阵，上一时刻的状态转移到当前时刻
A = np.array([[1, 0, 0, 0, 1, 0],
              [0, 1, 0, 0, 0, 1],
              [0, 0, 1, 0, 0, 0],
              [0, 0, 0, 1, 0, 0],
              [0, 0, 0, 0, 1, 0],
              [0, 0, 0, 0, 0, 1]])
# 控制输入矩阵B
B = None
# 过程噪声协方差矩阵Q，p(w)~N(0,Q)，噪声来自真实世界中的不确定性,
# 在跟踪任务当中，过程噪声来自于目标移动的不确定性（突然加速、减速、转弯等）
Q = np.eye(A.shape[0]) * 0.1
# 状态观测矩阵
H = np.array([[1, 0, 0, 0, 0, 0],
              [0, 1, 0, 0, 0, 0],
              [0, 0, 1, 0, 0, 0],
              [0, 0, 0, 1, 0, 0]])
# 观测噪声协方差矩阵R，p(v)~N(0,R)
# 观测噪声来自于检测框丢失、重叠等
R = np.eye(H.shape[0]) * 1
# 状态估计协方差矩阵P初始化
P = np.eye(A.shape[0])
# -------------------------------------------------------------------------------
to_track_queue = queue.Queue(1)  # 用于存储需要跟踪的行人
to_cruise_queue = queue.Queue()

def detect():
    # 1.载入视频
    cap = cv2.VideoCapture(const.VIDEO_PATH)  # 穿插着视频是为了方便展示
    sz = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
          int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    # 初始化视频写入器
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')  # opencv3.0
    video_writer = cv2.VideoWriter(const.VIDEO_OUTPUT_PATH, fourcc, const.FPS, sz, True)
    # 2. 逐帧观测并且滤波
    state_list = []  # 单帧目标状态信息，存kalman对象
    frame_cnt = 1
    while True:
        # --------------------------------------------加载当帧图像------------------------------------
        ret, frame = cap.read()
        if not ret:
            break
        
        # meas_list_all = measure.load_measurement(const.FILE_DIR)
        meas_list_frame = detect_pedestrians_offline(frame, conf_thres=const.CONF_THRES, weights_path = const.WEIGHTS_PATH)    

        # ---------------------------------------Kalman Filter for multi-objects-------------------
        # 预测
        for target in state_list:
            target.predict()
            print(target.X_prior)
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
            state_list.append(Kalman(A, B, H, Q, R, kal_utils.mea2state(mea_list[idx]), P))

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
        min_dist = float('inf')
        closest_kalman = None
        
        for kalman in state_list:
            if len(kalman.track) > 0:
                # 获取当前位置（最后一个轨迹点）
                current_pos = kalman.track[-1]
                X_prior = kalman.X_prior
                # 计算与图像中心的距离
                dist = X_prior[2]
                
                if dist > min_dist:
                    min_dist = dist
                    closest_kalman = kalman

        # 只绘制距离中心最近的行人的轨迹
        if closest_kalman is not None:
            
            # 计算偏移量 [ 偏移x, 偏移y, 距离, 中心x, 中心y ]
            bias = [ closest_kalman.X_prior[0] - center_x, center_y - closest_kalman.X_prior[1], closest_kalman.X_prior[2]/frame.shape[1], center_x, center_y]

            # 停止巡航
            to_cruise_queue.put("stop")

            # 将最近的行人加入队列
            if to_track_queue.full():
                to_track_queue.get()
            to_track_queue.put(bias)  # 将最近的行人加入队列

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

    cap.release()
    cv2.destroyAllWindows()
    video_writer.release()


if __name__ == '__main__':
    detect()
