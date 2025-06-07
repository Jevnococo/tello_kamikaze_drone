import torch
import numpy as np
import cv2
from PIL import Image
import time
import sys
from pathlib import Path
def detect_pedestrians(image, conf_thres=0.25, weights_path=None, yolov5_repo_path=None):
    """
    使用YOLOv5n.pt检测图片中的行人
    
    参数:
        image: 图片路径或图像数组
        conf_thres: 置信度阈值
        weights_path: 本地YOLOv5n.pt权重文件路径
        
    返回:
        pedestrian_boxes: 行人边界框列表，格式为[x1,y1,x2,y2]
    """
    # 加载YOLOv5模型（仅在第一次调用时加载）
    if not hasattr(detect_pedestrians, 'model'):
        if weights_path:
            # 使用本地权重文件
            print(f"加载本地权重文件: {weights_path}")
            detect_pedestrians.model = torch.hub.load(yolov5_repo_path, 'custom', path=weights_path, source='local', force_reload=True)
        else:
            # 使用在线模型
            print("未提供权重路径，使用默认的YOLOv5n模型")
            detect_pedestrians.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
            pass
        
        # 设置参数
        detect_pedestrians.model.conf = conf_thres  # 置信度阈值
        detect_pedestrians.model.classes = [0]      # 只检测行人类别 (COCO数据集中行人的类别ID为0)
    # DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
    # model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path)
    # 执行推理
    results = detect_pedestrians.model(image)
    
    # 提取检测结果
    detections = results.xyxy[0].cpu().numpy()  # xyxy格式的检测结果
    
    # 创建行人框列表
    pedestrian_boxes = []
    for detection in detections:
        x1, y1, x2, y2, conf, cls = detection
        pedestrian_boxes.append([int(x1), int(y1), int(x2), int(y2)])
    
    return pedestrian_boxes


def visualize_detections(image, boxes):
    """可视化检测结果"""
    # 如果输入是路径，读取图像
    if isinstance(image, str):
        image = cv2.imread(image)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # 复制图像以防止修改原图
    vis_image = image.copy()
    
    for box in boxes:
        x1, y1, x2, y2 = box
        cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    
    if vis_image.shape[2] == 3 and vis_image.dtype == np.uint8:
        vis_image = cv2.cvtColor(vis_image, cv2.COLOR_RGB2BGR)
    
    return vis_image

def process_video(video_source, weights_path=None, conf_thres=0.25, save_output=None):
    """
    处理视频或摄像头流中的行人检测
    
    参数:
        video_source: 视频文件路径或摄像头索引(0表示默认摄像头)
        weights_path: YOLOv5模型权重路径
        conf_thres: 置信度阈值
        save_output: 输出视频保存路径(可选)
    """
    # 打开视频源
    cap = cv2.VideoCapture(video_source)
    if not cap.isOpened():
        print(f"无法打开视频源: {video_source}")
        return

    # 获取视频属性
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # 设置视频写入器
    out = None
    if save_output:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(save_output, fourcc, fps, (width, height))
    
    # 处理每一帧
    frame_count = 0
    start_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        frame_count += 1
        
        # 检测行人
        pedestrian_boxes = detect_pedestrians(frame, conf_thres=conf_thres, weights_path=weights_path)
        
        # 添加检测结果及FPS信息到帧
        processed_frame = visualize_detections(frame, pedestrian_boxes)
        
        # 计算FPS
        elapsed_time = time.time() - start_time
        if elapsed_time > 0:
            fps_text = f"FPS: {frame_count / elapsed_time:.2f}"
            cv2.putText(processed_frame, fps_text, (10, 30), 
                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # 显示检测到的行人数量
        ped_text = f"行人: {len(pedestrian_boxes)}"
        cv2.putText(processed_frame, ped_text, (10, 70), 
                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # 显示结果
        cv2.imshow("Pedestrian Detection", processed_frame)
        
        # 保存结果
        if out:
            out.write(processed_frame)
        
        # 按'q'退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    # 释放资源
    cap.release()
    if out:
        out.release()
    cv2.destroyAllWindows()
    
    print(f"处理了 {frame_count} 帧，平均 FPS: {frame_count / (time.time() - start_time):.2f}")

if __name__ == "__main__":
    # 本地权重文件路径
    weights_path = r"C:\Users\JEVNOCOCO\OneDrive\Desktop\week7\yolov5n.pt"
    
    # 选择要运行的模式
    mode = input("请选择模式 (1: 图像, 2: 视频文件, 3: 摄像头): ")
    
    if mode == "1":
        # 处理单张图像
        image_path = r"C:\Users\JEVNOCOCO\OneDrive\Desktop\week7\image.png"
        pedestrian_boxes = detect_pedestrians(image_path, weights_path=weights_path)
        print(f"检测到 {len(pedestrian_boxes)} 个行人")
        for i, box in enumerate(pedestrian_boxes):
            print(f"行人 {i+1}: {box}")
        visualized = visualize_detections(image_path, pedestrian_boxes)
        cv2.imshow("Pedestrian Detection", visualized)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    elif mode == "2":
        # 处理视频文件
        video_path = input("请输入视频文件路径: ")
        output_path = input("请输入输出视频路径 (直接回车不保存): ")
        if not output_path:
            output_path = None
        process_video(video_path, weights_path=weights_path, save_output=output_path)
    
    elif mode == "3":
        # 使用摄像头
        camera_id = 0  # 默认摄像头
        output_path = input("请输入输出视频路径 (直接回车不保存): ")
        if not output_path:
            output_path = None
        process_video(camera_id, weights_path=weights_path, save_output=output_path)
    
    else:
        print("无效的选择!")