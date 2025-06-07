# Tello无人机自杀式无人机系统架构

本项目目标跟踪部分使用卡尔曼滤波器实现视频流目标跟踪识别，代码基于https://github.com/ZhangPHEngr/Kalman-in-MOT实现。



基于Tello无人机和YOLOv5的察打一体自杀式无人机系统可以分为以下几个主要模块：（仅供参考，实际功能只有部分实现）

## 1. 系统总体架构

```
自杀式无人机系统
│
├── 控制模块 - 总体协调无人机行为
│   ├── 状态管理（巡航/侦察/跟踪/攻击）
│   └── 任务调度
│
├── 通信模块 - 与Tello无人机通信
│   ├── 命令发送
│   ├── 状态接收
│   └── 视频流接收
│
├── 感知模块 - 处理视频流和目标识别
│   ├── 视频预处理
│   ├── YOLOv5目标检测
│   └── 目标跟踪
│
├── 导航模块 - 控制无人机移动
│   ├── 巡航路径规划
│   ├── 目标跟踪导航
│   └── 攻击路径规划
│
└── 任务执行模块 - 执行具体任务
    ├── 巡航任务
    ├── 侦察识别统计
    ├── 目标跟踪
    └── 攻击模拟（LED控制）
```

## 2. 详细代码架构

### 项目目录结构

```
tello_kamikaze_drone/
│
├── main.py                  # 主程序入口
├── config.py                # 配置文件
├── requirements.txt         # 依赖包
│
├── modules/
│   ├── __init__.py
│   ├── drone_controller.py  # 无人机控制模块
│   ├── video_stream.py      # 视频流处理
│   ├── yolo_detector.py     # YOLOv5目标检测
│   ├── tracker.py           # 目标跟踪
│   ├── navigation.py        # 导航模块
│   └── mission.py           # 任务执行模块
│
├── utils/
│   ├── __init__.py
│   ├── logger.py            # 日志工具
│   └── visualization.py     # 可视化工具
│
└── models/                  # YOLOv5模型文件
    └── yolov5s.pt           # 预训练模型
```

# Tello自杀式无人机系统模块交互关系

以下是各模块之间的交互关系和数据流动图解：

## 1. 模块交互总体流程图

```
+----------------+    视频流    +----------------+    检测结果    +----------------+
|                |------------->|                |-------------->|                |
| VideoStream    |              | YoloDetector   |               | TargetTracker  |
|                |<-------------|                |<--------------|                |
+----------------+    帧请求    +----------------+    目标选择    +----------------+
        ^                                                  |
        |                                                  |
        | 视频流控制                                        | 目标位置信息
        |                                                  v
+----------------+    控制命令    +----------------+    导航指令    +----------------+
|                |-------------->|                |<--------------|                |
| DroneController|               | Navigation     |               | MissionExecutor|
|                |<--------------|                |-------------->|                |
+----------------+    状态反馈    +----------------+    导航状态    +----------------+
        ^                                                  ^
        |                                                  |
        | 无人机状态                                        | 任务控制
        +--------------------------------------------------+
```

## 2. 模块间详细交互关系

### 2.1 DroneController (无人机控制模块)

**提供服务：**
- 向Tello无人机发送控制命令（起飞、降落、移动、旋转等）
- 获取无人机状态信息（电量、飞行状态等）
- 控制LED灯模拟攻击状态

**接收数据：**
- 来自Navigation模块的导航指令
- 来自MissionExecutor的高级任务命令

**发送数据：**
- 向Navigation提供无人机当前状态信息
- 向VideoStream提供摄像头控制命令
- 向MissionExecutor报告无人机状态（电量、飞行状态）

### 2.2 VideoStream (视频流处理模块)

**提供服务：**
- 从Tello获取实时视频帧
- 处理和分发视频帧给其他模块

**接收数据：**
- 来自DroneController的摄像头控制命令

**发送数据：**
- 向YoloDetector提供处理后的视频帧
- 向MissionExecutor提供视频帧用于显示和记录

### 2.3 YoloDetector (目标检测模块)

**提供服务：**
- 使用YOLOv5模型对视频帧进行目标检测
- 识别和分类视频中的物体

**接收数据：**
- 来自VideoStream的视频帧
- 来自MissionExecutor的检测参数（如置信度阈值）

**发送数据：**
- 向TargetTracker提供检测结果（边界框、类别、置信度）
- 向MissionExecutor提供检测统计信息

### 2.4 TargetTracker (目标跟踪模块)

**提供服务：**
- 从检测结果中选择和跟踪特定目标
- 计算目标相对于无人机的位置和方向
- 维护目标历史轨迹

**接收数据：**
- 来自YoloDetector的检测结果
- 来自MissionExecutor的目标选择标准

**发送数据：**
- 向Navigation提供目标位置和方向信息
- 向MissionExecutor提供目标跟踪状态

### 2.5 Navigation (导航模块)

**提供服务：**
- 规划和执行巡航路径
- 控制无人机跟踪目标
- 计算攻击路径

**接收数据：**
- 来自DroneController的无人机状态信息
- 来自TargetTracker的目标位置信息
- 来自MissionExecutor的导航指令

**发送数据：**
- 向DroneController发送具体飞行控制命令
- 向MissionExecutor报告导航状态和完成情况

### 2.6 MissionExecutor (任务执行模块)

**提供服务：**
- 协调和管理整个系统的工作流程
- 根据当前状态决定执行哪种任务
- 管理状态转换（巡航->侦察->跟踪->攻击）

**接收数据：**
- 来自所有其他模块的状态报告
- 来自YoloDetector的目标统计信息
- 来自TargetTracker的目标跟踪状态
- 来自Navigation的导航状态
- 来自DroneController的无人机状态

**发送数据：**
- 向DroneController发送高级任务命令
- 向Navigation发送导航目标和策略
- 向TargetTracker发送目标选择标准
- 向YoloDetector发送检测参数

## 3. 数据流示例场景

### 场景1: 巡航和目标检测

1. **MissionExecutor** 启动巡航任务
2. **MissionExecutor** → **Navigation**: 发送巡航路径
3. **Navigation** → **DroneController**: 发送具体飞行命令
4. **DroneController** → **Tello**: 执行飞行动作
5. **VideoStream** ← **Tello**: 接收视频流
6. **VideoStream** → **YoloDetector**: 发送视频帧
7. **YoloDetector** 执行目标检测
8. **YoloDetector** → **MissionExecutor**: 报告检测结果
9. 如果检测到目标，**MissionExecutor** 切换到侦察状态

### 场景2: 目标跟踪和攻击

1. **YoloDetector** → **TargetTracker**: 提供检测结果
2. **TargetTracker** 选择目标并计算位置
3. **TargetTracker** → **Navigation**: 提供目标位置和方向
4. **Navigation** 计算跟踪路径
5. **Navigation** → **DroneController**: 发送飞行命令以跟踪目标
6. **DroneController** → **Tello**: 执行跟踪动作
7. **TargetTracker** → **MissionExecutor**: 报告目标足够近
8. **MissionExecutor** 决定开始攻击
9. **MissionExecutor** → **Navigation**: 发送攻击指令
10. **Navigation** → **DroneController**: 发送攻击路径飞行命令
11. **DroneController** → **Tello**: 改变LED颜色模拟攻击
12. **MissionExecutor** 记录攻击结果，切换回巡航状态

## 4. 核心状态转换

MissionExecutor控制的状态机转换流程：

```
       +------------+
       |    IDLE    |
       +------------+
              |
              v
       +------------+
 +---->|   PATROL   |<----+
 |     +------------+     |
 |            |           |
 |            v           |
 |     +------------+     |
 |     |    RECON   |     |
 |     +------------+     |
 |            |           |
 |            v           |
 |     +------------+     |
 |     |  TRACKING  |     |
 |     +------------+     |
 |            |           |
 |            v           |
 |     +------------+     |
 +-----|   ATTACK   |     |
       +------------+     |
              |           |
              v           |
       +------------+     |
       |   RETURN   |-----+
       +------------+
```

## 5. 代码集成关系

```python
# 从main.py中的初始化和集成顺序可以看出各模块之间的依赖关系
drone = DroneController()  # 首先初始化无人机控制器

video = VideoStream(drone).start()  # 视频流依赖于无人机控制器

detector = YoloDetector()  # 目标检测器相对独立

tracker = TargetTracker(frame_center)  # 目标跟踪器需要视频帧信息

navigation = Navigation(drone)  # 导航模块依赖于无人机控制器

# 任务执行器依赖所有其他模块
mission = MissionExecutor(drone, video, detector, tracker, navigation)  
```

通过这种模块化设计，系统各部分职责明确，可以独立开发和测试，同时通过定义良好的接口进行协作，构成完整的自杀式无人机系统。这种架构也便于后续扩展新功能或替换特定模块的实现。                                                                                                                                                   
