import time
import math
from djitellopy import Tello
import threading
import cv2
class PIDController:
    def __init__(self, kp, ki, kd, min_output, max_output):
        # PID参数
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # 输出限制
        self.min_output = min_output
        self.max_output = max_output
        
        # 内部状态
        self.last_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def update(self, error):
        current_time = time.time()
        dt = current_time - self.last_time
        
        # 防止dt为零导致计算错误
        if dt <= 0:
            return 0
            
        # 计算积分项和微分项
        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        
        # 计算PID输出
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # 限制输出范围
        output = max(self.min_output, min(self.max_output, output))
        
        # 更新状态
        self.last_error = error
        self.last_time = current_time
        
        return output

class TelloController(threading.Thread):
    def __init__(self, tello:Tello = None, target_queue=None, laser_queue=None):
        # 初始化Tello无人机
        super().__init__()
        self.tello = tello 
        # self.tello.connect()
        # self.tello.streamon()
        
        # 获取图像中心坐标(假设图像尺寸为960x720)
        self.image_width = 960
        self.image_height = 720
        self.image_center_x = self.image_width // 2
        self.image_center_y = self.image_height // 2
        self.target_queue = target_queue  
        # 目标距离(厘米)
        self.target_distance = 100
        
        # 初始化PID控制器
        # 水平方向(X轴)PID参数
        self.pid_x = PIDController(kp=0.5, ki=0.01, kd=0.1, min_output=-50, max_output=50)
        # 垂直方向(Y轴)PID参数
        self.pid_y = PIDController(kp=0.5, ki=0.01, kd=0.1, min_output=-50, max_output=50)
        # 前后方向(Z轴)PID参数
        self.pid_z = PIDController(kp=0.8, ki=0.01, kd=0.2, min_output=-50, max_output=50)
        # 偏航角(Yaw)PID参数
        self.pid_yaw = PIDController(kp=0.5, ki=0.01, kd=0.1, min_output=-50, max_output=50)
        
        # 控制器状态
        self.is_tracking = False
        # ------------------------------------
    
    def run(self):
        self.track_target()
        
    def get_target_position(self):
        """获取目标在图像中的位置(需要根据具体目标检测算法实现)"""
        # 这里需要实现目标检测算法，返回目标中心坐标和大小
        # 示例返回值：(center_x, center_y, width, height)
        # 目前使用模拟值
        self.target = self.target_queue.get() if not self.target_queue.empty() else None
        if self.target is None:
            # 如果没有目标，返回图像中心位置
            return (self.image_center_x, self.image_center_y, 100, 100)
        return (self.target[3][0], self.target[3][1],self.target[3][2] , self.target[3][3])
    
    def calculate_distance(self, width):
        """根据目标在图像中的宽度计算距离(简化模型)"""
        # 实际距离(厘米) = (目标实际宽度 * 相机焦距) / 目标在图像中的像素宽度
        # 假设目标实际宽度为30厘米，相机焦距为800像素
        return self.target[2]
    
    def calculate_yaw_error(self, target_x):
        """计算偏航角误差"""
        # 计算目标与图像中心的水平偏差(像素)
        error_x = target_x - self.image_center_x
        # 将像素误差转换为角度误差(简化模型)
        # 假设每10个像素对应1度的偏航角
        return error_x / 10
    
    def track_target(self):
        """跟踪目标并控制无人机移动"""
        try:
            # self.tello.takeoff()
            self.is_tracking = True
            
            while self.is_tracking:
                # 获取当前帧
                frame = self.tello.get_frame_read().frame
                
                # 获取目标位置
                target_x, target_y, target_width, target_height = self.get_target_position(frame)
                
                # 计算目标距离
                current_distance = self.calculate_distance(target_width)
                
                # 计算PID控制输出
                # X方向(左右)速度控制
                vel_x = self.pid_x.update(self.image_center_x - target_x)
                
                # Y方向(上下)速度控制
                vel_y = self.pid_y.update(self.image_center_y - target_y)
                
                # Z方向(前后)速度控制
                vel_z = self.pid_z.update(self.target_distance - current_distance)
                
                # 偏航角控制
                yaw = self.pid_yaw.update(self.calculate_yaw_error(target_x))
                
                # 发送控制命令
                # self.tello.send_rc_control(int(vel_z), int(vel_x), int(-vel_y), int(yaw))
                
                # 显示状态
                print(f"目标位置: ({target_x}, {target_y}), 距离: {current_distance:.1f}cm")
                print(f"控制命令: 前进={vel_z:.1f}, 左右={vel_x:.1f}, 上下={vel_y:.1f}, 偏航={yaw:.1f}")
                
                # 按q键退出跟踪
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # 短暂延迟
                time.sleep(0.05)
                
        except Exception as e:
            print(f"错误: {e}")
        finally:
            # 停止跟踪并降落
            self.is_tracking = False
            self.tello.send_rc_control(0, 0, 0, 0)
            self.tello.land()
            self.tello.streamoff()

if __name__ == "__main__":
    controller = TelloController()
    controller.track_target()    