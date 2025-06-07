import time
import numpy as np
class PIDController:
    """简单的PID控制器实现"""
    def __init__(self, kp=0.0, ki=0.0, kd=0.0, setpoint=0.0, output_limits=None):
        self.kp = kp  # 比例增益
        self.ki = ki  # 积分增益
        self.kd = kd  # 微分增益
        self.setpoint = setpoint  # 目标值
        self.output_limits = output_limits  # 输出限制 (min, max)
        
        # 内部状态
        self.last_error = 0.0  # 上一次误差
        self.integral = 0.0    # 积分项
        self.last_time = time.time()  # 上一次更新时间
        
    def reset(self):
        """重置控制器状态"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def compute(self, process_value):
        """计算PID输出值"""
        # 计算时间差
        now = time.time()
        dt = now - self.last_time
        
        # 防止除零错误
        if dt <= 0:
            dt = 0.1
            
        # 计算误差
        error = self.setpoint - process_value
        
        # 比例项
        p_term = self.kp * error
        
        # 积分项
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # 微分项
        d_term = 0.0
        if dt > 0:
            d_term = self.kd * (error - self.last_error) / dt
            
        # 计算输出
        output = p_term + i_term + d_term
        
        # 应用输出限制
        if self.output_limits:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
            
        # 更新状态
        self.last_error = error
        self.last_time = now
        
        return output