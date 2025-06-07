from djitellopy import Tello
import time

# tello.send_expansion_command("led 255 0 0") 

def attack_sim(tello: Tello):
    """
    模拟攻击函数,点亮LED灯
    """
    tello.send_expansion_command("led 255 0 0")  # 红色LED灯
    print("LED灯已点亮,模拟攻击开始。")
    tello.send_expansion_command("mled l b 1 attack simulation")  # 模拟攻击命令

    # 模拟攻击持续时间
    time.sleep(5)
    
    # 攻击结束，关闭LED灯
    tello.send_expansion_command("led 0 0 0")  # 关闭LED灯
    tello.send_expansion_command("mled l b 1 attack completed")
    time.sleep(2)
    tello.send_expansion_command("mled l b 1 curise")  # 模拟攻击结束命令
    print("攻击结束,LED灯已关闭。")