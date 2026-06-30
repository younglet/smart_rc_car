#!/usr/bin/python3
# -*- coding: utf-8 -*-
# rc_car.py - smart_rc_car 主控入口（上位机侧 Python 业务逻辑）
#
# 硬件:
#   - 上位机 (Host)       : NVIDIA Jetson Orin Nano
#   - 下位机 (Controller) : 鲸鱼 MC602 主控
# 本进程跑在 Jetson Orin Nano 上，通过串口协议向 MC602 下发控制指令。
#
#       .╭────────────────────────╮                            ╭────────────────────────╮.
#       .│         10 L           │                            │         11 R           │.
#       │    🚗 自动巡航           │                            │    🎯 X方向对齐         │
#       ╭╰════════════════════════╯════════════════════════════╰════════════════════════╯╮
#       │                                                                              │
#       │    左摇杆                                      右摇杆                         │
#       │    X: 横移                                      X: 旋转                       │
#       │    Y: 前进/后退                                                               │
#       │                                                                              │
#       │  ╭──────╮                                                          ╭──────╮   │
#       │  │  14  │                                                          │  15  │   │
#       │  │wheel-1│                                                         │wheel+1│   │
#       │  ╰──────╯                                                          ╰──────╯   │
#       │                                                                              │
#       │  ╭─── 十字键 ──────────────────────────────────────── 功能键 ──────────────╮   │
#       │  │    ┌─────────────────────┐        ┌─────────────────────┐               │   │
#       │  │    │    0  ↑ 手爪UP     │        │    4  △ 机械臂Y↑   │               │   │
#       │  │    │                    │        │                    │               │   │
#       │  │    │ 1  ←  8  ○  →  3  │        │ 7  ◁  9  □  ▷  5  │               │   │
#       │  │    │ 手臂L ⚡发射 手臂R │        │ 机械臂X← 抓手  机械臂X→│               │   │
#       │  │    │                    │        │                    │               │   │
#       │  │    │    2  ↓ 手爪DOWN  │        │    6  ▽ 机械臂Y↓   │               │   │
#       │  │    └─────────────────────┘        └─────────────────────┘               │   │
#       │  ╰────────────────────────────────────────────────────────────────────────╯   │
#       │                                                                              │
#       │                          L + R 同时按 = 退出                                  │
#       ╰──────────────────────────────────────────────────────────────────────────────╯

import sys
import time
import yaml

sys.path.insert(0, '.')

from smartcar.infer_service import LaneInferService, OCRInferService, TaskInferService
from smartcar import PID
from smartcar.whalesbot.tools import get_lan_ip, get_wifi_ssid
from smartcar.whalesbot.vehicle import *
from smartcar.whalesbot.vehicle.rc_arm import RCArm
from smartcar.whalesbot.vehicle.rc_shooter import Shooter
from smartcar.whalesbot.vehicle.rc_pid import PidCal2


class RCCar:
    """遥控车控制类"""

    @staticmethod
    def deadzone(value, center=0, threshold=0.0):
        """摇杆死区：center±threshold 内输出 0，超出部分线性重映射（threshold=0 则关闭）"""
        norm = value - center
        if abs(norm) <= threshold:
            return 0.0
        sign = 1 if norm > 0 else -1
        return (abs(norm) - threshold) * sign

    def __init__(self):
        # ==================== 推理服务 ====================
        self.lane_infer_service = LaneInferService()
        self.ocr_infer_service = OCRInferService()
        self.task_infer_service = TaskInferService()

        # ==================== 车辆基础组件 ====================
        with open("rc_cfg.yaml", "r") as f:
            cfg = yaml.load(f, Loader=yaml.FullLoader)

        self.car = MecanumDriver()
        self.arm = RCArm(
            vertical_motor    = cfg["vertical"]["motor"],
            vertical_limit_port = cfg["vertical"]["limit_port"],
            horizontal_motor  = cfg["horizontal"]["motor"],
            arm_servo_port    = cfg["arm"]["port"],
            arm_angle_map     = cfg["arm"]["angle_list"],
            hand_servo_port   = cfg["hand"]["port"],
            hand_servo_mode   = cfg["hand"]["mode"],
            hand_angle_map    = cfg["hand"]["angle_list"],
            grasp_pump_port   = cfg["grasp"]["port_pump"],
            grasp_valve_port  = cfg["grasp"]["port_valve"],
        )
        shooter_cfg = cfg["shooter"]
        self.shooter = Shooter(
            limit_sensor_port = shooter_cfg["limit_sensor"],
            motor_port        = shooter_cfg["motor"],
            magnet_port       = shooter_cfg["magnet"],
            stepper_id        = shooter_cfg["stepper"]["id"],
            stepper_reverse   = shooter_cfg["stepper"]["reverse"],
            power             = shooter_cfg["power"],
        )
        self.wheel = ServoBus(cfg["wheel"]["port"])
        self.wheel_angle = 0
        self.wheel_angle_range = cfg["wheel"]["angle_range"]
        self.wheel_step = cfg["wheel"]["step"]
        self.rings = Beep()
        self.display = ScreenShow()
        self.blue_pad = BluetoothPad()

        self.lane_pid = PidCal2(cfg["auto_lane"]["pid_y"], cfg["auto_lane"]["pid_angle"])
        self.lane_forward_speed = cfg["auto_lane"]["forward_speed"]

        # X 方向对齐 PID
        self.x_align_pid = PID(**cfg["visual_align"]["pid"])

        # ==================== 控制参数 ====================
        dz = cfg["chassis"]["deadzone"]
        self.dz_center = dz["center"]
        self.dz_threshold = dz["threshold"]
        self.state_start = cfg["chassis"]["state_start"]
        self.car_state = [0.0, 0.0, 0.0]

        # ==================== 按键沿检测 ====================
        self.prev_btn = 0
        self._y_limit_triggered = False  # Y轴限位跳变检测
        self._arm_side = "RIGHT"          # 手臂当前朝向

        # ==================== 退出标志 ====================
        self.exit_flag = False

        # ==================== 启动 ====================
        self.display.show(
            "   smart_rc_car\n"
            "==================\n"
            " Host: Jetson Orin\n"
            " Ctrl: MC602\n"
            "   L+R to Exit"
        )
        # 手臂初始化到右侧
        self.arm.arm_swing("RIGHT")
        self._arm_side = "RIGHT"

        self.beep()
        print("[smart_rc_car] 遥控车系统启动!! 上位机=Jetson Orin Nano, 下位机=MC602")
        time.sleep(3)
        self.display.show(
            "L:Lane   |R:Align\n"
            "</>:Arm  |</> X\n"
            "^/v:Hand |^/v Y\n"
            "O:Shoot  |[]=Pump\n"
            "1/2:Whl  |L+R:Exit"
        )
        self.run()

    def alert(self, msg):
        """蜂鸣 + 屏幕提示"""
        self.rings.rings()
        self.display.show(msg)

    def beep(self):
        self.rings.rings()

    def lane_infer(self, port="2.1"):
        return self.lane_infer_service.infer(port)

    def ocr_infer(self, port="2.2"):
        return self.ocr_infer_service.infer(port)

    def task_infer(self, port="2.2"):
        return self.task_infer_service.infer(port)

    # ==================== 主循环 ====================
    def run(self):
        grasp_flag = False
        self.arm.grasp(grasp_flag)

        while not self.exit_flag:
            keys_val = self.blue_pad.read()

            # --- 蓝牙手柄连接检测 ---
            if keys_val == [-1, -1, -1, -1, 0]:
                self.car_state = [0.0, 0.0, 0.0]
                print("[WARN] 未检测到蓝牙手柄")
                self.beep()
                time.sleep(1)
                continue

            btn = keys_val[4]
            prev = self.prev_btn
            self.prev_btn = btn

            # ==================== 组合键优先检测 ====================
            # L1(10) + R1(11): 退出
            if btn == ((1 << 10) | (1 << 11)):
                self.close()
                break

            # L(10): 自动巡航
            if btn == (1 << 10):
                if not (prev & (1 << 10)):
                    print("[L] 自动巡航")
                res = self.lane_infer("2.1")
                if res:
                    error_y, error_angle = res
                    y_speed, angle_speed = self.lane_pid.get_out(-error_y, -error_angle)
                    self.car.set_velocity(self.lane_forward_speed, y_speed, angle_speed)
                else:
                    self.alert("lane infer err")
                continue

            # R(11): X 方向对齐
            if btn == (1 << 11):
                if not (prev & (1 << 11)):
                    print("[R] X方向对齐")
                dets = self.task_infer("2.2")
                if dets and len(dets) > 0:
                    bbox = dets[0]["bbox"]
                    cx = (bbox[0] + bbox[2]) / 2
                    dx = (cx / 320) - 1
                    # LEFT / RIGHT 相机方向不同，PID 符号需区分
                    if self._arm_side == "LEFT":
                        out_x = -self.x_align_pid(dx)
                    else:
                        out_x = self.x_align_pid(dx)
                    self.car.set_velocity(out_x, 0, 0)
                else:
                    self.alert("no detection")
                continue

            # ○ (8): 发射（按下沿）
            if (btn & (1 << 8)) and not (prev & (1 << 8)):
                print("[○] 发射")
                self.shooter.reset()
                time.sleep(2)
                self.shooter.charge()
                time.sleep(2)
                self.shooter.move_to_next()
                time.sleep(2)
                self.shooter.shoot()

            # 14 + 15 同时按: 显示监控地址（隐藏特性）
            if (btn & (1 << 14)) and (btn & (1 << 15)):
                if not ((prev & (1 << 14)) and (prev & (1 << 15))):
                    print("[14+15] 显示监控地址")
                    ip = get_lan_ip()
                    ssid = get_wifi_ssid()
                    self.display.show(
                        "   Monitor URL\n"
                        f"enter wifi: <{ssid}>, then open :\n"
                        "https://"
                        f"{ip}"
                        ":8808/monitor"
                    )
                continue

            # 14: wheel - | 15: wheel +（按住连续转，松开停）
            elif btn & (1 << 14):
                if not (prev & (1 << 14)):
                    print(f"[14] wheel -")
                lo = self.wheel_angle_range[0]
                if self.wheel_angle > lo:
                    self.wheel_angle -= self.wheel_step
                    if self.wheel_angle <= lo:
                        self.wheel_angle = lo
                        print(f"[!] wheel 已到{lo}°限位")
                        self.alert(f"wheel {lo}")
                elif not (prev & (1 << 14)):
                    print(f"[!] wheel 已到{lo}°限位")
                    self.alert(f"wheel {lo}")
                self.wheel.set_angle(self.wheel_angle)
            elif btn & (1 << 15):
                if not (prev & (1 << 15)):
                    print(f"[15] wheel +")
                hi = self.wheel_angle_range[1]
                if self.wheel_angle < hi:
                    self.wheel_angle += self.wheel_step
                    if self.wheel_angle >= hi:
                        self.wheel_angle = hi
                        print(f"[!] wheel 已到{hi}°限位")
                        self.alert(f"wheel {hi}")
                elif not (prev & (1 << 15)):
                    print(f"[!] wheel 已到{hi}°限位")
                    self.alert(f"wheel {hi}")
                self.wheel.set_angle(self.wheel_angle)

            # ==================== 车辆运动控制（同原程序 + 死区） ====================
            lx = self.deadzone(keys_val[0], self.dz_center, self.dz_threshold)
            ly = self.deadzone(keys_val[1], self.dz_center, self.dz_threshold)
            rx = self.deadzone(keys_val[2], self.dz_center, self.dz_threshold)

            self.car_state[0] = self.state_start[0] * ly
            self.car_state[1] = -1 * self.state_start[1] * lx
            self.car_state[2] = -3.14 * self.state_start[2] * rx
            self.car.set_velocity(*self.car_state)

            # ==================== 机械臂 Y 轴 ====================
            if btn == (1 << 4):       # △
                if not (prev & (1 << 4)):
                    print("[△] 机械臂Y↑")
                self._y_limit_triggered = False
                self.arm.lift_up()
            elif btn == (1 << 6):     # ▽
                at_limit = self.arm.lift_at_limit()
                if at_limit:
                    if not self._y_limit_triggered:
                        self._y_limit_triggered = True
                        print("[!] Y轴已到下位限")
                        self.alert("Y lower limit")
                    elif not (prev & (1 << 6)):
                        print("[!] Y轴已到下位限")
                        self.alert("Y lower limit")
                    self.arm.lift_stop()
                else:
                    self._y_limit_triggered = False
                    if not (prev & (1 << 6)):
                        print("[▽] 机械臂Y↓")
                    self.arm.lift_down()
            else:
                self._y_limit_triggered = False
                self.arm.lift_stop()

            # ==================== 机械臂 X 轴 ====================
            if btn == (1 << 7):       # ◁
                if not (prev & (1 << 7)):
                    print("[◁] 机械臂X←")
                self.arm.stretch_out()
            elif btn == (1 << 5):     # ▷
                if not (prev & (1 << 5)):
                    print("[▷] 机械臂X→")
                self.arm.stretch_in()
            else:
                self.arm.stretch_stop()

            # ==================== 手臂方向 ====================
            if btn == (1 << 0):       # ↑
                if not (prev & (1 << 0)):
                    print("[↑] 手爪UP")
                self.arm.hand_tilt("UP")
            elif btn == (1 << 2):     # ↓
                if not (prev & (1 << 2)):
                    print("[↓] 手爪DOWN")
                self.arm.hand_tilt("DOWN")

            if btn == (1 << 1):       # ←
                if not (prev & (1 << 1)):
                    print("[←] 手臂LEFT")
                self._arm_side = "LEFT"
                self.arm.arm_swing("LEFT")
            elif btn == (1 << 3):     # →
                if not (prev & (1 << 3)):
                    print("[→] 手臂RIGHT")
                self._arm_side = "RIGHT"
                self.arm.arm_swing("RIGHT")

            # ==================== 抓手切换 ====================
            if btn == (1 << 9):       # □
                if not (prev & (1 << 9)):
                    print("[□] 抓手 切换")
                grasp_flag = not grasp_flag
                self.arm.grasp(grasp_flag)
                time.sleep(0.3)
            time.sleep(0.05)

    def close(self):
        """安全退出（同原程序）"""
        print("[EXIT] 正在关闭系统...")

        # 停止车辆与机械臂
        self.car.set_velocity(0.0, 0.0, 0.0)
        self.arm.lift_stop()
        self.arm.stretch_stop()

        # 屏幕显示
        self.display.show(
            "Program Exited\n"
            "================\n"
            "Restart Machine\n"
            "to Play Again"
        )

        # 蜂鸣器提示
        self.exit_flag = True
        for i in range(3):
            self.beep()
            time.sleep(0.4)

        print("[EXIT] 系统已安全关闭")


if __name__ == "__main__":
    rc = RCCar()
