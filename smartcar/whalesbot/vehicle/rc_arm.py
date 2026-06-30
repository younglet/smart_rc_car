#!/usr/bin/python3
# -*- coding: utf-8 -*-
# rc_arm.py - 精简机械臂控制（仅 smart_rc_car 需要的接口）
#
# 上位机 Host       : NVIDIA Jetson Orin Nano
# 下位机 Controller : 鲸鱼 MC602 主控（端口号即 MC602 上的资源编号）

from smartcar.whalesbot.vehicle.base.controller_wrap import (
    StepperWrap, MotorWrap, ServoBus, ServoPwm,
    AnalogInput, PoutD,
)


class RCArm:
    """smart_rc_car 专用机械臂控制，参数由外部传入"""

    def __init__(self,
                 vertical_motor,       # dict: StepperWrap 参数
                 vertical_limit_port,  # int:  磁敏限位 ADC 端口
                 horizontal_motor,     # dict: MotorWrap 参数
                 arm_servo_port,       # int:  手臂左右摆舵机端口
                 arm_angle_map,        # dict: {"LEFT": 93, "RIGHT": -93, ...}
                 hand_servo_port,      # int:  手爪舵机端口
                 hand_servo_mode,      # int:  舵机模式 (180)
                 hand_angle_map,       # dict: {"UP": -90, "DOWN": 0, ...}
                 grasp_pump_port,      # int:  气泵 IO 端口
                 grasp_valve_port,     # int:  电磁阀 IO 端口
                 ):

        # ---- 升降轴 Y ----
        self.motor_y = StepperWrap(**vertical_motor)
        self.y_limit_sensor = AnalogInput(vertical_limit_port)

        # ---- 伸缩轴 X ----
        self.motor_x = MotorWrap(**horizontal_motor)

        # ---- 手臂左右摆 ----
        self.arm_servo = ServoBus(arm_servo_port)
        self._arm_angle_map = arm_angle_map

        # ---- 手爪上下 ----
        self.hand_servo = ServoPwm(hand_servo_port, mode=hand_servo_mode)
        self._hand_angle_map = hand_angle_map

        # ---- 气泵抓取 ----
        self.pump = PoutD(grasp_pump_port)
        self.valve = PoutD(grasp_valve_port)

    # ==================== Y 轴升降 ====================

    def lift_up(self):
        """Y 轴上升"""
        self.motor_y.set_velocity(0.5)

    def lift_down(self):
        """Y 轴下降"""
        self.motor_y.set_velocity(-0.5)

    def lift_stop(self):
        """Y 轴停止"""
        self.motor_y.set_velocity(0.0)

    def lift_at_limit(self):
        """Y 轴是否到达下限位"""
        return self.y_limit_sensor.read() > 1000

    # ==================== X 轴伸缩 ====================

    def stretch_out(self):
        """X 轴伸出"""
        self.motor_x.set_angular(50)

    def stretch_in(self):
        """X 轴缩回"""
        self.motor_x.set_angular(-50)

    def stretch_stop(self):
        """X 轴停止"""
        self.motor_x.set_angular(0.0)

    # ==================== 手臂左右摆 ====================

    def arm_swing(self, direction: str):
        """手臂方向 LEFT / RIGHT"""
        angle = self._arm_angle_map[direction]
        self.arm_servo.set_angle(angle)

    # ==================== 手爪上下 ====================

    def hand_tilt(self, direction: str):
        """手爪方向 UP / DOWN"""
        angle = self._hand_angle_map[direction]
        self.hand_servo.set_angle(angle)

    # ==================== 抓取 ====================

    def grasp(self, on: bool):
        """
        气泵抓取 / 释放
        on=True   → 吸气抓取
        on=False  → 释放
        """
        self.pump.set(not on)
        self.valve.set(on)
