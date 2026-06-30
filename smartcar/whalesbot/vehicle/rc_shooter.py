#!/usr/bin/python3
# -*- coding: utf-8 -*-
# rc_shooter.py - 发射器控制

import time
import math
from smartcar.whalesbot.vehicle.base.controller_wrap import (
    AnalogInput, MotorWrap, PoutD, StepperWrap,
)


class Shooter:
    """发射器控制，参数由外部传入"""

    def __init__(self,
                 limit_sensor_port,  # int: 复位限位 ADC 端口
                 motor_port,         # int: 摩擦轮电机端口
                 magnet_port,        # int: 电磁铁 IO 端口
                 stepper_id,         # int: 弹仓步进电机 id
                 stepper_reverse,    # int: 步进反向
                 power=20,           # int: 蓄力强度
                 ):
        self.limit_sensor = AnalogInput(limit_sensor_port)
        self.motor = MotorWrap(motor_port)
        self.magnet = PoutD(magnet_port)
        self.stepper = StepperWrap(stepper_id, stepper_reverse)
        self.power = power

    def reset(self):
        self.magnet.set(False)
        self.motor.set_linear(4)
        while not self.limit_sensor.read() > 1000:
            pass
        self.motor.set_linear(0)
        self.motor.set_angular(-30)
        time.sleep(2)
        print("shooter reseted!!")

    def charge(self):
        self.magnet.set(True)
        self.motor.set_angular(self.power)
        print("shooter charged!!")

    def shoot(self):
        self.magnet.set(False)
        self.motor.set_angular(-28)
        print("shoot!!")

    def move_to_next(self):
        start_rad = self.stepper.get_rad()
        self.stepper.set_angular(0.4)
        while self.stepper.get_rad() - start_rad < math.pi * 2 / 5:
            pass
        self.stepper.set_angular(0)
