#!/usr/bin/python3
# -*- coding: utf-8 -*-
# rc_pid.py - 自定义 PID 封装

from smartcar import PID


class PidCal2:
    """双通道 PID（Y 轴 + 角度），用于巡线 / 视觉对齐"""

    def __init__(self, cfg_pid_y, cfg_pid_angle):
        self.pid_y = PID(**cfg_pid_y)
        self.pid_angle = PID(**cfg_pid_angle)

    def get_out(self, error_y, error_angle):
        pid_y_out = self.pid_y(error_y)
        pid_angle_out = self.pid_angle(error_angle)
        return pid_y_out, pid_angle_out
