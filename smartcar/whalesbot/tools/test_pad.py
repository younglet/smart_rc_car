#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""遥控器按键测试 —— 按下任意键查看对应的 bit 位"""

import sys
sys.path.insert(0, '.')
from smartcar.whalesbot.vehicle import BluetoothPad


def main():
    pad = BluetoothPad()
    prev = 0
    print("=" * 60)
    print("遥控器按键测试")
    print("按下按键 → 查看 btn 值和触发的 bit 位")
    print("按 Ctrl+C 退出")
    print("=" * 60)

    try:
        while True:
            keys = pad.read()
            btn = keys[4]
            if btn == 0:
                prev = 0
                continue

            # 仅变化时打印
            if btn != prev:
                bits = [i for i in range(16) if btn & (1 << i)]
                print(f"btn = {btn:5d}  (0b{btn:016b})  触发位: {bits}")

            prev = btn
    except KeyboardInterrupt:
        print("\n退出。")


if __name__ == "__main__":
    main()
