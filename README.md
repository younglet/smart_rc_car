# smart_rc_car — 智能遥控车

基于蓝牙手柄的智能遥控车项目：底盘运动 + 机械臂控制 + 发射器 + AI 巡线 / 视觉对齐。

---

## 硬件架构

本项目采用 **上位机 + 下位机** 的双层控制架构。

```
┌──────────────────────────┐        USB / UART         ┌──────────────────────────┐
│   上位机  Host           │  ◀────────────────────▶   │   下位机  Controller     │
│   Jetson Orin Nano       │     控制指令 / 状态       │   鲸鱼 MC602 主控        │
│                          │                           │                          │
│  · 视觉推理 (车道 / OCR) │                           │  · 实时电机控制           │
│  · PID 闭环计算          │                           │  · 舵机 / 气泵 / 电磁阀   │
│  · 蓝牙手柄解析          │                           │  · ADC 限位 / 蜂鸣 / 屏显 │
│  · Python 主控逻辑       │                           │  · 串口协议层             │
└──────────────────────────┘                           └──────────────────────────┘
```

| 角色 | 型号 | 主要职责 |
|------|------|----------|
| **上位机** | **NVIDIA Jetson Orin Nano** | 运行 Python 主控、加载 AI 模型、解析蓝牙手柄、计算 PID、向下位机下发运动/机械臂指令 |
| **下位机** | **鲸鱼 MC602 主控** | 实时驱动四轮底盘 / 步进电机 / 舵机 / 气泵 / 电磁阀，采集 ADC 限位，按串口协议向上位机回报状态 |

上位机 = Jetson Orin Nano；下位机 = 鲸鱼 MC602 主控板。两者通过串口协议通信（见 `smartcar/whalesbot/vehicle/base/` 下的串口封装）。

---

## 依赖：jetson-paddle-infer-server

本项目的 **AI 推理** 部分依赖一个独立的推理后端进程：

> 🔗 **https://github.com/younglet/jetson-paddle-infer-server**

该仓库基于 PaddlePaddle 在 Jetson Orin Nano 上提供模型加载、视频源接入和 HTTP 推理接口。`smart_rc_car` **不内置模型**，运行时只通过 HTTP 与该服务对话——这样模型升级、热重载、TensorRT 调优都与控制逻辑解耦。

### 通信原理

```
┌────────────────────────────────┐    HTTP / HTTPS    ┌────────────────────────────────────┐
│  smart_rc_car  (Jetson)        │ ◀────────────────▶ │  jetson-paddle-infer-server (同机)  │
│                                │   loopback 8808    │                                    │
│  · InferService.infer(port)    │                    │  · 加载 lane_model / ocr / task2026│
│  · requests.get(url, verify=F) │   GET /models/...  │  · 拉取摄像头指定 port 的帧          │
│  · parse_response(json)        │                    │  · 调用 Paddle 模型推理             │
│  · 传给 PID / 控制 MC602        │                    │  · 返回 JSON 结构化结果             │
└────────────────────────────────┘                    └────────────────────────────────────┘
```

- 协议：`HTTPS`（自签证书，所以客户端用 `verify=False`；见 `smartcar/infer_service.py`）
- 地址：默认 `https://127.0.0.1:8808`（同机 loopback，跨机可改 `InferService(base_url=...)`）
- 调用方式：`requests.get(f"{base_url}{endpoint}?port={port}")`
- 超时/异常：连接失败时 `infer()` 返回 `None`，主循环走「提示 + 蜂鸣」分支（不会让小车失能）

### 对齐的端点

| 本项目调用方 | 端点 | 摄像头端口 | 响应 JSON 字段 | 用途 |
|-------------|------|-----------|----------------|------|
| `LaneInferService` | `GET /models/lane_model?port=2.1` | `2.1` | `output.error`、`output.angle` | **L 键** 自动巡航：返回 `(error_y, error_angle)` 给 `PidCal2` |
| `OCRInferService`  | `GET /models/ocr?port=...`        | 外部传入 | `texts` | 文字识别（备用，键盘未绑定） |
| `TaskInferService` | `GET /models/task2026?port=2.2`  | `2.2` | `detections[].bbox` | **R 键** X方向对齐：取首个检测框中心 → 归一化 dx → `PID` |

端点路径、参数与返回结构必须**与 `jetson-paddle-infer-server` 的 API 保持一致**，否则 `parse_response()` 会抛 `KeyError` 并被 `try/except` 吞掉返回 `None`。

### 启动顺序（对齐依赖）

`install_service.sh` 通过 systemd 显式表达了这个依赖：

```ini
[Unit]
After=network.target jetson-infer-server.service
Requires=jetson-infer-server.service

[Service]
ExecStartPre=/usr/bin/bash -c '
    for i in $(seq 1 30); do
        curl -sk --connect-timeout 2 https://127.0.0.1:8808 > /dev/null 2>&1 && break;
        sleep 2;
    done'
ExecStart=/usr/bin/python3 ${PROJECT_DIR}/rc_car.py
```

含义：

1. `Requires=` — systemd 启动 `rc-car.service` 之前必须先拉起 `jetson-infer-server.service`，否则本服务直接失败
2. `ExecStartPre=` — 即使服务已 `active`，也再轮询 `https://127.0.0.1:8808` 直到能连通（最多 60s），避免模型还没加载完就被 `rc_car.py` 调出空响应
3. 仅在轮询通过后才真正启动主循环

### 本地快速验证

未启用 systemd 时也可手动确认链路通畅：

```bash
# 1. 确认推理服务在线
curl -sk https://127.0.0.1:8808

# 2. 单次推理自检
curl -sk "https://127.0.0.1:8808/models/lane_model?port=2.1"
curl -sk "https://127.0.0.1:8808/models/task2026?port=2.2"

# 3. 本项目自带压测脚本（端到端延迟 / FPS）
python3 smartcar/infer_service.py
```

---

## 项目结构

```
smart_rc_car/
├── rc_car.py             # 主控入口（上位机侧 Python 业务逻辑）
├── rc_cfg.yaml           # 全局硬件 + PID 配置
├── install_service.sh    # 注册为 systemd 服务（开机自启）
├── smartcar/
│   ├── __init__.py
│   ├── infer_service.py  # 推理服务（巡线 / OCR / 目标检测）
│   └── whalesbot/
│       ├── tools/        # 日志 / PID / YAML / 网络工具
│       └── vehicle/
│           ├── rc_arm.py      # 精简机械臂控制
│           ├── rc_shooter.py  # 发射器控制
│           ├── rc_pid.py      # 双通道 PID 封装
│           ├── driver/        # 底盘 / 电机驱动
│           └── base/          # MC602 串口协议封装（下位机通信）
└── README.md
```

---

## 手柄布局

```
      .╭────────────────────────╮                            ╭────────────────────────╮.
      .│         10 L           │                            │         11 R           │.
      │    🚗 自动巡航           │                            │    🎯 X方向对齐         │
      ╭╰════════════════════════╯════════════════════════════╰════════════════════════╯╮
      │                                                                              │
      │    左摇杆                                      右摇杆                         │
      │    X: 横移                                      X: 旋转                       │
      │    Y: 前进/后退                                                               │
      │                                                                              │
      │  ╭──────╮                                                          ╭──────╮   │
      │  │  14  │                                                          │  15  │   │
      │  │wheel-3│                                                         │wheel+3│   │
      │  ╰──────╯                                                          ╰──────╯   │
      │                                                                              │
      │  ╭─── 十字键 ──────────────────────────────────────── 功能键 ──────────────╮   │
      │  │    ┌─────────────────────┐        ┌─────────────────────┐               │   │
      │  │    │    0  ↑ 手爪UP     │        │    4  △ 机械臂Y↑   │               │   │
      │  │    │                    │        │                    │               │   │
      │  │    │ 1  ←  8  ○  →  3  │        │ 7  ◁  9  □  ▷  5  │                │   │
      │  │    │ 手臂L ⚡发射 手臂R │        │ 机械臂X← 抓手  机械臂X→│              │   │
      │  │    │                    │        │                    │               │   │
      │  │    │    2  ↓ 手爪DOWN  │        │    6  ▽ 机械臂Y↓   │               │   │
      │  │    └─────────────────────┘        └─────────────────────┘               │   │
      │  ╰────────────────────────────────────────────────────────────────────────╯   │
      │                                                                              │
      │                          L + R 同时按 = 退出                                  │
      ╰──────────────────────────────────────────────────────────────────────────────╯
```

---

## 按键功能一览

| 按键 | 符号 | 功能 | 触发方式 |
|------|------|------|----------|
| L | — | 🚗 自动巡航 | 按住循环 |
| R | — | 🎯 X方向对齐 | 按住循环 |
| L+R | — | 退出程序 | 按下 |
| 14 | — | wheel -3° | 按住连续转，松开停 |
| 15 | — | wheel +3° | 按住连续转，松开停 |
| ↑ | ↑ | 手爪 UP | 按住 |
| ↓ | ↓ | 手爪 DOWN | 按住 |
| ← | ← | 手臂 LEFT | 按住 |
| → | → | 手臂 RIGHT | 按住 |
| △ | △ | 机械臂 Y 上升 | 按住 |
| ▽ | ▽ | 机械臂 Y 下降 | 按住（有限位保护） |
| ◁ | ◁ | 机械臂 X 伸出 | 按住 |
| ▷ | ▷ | 机械臂 X 缩回 | 按住 |
| ○ | ○ | ⚡ 发射 | 按下沿 |
| □ | □ | 抓手 开/关 | 按下沿 |

### 摇杆

| 摇杆 | 轴 | 功能 | 系数 | 死区 |
|------|----|------|------|------|
| 左 | X | 横向移动 | × 0.3 | 0.1 |
| 左 | Y | 前进/后退 | × 0.3 | 0.1 |
| 右 | X | 原地旋转 | × 0.5 × 3.14 | 0.1 |

---

## 硬件接线（以 MC602 端口号为准）

### 底盘

| 组件 | 接口 | 端口/ID |
|------|------|---------|
| Mecanum 四轮驱动 | MecanumDriver | 1-4 |

### 机械臂

| 组件 | 接口 | 端口/ID | 说明 |
|------|------|---------|------|
| Y 轴升降 | StepperWrap | id=3 | 丝杆导程 0.008m |
| Y 轴下限位 | AnalogInput | **5** | 磁敏，>1000 触发，限制**向下** |
| X 轴伸缩 | MotorWrap | id=6, motor_280 | 平移 |
| 手臂左右摆 | ServoBus | **1** | LEFT 93° / RIGHT -93° |
| 手爪上下 | ServoPwm | **3**, 180°模式 | UP -90° / DOWN 0° |
| 气泵 | PoutD | **2** | 吸气 |
| 电磁阀 | PoutD | **3** | 放气 |

### 发射器

| 组件 | 接口 | 端口/ID | 说明 |
|------|------|---------|------|
| 摩擦轮电机 | MotorWrap | **5** | charge 18, shoot -28 |
| 电磁铁 | PoutD | **4** | 吸合/释放 |
| 复位限位 | AnalogInput | **6** | 磁敏，>1000 |
| 弹仓步进 | StepperWrap | id=1, reverse=-1 | 72° 换弹 |

### 其他

| 组件 | 接口 | 端口/ID | 说明 |
|------|------|---------|------|
| 转向轮 | ServoBus | **2** | ±180°, 按住连续 ±3° |
| 蜂鸣器 | Beep | 内置 | |
| 屏幕 | ScreenShow | 内置 | |
| 蓝牙手柄 | BluetoothPad | 内置（上位机） | |

---

## AI 功能

> AI 推理调用的是 **[jetson-paddle-infer-server](https://github.com/younglet/jetson-paddle-infer-server)**，本项目只是客户端。端点协议、JSON 结构、调用方式见上一节「依赖：jetson-paddle-infer-server」。

### L — 自动巡航（车道保持）

- **模型**: `/models/lane_model`（摄像头 2.1，调用见 `LaneInferService`）
- **输出**: `(error_y, error_angle)`
- **PID** (从 `rc_cfg.yaml` 读取):

| 控制器 | Kp | Ki | Kd | output_limits |
|--------|----|----|----|---------------|
| pid_y | 5 | 0.1 | 0 | [-0.7, 0.7] |
| pid_angle | 3 | 0 | 0 | [-1.5, 1.5] |

- **控制**: 固定前进速度 0.2 m/s + PID 修正

### R — X 方向对齐（目标对中）

- **模型**: `/models/task2026`（摄像头 2.2，调用见 `TaskInferService`）
- **输出**: bbox → 归一化 dx → PID → 横向移动
- **PID**: Kp=1, Ki=0, Kd=0, output_limits=[-0.7, 0.7]

---

## 安全保护

| 保护项 | 条件 | 行为 |
|--------|------|------|
| Y 轴下限位 | AnalogInput(5) > 1000 | 下降停转 + 打印 + 蜂鸣 |
| wheel ±180° 限位 | angle ≤ -180 或 ≥ 180 | 停转 + 打印 + 蜂鸣 |
| 蓝牙断开 | keys_val == [-1,-1,-1,-1,0] | 停车 + 提示 + 蜂鸣 |

---

## 配置

所有参数集中在 `rc_cfg.yaml`：

```yaml
# 自动巡航
auto_lane:
  forward_speed: 0.2
  pid_y: { Kp: 5, Ki: 0.1, Kd: 0, setpoint: 0, output_limits: [-0.7, 0.7] }
  pid_angle: { Kp: 3, Ki: 0, Kd: 0, setpoint: 0, output_limits: [-1.5, 1.5] }

# 视觉对齐
visual_align:
  pid: { Kp: 1, Ki: 0, Kd: 0, setpoint: 0, output_limits: [-0.7, 0.7] }

# 升降轴 Y
vertical:
  motor: { id: 3, reverse: 1, perimeter: 0.008 }
  limit_port: 5

# 伸缩轴 X
horizontal:
  motor: { id: 6, reverse: 1, type: motor_280, perimeter: 0.032 }

# ... arm / hand / grasp / chassis / wheel / shooter
```

---

## 运行

> 本项目运行在 **Jetson Orin Nano**（上位机），通过串口下发指令给 **鲸鱼 MC602**（下位机）执行。

### 手动运行

```bash
cd ~/smart_rc_car
python3 rc_car.py
```

- 启动蜂鸣一声，抓手初始闭合
- L + R 同时按退出，停车 → 屏显 → 蜂鸣 × 3

### 注册为 systemd 服务（开机自启）

```bash
cd ~/smart_rc_car
./install_service.sh --install
```

服务名 `rc-car.service`，会等待本机的推理服务 `https://127.0.0.1:8808` 就绪后再拉起 `rc_car.py`。

```bash
systemctl status rc-car     # 查看状态
./install_service.sh --uninstall   # 卸载
```