"""
推理服务抽象基类及三个具体实现：
  - LaneInferService   ：巡线推理
  - OCRInferService    ：OCR 文字识别
  - TaskInferService   ：对象识别
"""

import abc
import requests
import urllib3

urllib3.disable_warnings()


class InferService(abc.ABC):
    """推理服务抽象基类"""

    def __init__(self, base_url: str = "https://127.0.0.1:8808"):
        self.base_url = base_url.rstrip("/")

    # ---------- 子类需要覆盖的钩子 ----------
    @property
    @abc.abstractmethod
    def endpoint(self) -> str:
        """推理模型端点，例如 /models/lane_model"""
        ...

    @abc.abstractmethod
    def parse_response(self, data: dict):
        """从响应 JSON 中提取所需字段"""
        ...

    # ---------- 对外接口 ----------
    def infer(self, port):
        """
        发起一次推理请求。
        :param port: 摄像头端口号
        :return: parse_response 的返回值
        """
        try:
            url = f"{self.base_url}{self.endpoint}?port={port}"
            resp = requests.get(url, verify=False)
            return self.parse_response(resp.json())
        except Exception:
            print(f"[{self.__class__.__name__}] 推理服务错误！")
            return None


# ==================== 具体实现 ====================

class LaneInferService(InferService):
    """巡线推理"""

    @property
    def endpoint(self) -> str:
        return "/models/lane_model"

    def parse_response(self, data: dict):
        output = data["output"]
        return output["error"], output["angle"]


class OCRInferService(InferService):
    """OCR 文字识别"""

    @property
    def endpoint(self) -> str:
        return "/models/ocr"

    def parse_response(self, data: dict):
        return data["texts"]


class TaskInferService(InferService):
    """对象识别（任务 2026）"""

    @property
    def endpoint(self) -> str:
        return "/models/task2026"

    def parse_response(self, data: dict):
        return data["detections"]


# ==================== 兼容旧接口 ====================

def lane_infer(port):
    """兼容 new_lane_infer.py 的旧调用方式"""
    return LaneInferService().infer(port)


def ocr_infer(port):
    """兼容 new_ocr_infer.py 的旧调用方式"""
    return OCRInferService().infer(port)


def task_infer(port):
    """兼容 new_task_infer.py 的旧调用方式"""
    return TaskInferService().infer(port)


# ==================== 测试 ====================

if __name__ == "__main__":
    import time

    N = 100                     # 测试轮数
    PORT = "2.1"               # 摄像头端口

    services = [
        ("LaneInferService",  LaneInferService()),
        ("OCRInferService",   OCRInferService()),
        ("TaskInferService",  TaskInferService()),
    ]

    for name, svc in services:
        print(f"\n=== {name}  ({N} 轮) ===")
        latencies = []
        t0 = time.perf_counter()

        for i in range(N):
            t1 = time.perf_counter()
            result = svc.infer(PORT)
            t2 = time.perf_counter()
            lat = (t2 - t1) * 1000           # ms
            latencies.append(lat)
            # print(f"  [{i+1:3d}] {lat:7.2f}ms  |  {result}")

        t_total = time.perf_counter() - t0

        # ---- 统计 ----
        latencies.sort()
        avg = sum(latencies) / len(latencies)
        fps = len(latencies) / t_total

        print(f"\n  {'=' * 45}")
        print(f"  总耗时     : {t_total:.3f}s")
        print(f"  平均延迟   : {avg:.2f}ms")
        print(f"  最小延迟   : {latencies[0]:.2f}ms")
        print(f"  最大延迟   : {latencies[-1]:.2f}ms")
        print(f"  中位数延迟 : {latencies[len(latencies)//2]:.2f}ms")
        print(f"  FPS        : {fps:.2f}")
        print(f"  {'=' * 45}")
