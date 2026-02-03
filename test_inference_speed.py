#!/usr/bin/env python3
"""测试 OpenVINO 推理速度"""

import cv2
import time
from openvino.runtime import Core
import numpy as np

print("=" * 50)
print("OpenVINO 推理速度测试")
print("=" * 50)

# 加载模型
core = Core()
print(f"可用设备: {core.available_devices}")

model_path = '/home/user/droneAim/TDrone/armor_detector_ros2/models/0526.onnx'
print(f"加载模型: {model_path}")

model = core.read_model(model_path)
model.reshape([1, 3, 320, 320])
compiled_model = core.compile_model(model, "CPU")
# 测试不同配置
configs = [
    ("默认配置", {}),
    ("LATENCY模式", {"PERFORMANCE_HINT": "LATENCY"}),
    ("THROUGHPUT模式", {"PERFORMANCE_HINT": "THROUGHPUT"}),
    ("4线程", {"INFERENCE_NUM_THREADS": 4}),
    ("6线程", {"INFERENCE_NUM_THREADS": 6}),
    ("12线程", {"INFERENCE_NUM_THREADS": 12}),
]

# 创建假输入
dummy_input = np.random.rand(1, 3, 320, 320).astype(np.float32)
N = 100

for name, config in configs:
    try:
        compiled = core.compile_model(model, 'CPU', config)
        infer_request = compiled.create_infer_request()
        
        # 预热
        for _ in range(10):
            infer_request.infer({0: dummy_input})
        
        # 测试
        start = time.time()
        for _ in range(N):
            infer_request.infer({0: dummy_input})
        elapsed = time.time() - start
        
        print(f"{name}: {N/elapsed:.1f} FPS, {elapsed/N*1000:.2f}ms/帧")
    except Exception as e:
        print(f"{name}: 失败 - {e}")

print("=" * 50)
