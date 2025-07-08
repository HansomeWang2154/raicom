import torch

# 创建一个张量并尝试转移到 CPU
try:
    tensor = torch.tensor([1, 2, 3]).to('cpu')
    print("CPU 设备可用，张量已成功加载到 CPU:")
    print(f"张量设备: {tensor.device}")  # 输出: cpu
    print(f"张量值: {tensor}")
except Exception as e:
    print(f"CPU 设备不可用（理论上不会发生）: {e}")