import cv2
import numpy as np
import requests
import onnxruntime as ort
import time
from io import BytesIO
import mmap
import struct
import os

# 定义视频流大小和通道
WIDTH = 320
HEIGHT = 240
CHANNELS = 3

# 加载预训练的模型
model_path = '/home/code/main/src/service/python/face/models/version-slim-320.onnx'
session = ort.InferenceSession(model_path, providers=['CPUExecutionProvider'])
input_name = session.get_inputs()[0].name
output_names = [session.get_outputs()[0].name, session.get_outputs()[1].name]

def postprocess(scores, boxes, threshold=0.8):
    scores = scores[0][:, 1]
    mask = scores > threshold
    filtered_scores = scores[mask]
    filtered_boxes = boxes[0][mask]

    if len(filtered_scores) == 0:
        return None

    weights = filtered_scores / filtered_scores.sum()
    weighted_boxes = filtered_boxes * weights[:, np.newaxis]
    avg_box = weighted_boxes.sum(axis=0)

    return tuple(avg_box)

def main():
    # 创建或打开共享内存
    shm_fd = os.open('/dev/shm/best_box', os.O_CREAT | os.O_RDWR)
    assert shm_fd != -1
    # 分配足够存储四个浮点数的内存空间
    os.ftruncate(shm_fd, 32)
    mm = mmap.mmap(shm_fd, 32, mmap.MAP_SHARED, mmap.PROT_WRITE)

    snapshot_url = 'http://192.168.179.146:8080/?action=snapshot'
    
    try:
        while True:
            start_time = time.time()

            # 使用requests直接获取最新的单帧图像
            response = requests.get(snapshot_url)
            image_bytes = BytesIO(response.content)
            frame = cv2.imdecode(np.frombuffer(image_bytes.read(), np.uint8), cv2.IMREAD_COLOR)

            if frame is None:
                print("Error fetching the latest frame")
                continue

            # 缩放图像
            frame_resized = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_NEAREST)

            # 预处理图像
            img_processed = np.transpose(frame_resized, (2, 0, 1)).astype(np.float32)
            img_processed = (img_processed - 127.0) / 128.0
            input_blob = np.expand_dims(img_processed, axis=0)

            # 推理
            scores, boxes = session.run(output_names, {input_name: input_blob})

            # 后处理
            best_box = postprocess(scores, boxes)

            if best_box:
                # 将结果写入共享内存
                mm.seek(0)
                mm.write(struct.pack('4f', *best_box))
            else:
                # 若无有效结果，则写入无效值
                mm.seek(0)
                mm.write(struct.pack('4f', -1.0, -1.0, -1.0, -1.0))

            end_time = time.time()
            process_duration = (end_time - start_time) * 1000
            sleep_time = max(0, 200 - process_duration)
            time.sleep(sleep_time / 1000)
    finally:
        # 清理资源
        mm.close()
        os.close(shm_fd)
        os.unlink('/tmp/best_box')
        print("Releasing resources")

if __name__ == "__main__":
    main()
