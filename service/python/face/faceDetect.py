import cv2
import numpy as np
import requests
import onnxruntime as ort
import time
from io import BytesIO

# 定义视频流大小和通道
WIDTH = 320
HEIGHT = 240
CHANNELS = 3

# 加载预训练的模型
model_path = 'models/version-slim-320.onnx'
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
                print("Best box (x1, y1, x2, y2):", best_box)
            else:
                print("No detection")

            end_time = time.time()
            process_duration = (end_time - start_time) * 1000
            sleep_time = max(0, 200 - process_duration)
            time.sleep(sleep_time / 1000)
    finally:
        print("Releasing resources")

if __name__ == "__main__":
    main()
