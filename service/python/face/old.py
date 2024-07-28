import cv2
import numpy as np
import mmap
import os
import time
import onnxruntime as ort
import struct

# 定义共享内存相关参数
PROC_SHM_NAME = "/dev/shm/proc_video_shm"
RESULT_SHM_NAME = "/dev/shm/result_shm"
WIDTH = 320
HEIGHT = 240
CHANNELS = 3
PROC_SHM_SIZE = WIDTH * HEIGHT * CHANNELS
RESULT_SHM_SIZE = 4 * 4  # 存储4个float变量

# 加载预训练的Ultra-Light-Fast-Generic-Face-Detector-1MB模型
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

if __name__ == "__main__":
    # 打开共享内存
    shm_fd = os.open(PROC_SHM_NAME, os.O_RDONLY)
    memory = mmap.mmap(shm_fd, PROC_SHM_SIZE, mmap.MAP_SHARED, mmap.PROT_READ)

    # 创建或打开存储结果的共享内存
    result_shm_fd = os.open(RESULT_SHM_NAME, os.O_CREAT | os.O_RDWR)
    os.ftruncate(result_shm_fd, RESULT_SHM_SIZE)
    result_memory = mmap.mmap(result_shm_fd, RESULT_SHM_SIZE, mmap.MAP_SHARED, mmap.PROT_WRITE)

    while True:
        start_time = time.time()

        # 读取并预处理共享内存中的图像数据
        memory.seek(0)
        frame_data = memory.read(PROC_SHM_SIZE)
        img = np.frombuffer(frame_data, dtype=np.uint8).reshape(HEIGHT, WIDTH, CHANNELS)
        input_blob = np.expand_dims(np.transpose((img.astype(np.float32) - 127.0) / 128.0, [2, 0, 1]), axis=0)
        
        # 推理
        scores, boxes = session.run(output_names, {input_name: input_blob})

        # 后处理
        best_box = postprocess(scores, boxes)

        # 将结果写入共享内存
        if best_box:
            x1, y1, x2, y2 = best_box
            result_memory.seek(0)
            result_memory.write(struct.pack('ffff', x1, y1, x2, y2))
        else:
            result_memory.seek(0)
            result_memory.write(struct.pack('ffff', -1, 0, 0, 0))

        elapsed_time = (time.time() - start_time) * 1000
        print(f"Elapsed Time: {elapsed_time:.2f} ms")

        if elapsed_time < 100:
            time.sleep((100 - elapsed_time) / 1000)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    memory.close()
    os.close(shm_fd)
    result_memory.close()
    os.close(result_shm_fd)
