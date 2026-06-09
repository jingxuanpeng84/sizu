#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import subprocess
import threading
from nav_msgs.msg import OccupancyGrid
import time

# ---------------- 配置 ----------------
RTSP_URL = "rtsp://127.0.0.1:8554/mapping"
FPS = 15
OUT_WIDTH = 1920
OUT_HEIGHT = 1080

latest_map = None
map_lock = threading.Lock()

# ---------------- 将 ROS 地图转换为 BGR 图像 ----------------
def map_to_image(map_msg):
    width = map_msg.info.width
    height = map_msg.info.height
    data = np.array(map_msg.data).reshape((height, width))
    data = np.flipud(data)  # 上下翻转

    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[data == -1] = [128, 128, 128]  # 未知
    img[data == 0] = [255, 255, 255]   # 空
    img[data > 0] = [0, 0, 0]          # 占用

    # 保持比例缩放，填满高度（会裁剪左右）
    scale = OUT_HEIGHT / height
    new_w = int(width * scale)
    new_h = OUT_HEIGHT
    img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

    # 如果宽度超出，居中裁剪
    if new_w > OUT_WIDTH:
        w_offset = (new_w - OUT_WIDTH) // 2
        img = img[:, w_offset:w_offset+OUT_WIDTH]
    else:
        # 如果宽度不足，居中放置（保留黑边）
        canvas = np.zeros((OUT_HEIGHT, OUT_WIDTH, 3), dtype=np.uint8)
        w_offset = (OUT_WIDTH - new_w) // 2
        canvas[:, w_offset:w_offset+new_w] = img
        img = canvas

    return img

# ---------------- ROS 回调 ----------------
def map_callback(msg):
    global latest_map
    with map_lock:
        latest_map = map_to_image(msg)

# ---------------- FFmpeg 推流线程 ----------------
def streaming_loop():
    global latest_map
    # 等待第一帧地图
    while latest_map is None and not rospy.is_shutdown():
        time.sleep(0.05)

    height, width, _ = latest_map.shape

    # FFmpeg 命令 - 针对 ARM64 硬件编码器优化
    ffmpeg_cmd = [
        'ffmpeg',
        '-y',
        '-f', 'rawvideo',
        '-pix_fmt', 'bgr24',
        '-s', f'{width}x{height}',
        '-r', str(FPS),
        '-i', '-',
        '-c:v', 'libx264',          # 明确使用软件 H.264 编码器
        '-preset', 'ultrafast',     # 最快编码速度
        '-tune', 'zerolatency',     # 零延迟优化
        '-profile:v', 'baseline',   # 基线配置（兼容性最好）
        '-pix_fmt', 'yuv420p',
        '-g', str(FPS),             # GOP 大小
        '-keyint_min', str(FPS),
        '-sc_threshold', '0',
        '-b:v', '1M',               # 降低码率减轻 CPU 负担
        '-maxrate', '1M',
        '-bufsize', '500k',
        '-flags', 'low_delay',
        '-fflags', 'nobuffer',
        '-max_delay', '0',
        '-f', 'rtsp',
        '-rtsp_transport', 'tcp',
        RTSP_URL
    ]

    proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

    while not rospy.is_shutdown():
        with map_lock:
            frame = latest_map.copy()

        try:
            proc.stdin.write(frame.tobytes())
        except BrokenPipeError:
            rospy.logerr("FFmpeg pipe broken")
            break

        time.sleep(1.0 / FPS)

    proc.stdin.close()
    proc.wait()

# ---------------- 主程序 ----------------
if __name__ == "__main__":
    rospy.init_node("map_rtsp_publisher")
    rospy.Subscriber("/projected_map", OccupancyGrid, map_callback, queue_size=1)

    t = threading.Thread(target=streaming_loop)
    t.start()

    rospy.spin()
    t.join()
