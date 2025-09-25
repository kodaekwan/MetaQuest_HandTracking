from StereoStreamer import UdpImageSender
from camera_datacollection import RealsenseCamera
import time
import numpy as np

# 고정된 크기의 빈 이미지 (예: 480×1280)
dummy_img = np.zeros((480, 1280, 3), dtype=np.uint8)

cam1 = RealsenseCamera(name_keyword="D405", height=480, width=640,
                       fps=30, use_color=True, use_depth=False,
                       use_streo=True, reset_on_start=True)
black_image = np.zeros((480, int(640*2), 3), dtype=np.uint8)

time.sleep(1)
if cam1.is_opened:
    cam1.start_reader()

sender = UdpImageSender(
    ip='192.168.0.133', port=9003,
    width=int(640*2), height=480,
    max_payload=1024,
    jpeg_quality=50
)
sender.open()
sender.connect()

try:
    while True:
        # 실제 이미지가 있다면 dummy_img 대신 넘겨주세요
        if cam1.is_opened and cam1.frame_queue:
            c, d, s1, s2 = cam1.frame_queue.popleft()
            black_image[:,:640,0] = s1.copy();
            black_image[:,:640,1] = s1.copy();
            black_image[:,:640,2] = s1.copy();

            black_image[:,640:,0] = s2.copy();
            black_image[:,640:,1] = s2.copy();
            black_image[:,640:,2] = s2.copy();
            sender.send_image(black_image)
        time.sleep(1/30)
finally:
    sender.close()
