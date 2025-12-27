from StereoStreamer import UdpImageSender
from camera_datacollection import RealsenseCamera
import time
import numpy as np

# 고정된 크기의 빈 이미지 (예: 480×1280)
dummy_img = np.zeros((480, 1280, 3), dtype=np.uint8)

cam1 = RealsenseCamera(name_keyword="D405", height=480, width=640,
                       fps=60, use_color=True, use_depth=False,
                       use_streo=True, reset_on_start=True)
black_image = np.zeros((480, int(640*2), 3), dtype=np.uint8)

time.sleep(1)
if cam1.is_opened:
    cam1.start_reader()

sender = UdpImageSender(
    ip='192.168.0.133', port=9003,
    width=int(640*2), height=480,
    max_payload=1400,
    jpeg_quality=50
)
sender.open()
sender.connect()

#you can set focus, size
resp = sender.set_stereo_params("192.168.0.133", focus=0.0, quad=1.8, zoom=1.0, add_focus=False);

try:
    while True:
        # 실제 이미지가 있다면 dummy_img 대신 넘겨주세요
        if cam1.is_opened and cam1.frame_queue:
            c, d, s1, s2 = cam1.frame_queue.popleft()
            black_image[:,:640,0] = s1
            black_image[:,:640,1] = s1
            black_image[:,:640,2] = s1
            black_image[:,640:,0] = s2
            black_image[:,640:,1] = s2
            black_image[:,640:,2] = s2
            sender.send_image(black_image)
        time.sleep(1/60)
            
finally:
    sender.close()
