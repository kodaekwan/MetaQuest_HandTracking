import threading
import collections
import time
import cv2
import numpy as np
import socket
import struct

class UdpImageSender:
    def __init__(self, ip, port, width, height, max_payload=60*1024, jpeg_quality=50):
        self.ip = ip
        self.port = port
        self.width = width
        self.height = height
        self.max_payload = max_payload
        self.jpeg_quality = jpeg_quality

        self.sock: socket.socket | None = None
        self.frame_id = 0
        self.connected = False

        self._queue = collections.deque(maxlen=1)  # 최신 1장만 유지
        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)

    def open(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
        self._stop_event.clear()
        if not self._worker.is_alive():
            self._worker = threading.Thread(target=self._worker_loop, daemon=True)
            self._worker.start()

    def connect(self):
        if self.sock is None:
            raise RuntimeError("소켓이 생성되지 않았습니다. 먼저 open()을 호출하세요.")
        self.sock.connect((self.ip, self.port))
        self.connected = True

    def send_image(self, img: np.ndarray):
        """
        이미지를 덱에 넣고 바로 반환. 실제 압축·송출은 백그라운드 스레드에서.
        """
        # 최신 1장만 유지, 오래된 건 자동 삭제됨
        self._queue.append(img.copy())

    def _worker_loop(self):
        while not self._stop_event.is_set():
            if not self._queue:
                time.sleep(0.005)
                continue

            # 최신 프레임만 꺼냄 (지연 누적 방지)
            img = self._queue.pop()
            self._queue.clear()  # 혹시 더 쌓였다면 비움

            h, w = img.shape[:2]
            if (w, h) != (self.width, self.height):
                img = cv2.resize(img, (self.width, self.height))

            # JPEG 인코딩
            success, buf = cv2.imencode(".jpg", img,
                                        [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            if not success:
                print("[WARN] JPEG 인코딩 실패")
                continue
            data = buf.tobytes()

            fid = self.frame_id & 0xFFFFFFFF
            self.frame_id += 1
            chunks = [data[i:i + self.max_payload]
                      for i in range(0, len(data), self.max_payload)]
            total = len(chunks)
            for idx, chunk in enumerate(chunks):
                header = struct.pack('!IHH', fid, idx, total)
                packet = header + chunk
                try:
                    if self.connected:
                        self.sock.send(packet)
                    else:
                        self.sock.sendto(packet, (self.ip, self.port))
                except Exception as e:
                    print(f"[UDP 전송 오류] {e}")

    def close(self):
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        if self.sock:
            self.sock.close()
            self.sock = None
            self.connected = False
