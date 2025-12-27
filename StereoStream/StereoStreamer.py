import threading
import socket
import struct
import time
import json  # 위로 이동
import queue # Deque 대신 Thread-safe Queue 사용
import numpy as np
import cv2

try:
    # pip install PyTurboJPEG
    from turbojpeg import TurboJPEG, TJPF_GRAY, TJSAMP_GRAY, TJPF_BGR
    _USE_TURBOJPEG = True
except ImportError:
    TurboJPEG = None
    _USE_TURBOJPEG = False
    print("[INFO] TurboJPEG 모듈이 없어 OpenCV(imencode)로 fallback합니다.")

class UdpImageSender:
    # max_payload: 1400 bytes (일반적인 MTU 1500 - 헤더 크기) 권장. 
    # 60KB로 설정하면 WiFi나 일반 라우터에서 패킷이 자주 유실됩니다.
    def __init__(self, ip, port, width, height, max_payload=1400, jpeg_quality=50):
        self.ip = ip
        self.port = port
        self.width = width
        self.height = height
        self.max_payload = max_payload
        self.jpeg_quality = jpeg_quality

        self.sock: socket.socket | None = None
        self.frame_id = 0
        self.connected = False

        # maxsize=1로 설정하여 가장 최신 프레임만 유지 (자동 Drop 기능 대체)
        self._queue = queue.Queue(maxsize=1)
        self._stop_event = threading.Event()
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        
        if _USE_TURBOJPEG:
            self.jpeg = TurboJPEG()
        else:
            self.jpeg = None

    def open(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # 송신 버퍼 크기 늘리기 (고해상도 전송 시 필수)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        self._stop_event.clear()
        
        if not self._worker.is_alive():
            self._worker = threading.Thread(target=self._worker_loop, daemon=True)
            self._worker.start()

    def connect(self):
        if self.sock is None:
            raise RuntimeError("소켓이 생성되지 않았습니다. 먼저 open()을 호출하세요.")
        self.sock.connect((self.ip, self.port))
        self.connected = True

    def close(self):
        self._stop_event.set()
        # 스레드가 대기 중일 수 있으므로 빈 데이터를 보내 깨울 수도 있음 (선택)
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        if self.sock:
            self.sock.close()
            self.sock = None
            self.connected = False

    def encode_jpeg(self, img, quality=95):
        # 입력 이미지가 2차원(흑백)인지 3차원(컬러)인지 확인
        is_gray = (img.ndim == 2)

        if _USE_TURBOJPEG:
            # [수정 2] 컬러일 경우 None이 아니라 TJPF_BGR을 사용해야 함
            if is_gray:
                pixel_format = TJPF_GRAY
            else:
                # OpenCV 이미지는 기본적으로 BGR 순서입니다.
                pixel_format = TJPF_BGR 
            
            flags = 0
            return self.jpeg.encode(img, quality=quality, pixel_format=pixel_format, flags=flags)
        else:
            # OpenCV Fallback
            params = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            success, buf = cv2.imencode(".jpg", img, params)
            if not success:
                print("[WARN] JPEG 인코딩 실패")
                return b''
            return buf.tobytes()

    def send_image(self, img: np.ndarray):
        """
        메인 스레드: 이미지를 큐에 넣음. 큐가 꽉 차 있다면(이전 프레임 처리 중) 즉시 버림(Drop).
        """
        if self._stop_event.is_set():
            return

        try:
            # put_nowait: 큐가 꽉 차면 Full 예외 발생 -> 최신성 유지를 위해 이전 것 무시
            self._queue.put_nowait(img)
        except queue.Full:
            pass # 이전 프레임이 아직 전송 중이면 이번 프레임은 쿨하게 드랍

    def _worker_loop(self):
        while not self._stop_event.is_set():
            try:
                # 0.1초 동안 기다리며 이미지 꺼내기 (Polling 방식인 sleep보다 효율적)
                img = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # 1. 리사이즈 (필요한 경우에만 수행하여 CPU 절약)
            h, w = img.shape[:2]
            if (w, h) != (self.width, self.height):
                img = cv2.resize(img, (self.width, self.height))

            # 2. JPEG 인코딩
            data = self.encode_jpeg(img, self.jpeg_quality)
            if not data:
                continue

            # 3. 패킷 분할 및 전송
            fid = self.frame_id & 0xFFFFFFFF
            self.frame_id += 1
            
            # 리스트 컴프리헨션 대신 제너레이터 스타일로 순회 (메모리 절약)
            total_len = len(data)
            total_packets = (total_len + self.max_payload - 1) // self.max_payload

            for idx in range(total_packets):
                start = idx * self.max_payload
                end = min(start + self.max_payload, total_len)
                chunk = data[start:end]

                # Header: FrameID(4) + PacketIdx(2) + TotalPackets(2) = 8 bytes
                header = struct.pack('!IHH', fid, idx, total_packets)
                
                try:
                    if self.connected:
                        self.sock.send(header + chunk)
                    else:
                        self.sock.sendto(header + chunk, (self.ip, self.port))
                except OSError as e:
                    # 버퍼 가득 참 등의 일시적 오류 무시
                    # print(f"[UDP Error] {e}") 
                    pass

    def set_stereo_params(self, host: str, port: int = 9004,
                          focus: float | None = None,
                          quad: float | None = None,
                          zoom: float | None = None,    # [추가됨] 줌 제어용
                          add_focus: bool | None = None,
                          timeout: float = 3.0):
        """
        Unity 또는 Vision Pro 등 외부 기기에 파라미터 전송
        - focus: 양안 시차 조절
        - quad: 스크린 거리(Z축)
        - zoom: 테두리 제거를 위한 화면 확대 (1.0 ~ 2.0)
        """
        payload = {}
        if focus is not None:     payload["focus"] = float(focus)
        if quad  is not None:     payload["quad"]  = float(quad)
        if zoom  is not None:     payload["zoom"]  = float(zoom)  # [추가됨]
        if add_focus is not None: payload["addFocus"] = bool(add_focus)

        line = json.dumps(payload)
        resp = {}

        try:
            with socket.create_connection((host, port), timeout=timeout) as sock:
                sock.sendall((line + "\n").encode("utf-8"))
                sock.shutdown(socket.SHUT_WR)

                data = b""
                while True:
                    chunk = sock.recv(4096)
                    if not chunk: break
                    data += chunk
                
                text = data.decode("utf-8", errors="ignore").strip()
                if text:
                    resp = json.loads(text)
        except Exception as e:
            resp = {"error": str(e)}

        return resp