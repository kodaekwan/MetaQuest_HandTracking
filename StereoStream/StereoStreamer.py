import socket
import struct
import cv2
import numpy as np

class UdpImageSender:
    """
    고정된 크기(width×height)의 이미지를 JPEG로 인코딩하여
    UDP로 분할 전송하는 유틸리티 클래스
    """

    def __init__(self,
                 ip: str,
                 port: int,
                 width: int,
                 height: int,
                 max_payload: int = 60 * 1024,
                 jpeg_quality: int = 50):
        """
        :param ip: 전송 대상 IP 주소
        :param port: 전송 대상 포트 번호
        :param width: 전송할 이미지의 너비 (픽셀)
        :param height: 전송할 이미지의 높이 (픽셀)
        :param max_payload: UDP 페이로드 최대 크기 (바이트)
        :param jpeg_quality: JPEG 인코딩 품질 (0~100)
        """
        self.ip = ip
        self.port = port
        self.width = width
        self.height = height
        self.max_payload = max_payload
        self.jpeg_quality = jpeg_quality

        self.sock: socket.socket | None = None
        self.frame_id = 0
        self.connected = False

    def open(self):
        """UDP 소켓을 생성합니다."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def connect(self):
        """
        소켓을 특정 IP:포트에 연결(connect)하여
        send()만으로 전송할 수 있도록 설정합니다.
        """
        if self.sock is None:
            raise RuntimeError("소켓이 생성되지 않았습니다. 먼저 open()을 호출하세요.")
        self.sock.connect((self.ip, self.port))
        self.connected = True

    def send_image(self, img: np.ndarray):
        """
        이미지를 고정된 크기로 맞추고 JPEG로 인코딩,
        여러 UDP 패킷으로 분할 전송합니다.
        :param img: HxWxC 형태의 NumPy 배열
        """
        if self.sock is None:
            raise RuntimeError("소켓이 생성되지 않았습니다. 먼저 open()을 호출하세요.")

        # 1) 크기 검사 및 리사이즈
        h, w = img.shape[:2]
        if (w, h) != (self.width, self.height):
            img = cv2.resize(img, (self.width, self.height))

        # 2) JPEG 인코딩
        success, buf = cv2.imencode(".jpg", img,
                                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not success:
            raise RuntimeError("JPEG 인코딩에 실패했습니다.")
        data = buf.tobytes()

        # 3) 프레임 ID (32비트 순환)
        fid = self.frame_id & 0xFFFFFFFF
        self.frame_id += 1

        # 4) 페이로드 분할
        chunks = [data[i:i + self.max_payload]
                  for i in range(0, len(data), self.max_payload)]
        total = len(chunks)

        # 5) 헤더 붙여 전송
        # 헤더 포맷: !IHH → frame_id:uint32, packet_idx:uint16, total_packets:uint16
        for idx, chunk in enumerate(chunks):
            header = struct.pack('!IHH', fid, idx, total)
            packet = header + chunk
            if self.connected:
                self.sock.send(packet)
            else:
                self.sock.sendto(packet, (self.ip, self.port))

    def close(self):
        """소켓을 닫고 리소스를 해제합니다."""
        if self.sock:
            self.sock.close()
            self.sock = None
            self.connected = False
