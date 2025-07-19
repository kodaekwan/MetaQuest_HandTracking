import socket
import struct
import numpy as np
import threading
import time
from collections import deque
from scipy.spatial.transform import Rotation as R

class XRHandReceiver:
    def __init__(self,
                 server_ip="192.168.0.133",
                 server_port=9001,
                 buffer_size=1500):
        self.server_ip = server_ip
        self.server_port = server_port
        self.buffer_size = buffer_size
        self.sock = None
        self.packet_queue = deque(maxlen=1)
        self.connected = False
        self._lock = threading.Lock()

        self.RM_U2R = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, 1, 0]
        ])

        # 쓰레드 시작
        self._start_threads()

    def connect(self):
        """UDP 소켓 연결 및 바인드"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)  # 1MB
        self.sock.bind(("0.0.0.0", self.server_port))
        self.sock.settimeout(1.0)
        self.connected = True

    def _start_threads(self):
        """Ping 및 수신 쓰레드 실행"""
        threading.Thread(target=self._ping_loop, daemon=True).start()
        threading.Thread(target=self._receiver_loop, daemon=True).start()

    def _ping_loop(self):
        while True:
            if not self.connected:
                time.sleep(0.5)
                continue
            try:
                self.sock.sendto(b"ping", (self.server_ip, self.server_port))
            except Exception:
                pass
            time.sleep(0.5)

    def _receiver_loop(self):
        while True:
            if not self.connected:
                time.sleep(0.1)
                continue
            try:
                data, _ = self.sock.recvfrom(8192)
                with self._lock:
                    self.packet_queue.clear()
                    self.packet_queue.append(data)
            except Exception:
                pass

    def get(self):
        """가장 최근의 패킷 반환 (없으면 None)"""
        with self._lock:
            return self.packet_queue[-1] if self.packet_queue else None

    def convert_unity_pose_to_robot(self, position, quaternion):
        """
        Unity 좌표계 기준의 위치와 쿼터니언을
        로봇 좌표계 기준으로 변환
        """
        pos_robot = self.RM_U2R @ position
        rot_unity = R.from_quat(quaternion)
        rot_robot = self.RM_U2R @ rot_unity.as_matrix() @ self.RM_U2R.T
        return pos_robot, rot_robot

    def parse(self, data):
        """HND0/HND1 검증 및 구조 파싱 + 로봇 좌표계 변환 포함"""
        if data is None or len(data) != self.buffer_size or data[:4] != b"HND0" or data[-4:] != b"HND1":
            return None
        ts = struct.unpack("d", data[4:12])[0]
        arr = np.frombuffer(data[12:-4], dtype=np.float32)
        arr_l = arr[:182]
        arr_r = arr[182:-7]
        arr_h = arr[-7:]

        # === 좌표계 변환: 손목 위치 + 회전 쿼터니언만 변환 ===
        left_pos_u = arr_l[0:3]
        left_rot_u = arr_l[3:7]
        right_pos_u = arr_r[0:3]
        right_rot_u = arr_r[3:7]
        head_pos_u = arr_h[0:3]
        head_rot_u = arr_h[3:7]

        left_pos_r, left_rotmat_r = self.convert_unity_pose_to_robot(left_pos_u, left_rot_u)
        right_pos_r, right_rotmat_r = self.convert_unity_pose_to_robot(right_pos_u, right_rot_u)
        head_pos_r, head_rotmat_r = self.convert_unity_pose_to_robot(head_pos_u, head_rot_u)

        return {
            "timestamp": ts,
            "left_raw": arr_l,
            "right_raw": arr_r,
            "head_raw": arr_h,
            "left_robot": {
                "pos": left_pos_r,
                "rotmat": left_rotmat_r
            },
            "right_robot": {
                "pos": right_pos_r,
                "rotmat": right_rotmat_r
            },
            "head_robot": {
                "pos": head_pos_r,
                "rotmat": head_rotmat_r
            }
        }