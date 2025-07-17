import os
# 원격 서버 연결시
# os.environ["DISPLAY"] = '192.168.0.XXX:0.0'
# os.environ["LIBGL_ALWAYS_INDIRECT"] = 'X'

import socket
import struct
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt5 import QtWidgets
import sys
import time
import threading
from collections import deque
from scipy.spatial.transform import Rotation as R

# === 네트워크 설정 ===(메타퀘스트 IP 및 포트 번호)
SERVER_IP = "192.168.0.133"
SERVER_PORT = 9001
BYTES_TOTAL = 1500

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)  # 1MB
sock.bind(("0.0.0.0", SERVER_PORT))
sock.settimeout(1.0)

packet_queue = deque(maxlen=1)

# === ping 루프 ===
def ping_loop():
    while True:
        try:
            sock.sendto(b"ping", (SERVER_IP, SERVER_PORT))
        except:
            pass
        time.sleep(0.5)

threading.Thread(target=ping_loop, daemon=True).start()

def udp_receiver_loop():
    while True:
        try:
            data, _ = sock.recvfrom(8192)
            packet_queue.clear()
            packet_queue.append(data)
        except:
            pass

threading.Thread(target=udp_receiver_loop, daemon=True).start()

# === 좌표계 변환 (Unity → Real-world) ===
RM_U2R = np.array([
    [0, 0, 1],
    [-1, 0, 0],
    [0, 1, 0]
])

#RM_U2R = np.eye(3)

# === 본 연결 정보 ===
bone_connection = [
    (0, 1), (1, 2), (2, 3), (3, 4), (4, 5),
    (1, 6), (6, 7), (7, 8), (8, 9), (9, 10),
    (1, 11), (11, 12), (12, 13), (13, 14), (14, 15),
    (1, 16), (16, 17), (17, 18), (18, 19), (19, 20),
    (1, 21), (21, 22), (22, 23), (23, 24), (24, 25),
    (2, 6), (6, 11), (11, 16), (16, 21)
]

# === PyQtGraph 초기화 ===
app = QtWidgets.QApplication(sys.argv)
w = gl.GLViewWidget()
w.show()
w.setWindowTitle('XRHand Full Pose Viewer')
w.setCameraPosition(distance=1.5, azimuth=60, elevation=30 ) # <- 유니티 좌표계 기준 시점

# x축 (빨강)
x_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [5, 0, 0]]), color=(1, 0, 0, 1), width=3, antialias=True)
# y축 (초록)
y_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 5, 0]]), color=(0, 1, 0, 1), width=3, antialias=True)
# z축 (파랑, 사용자 쪽에서 보면 안쪽 방향)
z_axis = gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 5]]), color=(0, 0, 1, 1), width=3, antialias=True)
w.addItem(x_axis)
w.addItem(y_axis)
w.addItem(z_axis)

for axis in [gl.GLGridItem() for _ in range(3)]:
    w.addItem(axis)

# 좌우 손 시각화 객체
scatter_l = gl.GLScatterPlotItem(size=10, color=(1, 0, 0, 1))
scatter_r = gl.GLScatterPlotItem(size=10, color=(0, 0, 1, 1))
w.addItem(scatter_l)
w.addItem(scatter_r)

lines_l = [gl.GLLinePlotItem(color=(1, 0, 0, 1), width=2) for _ in bone_connection]
lines_r = [gl.GLLinePlotItem(color=(0, 0, 1, 1), width=2) for _ in bone_connection]
for l in lines_l + lines_r:
    w.addItem(l)

# 추가 (최상단)
root_axes_l = [gl.GLLinePlotItem(color=c, width=3) for c in [(1,0,0,1), (0,1,0,1), (0,0,1,1)]]
root_axes_r = [gl.GLLinePlotItem(color=c, width=3) for c in [(1,0,0,1), (0,1,0,1), (0,0,1,1)]]
for ax in root_axes_l + root_axes_r:
    w.addItem(ax)


# 방향 축 (RGB 기준축)
def create_axes(length=0.03):
    axes = []
    for color, axis in zip([(1,0,0,1), (0,1,0,1), (0,0,1,1)], np.eye(3)):
        item = gl.GLLinePlotItem(color=color, width=2)
        axes.append(item)
        w.addItem(item)
    return axes

axes_l = [create_axes() for _ in range(26)]
axes_r = [create_axes() for _ in range(26)]

# 헤드셋 추가
axes_h = [create_axes() for _ in range(1)]

# === 복원 함수 ===
def recover_world_pose(root_pos, root_rot_q, rel_pos, rel_rot_q):
    """
    Unity 기준 상대 pose → 절대 pose 복원 (Unity 좌표계 기준)
    이후 위치는 RM_U2R.T, 회전은 RM_U2R * R 로 바깥에서 처리
    """
    rel_pos_world = root_rot_q.apply(rel_pos)
    abs_pos = root_pos + rel_pos_world
    abs_rot = root_rot_q * rel_rot_q
    return abs_pos, abs_rot


def update_head(raw_data, axes):
    head_pos = raw_data[0:3]
    head_rot = R.from_quat((raw_data[3:7]))

    points_unity = [head_pos]
    rotations_unity = [head_rot]

    # 🎯 여기서 일괄적으로 좌표계 변환 적용
    points = [RM_U2R @ p for p in points_unity]
    # 회전 객체 리스트를 유지 (쿼터니언 그대로 사용)
    rotations = rotations_unity

    # 회전 행렬은 별도로 변환된 것만 보관 (시각화 전용)
    rot_mats = [RM_U2R @ r.as_matrix() for r in rotations]
    
    points = np.array(points)

    for i in range(1):
        origin = points[i]
        Rmat = rot_mats[i]
        for j in range(3):  # x,y,z 축
            axes[i][j].setData(pos=np.array([origin, origin + Rmat[:, j]*0.08]))
    

# === 본 업데이트 ===
def update_hand(raw_data, scatter, lines, axes, root_axes):
    # 손목 위치와 회전 (Unity 기준으로 그대로 사용)
    root_pos = raw_data[0:3]
    root_rot = R.from_quat((raw_data[3:7]))

    points_unity = [root_pos]
    rotations_unity = [root_rot]

    ptr = 7
    for _ in range(25):
        rel_pos = raw_data[ptr:ptr+3]; ptr += 3
        rel_rot = raw_data[ptr:ptr+4]; ptr += 4

        rel_rot = R.from_quat(rel_rot)
        abs_pos_u, abs_rot_u = recover_world_pose(root_pos, root_rot, rel_pos, rel_rot)

        points_unity.append(abs_pos_u)
        rotations_unity.append(abs_rot_u)

    # 🎯 여기서 일괄적으로 좌표계 변환 적용
    #points = [p @ RM_U2R.T for p in points_unity]
    points = [RM_U2R @ p for p in points_unity]
    # 회전 객체 리스트를 유지 (쿼터니언 그대로 사용)
    rotations = rotations_unity

    # 회전 행렬은 별도로 변환된 것만 보관 (시각화 전용)
    rot_mats = [RM_U2R @ r.as_matrix() for r in rotations]

    points = np.array(points)
    scatter.setData(pos=points)

    for i, (a, b) in enumerate(bone_connection):
        lines[i].setData(pos=np.array([points[a], points[b]]))

    for i in range(26):
        origin = points[i]
        Rmat = rot_mats[i]
        for j in range(3):  # x,y,z 축
            axes[i][j].setData(pos=np.array([origin, origin + Rmat[:, j]*0.03]))


    # === 손목 좌표계 방향 시각화 ===
    origin = points[0]
    Rmat = rot_mats[0]
    for j in range(3):  # x/y/z
        axis_line = np.array([origin, origin + Rmat[:, j] * 0.05])
        root_axes[j].setData(pos=axis_line)

# === 디버깅용 시간 기록 변수 ===
is_Time_Check = True;
recv_times = []
last_print_time = time.time()
unity_time_offset = None


# === 메인 루프 ===
def update():
    if not packet_queue:
        return

    data = packet_queue.popleft()
    print(len(data))
    if len(data) != BYTES_TOTAL or data[:4] != b"HND0" or data[-4:] != b"HND1":
        return

    ts = struct.unpack("d", data[4:12])[0]
    arr = np.frombuffer(data[12:-4], dtype=np.float32)

    arr_l = arr[:182]
    arr_r = arr[182:-7]
    arr_h = arr[-7:]
    

    update_hand(arr_l, scatter_l, lines_l, axes_l, root_axes_l)
    update_hand(arr_r, scatter_r, lines_r, axes_r, root_axes_r)
    update_head(arr_h,axes_h)

    w.setWindowTitle(f"XRHand Full Pose Viewer | ts={ts:.3f}")

    if(is_Time_Check):
        global recv_times, last_print_time,unity_time_offset
        # === 디버깅 시간 기록 ===
        ts_sent = struct.unpack("d", data[4:12])[0]  # Unity에서 보낸 timestamp
        ts_now = time.time()                         # Python 수신 시각

        if unity_time_offset is None:
            unity_time_offset = ts_now - ts_sent

        # 이후 실제 지연 시간 계산
        delay_ms = (ts_now - (ts_sent + unity_time_offset)) * 1000
        print(f"[지연 시간] Unity→Python delay: {delay_ms:.2f} ms")

        recv_time = time.time()  # 🔹 수신 시각 기록
        recv_times.append(recv_time)
        if len(recv_times) > 50:
            recv_times.pop(0)

        # 🔹 1초 간격으로 통계 출력
        if recv_time - last_print_time >= 1.0 and len(recv_times) > 2:
            intervals = np.diff(recv_times)
            mean_interval = np.mean(intervals)
            std_jitter = np.std(intervals)
            max_jitter = np.max(intervals) - np.min(intervals)
            print(f"[통계] 평균 간격: {mean_interval*1000:.2f} ms | 지터(std): {std_jitter*1000:.2f} ms | 최대 지터: {max_jitter*1000:.2f} ms")
            last_print_time = recv_time


timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(20)

# === 실행 ===
if __name__ == "__main__":
    sys.exit(app.exec_())