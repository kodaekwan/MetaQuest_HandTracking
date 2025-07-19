import os
# 원격 GUI 렌더링 환경 설정 (X-forwarding)
#os.environ["DISPLAY"] = '192.168.0.201:1.0'
#os.environ["LIBGL_ALWAYS_INDIRECT"] = '1'

import sys, time
import numpy as np
from PyQt5 import QtWidgets
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from scipy.spatial.transform import Rotation as R
from XRHandReceiver import XRHandReceiver  # UDP 수신 및 변환 클래스

# === 본 연결 정보 (26점 손 관절 구조) ===
bone_connection = [
    (0, 1), (1, 2), (2, 3), (3, 4), (4, 5),
    (1, 6), (6, 7), (7, 8), (8, 9), (9, 10),
    (1, 11), (11, 12), (12, 13), (13, 14), (14, 15),
    (1, 16), (16, 17), (17, 18), (18, 19), (19, 20),
    (1, 21), (21, 22), (22, 23), (23, 24), (24, 25),
    (2, 6), (6, 11), (11, 16), (16, 21)
]

# === PyQtGraph OpenGL 3D 창 초기화 ===
app = QtWidgets.QApplication(sys.argv)
w = gl.GLViewWidget()
w.show()
w.setWindowTitle('XRHand Full Pose Viewer')
w.setCameraPosition(distance=1.5, azimuth=60, elevation=30)

# === 전역 좌표 축 추가 (빨/초/파: x/y/z) ===
def add_axis():
    w.addItem(gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [5, 0, 0]]), color=(1, 0, 0, 1), width=3))
    w.addItem(gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 5, 0]]), color=(0, 1, 0, 1), width=3))
    w.addItem(gl.GLLinePlotItem(pos=np.array([[0, 0, 0], [0, 0, 5]]), color=(0, 0, 1, 1), width=3))
    for grid in [gl.GLGridItem() for _ in range(3)]:
        w.addItem(grid)

add_axis()

# === 손 시각화 객체 생성 함수 ===
def create_hand_vis(color):
    # 점 + 뼈대 + 관절 방향축 + 손목 좌표축
    scatter = gl.GLScatterPlotItem(size=10, color=color)
    w.addItem(scatter)

    lines = [gl.GLLinePlotItem(color=color, width=2) for _ in bone_connection]
    for l in lines:
        w.addItem(l)

    axes = [create_axes() for _ in range(26)]  # 관절마다 xyz축
    root_axes = [gl.GLLinePlotItem(color=c, width=3) for c in [(1,0,0,1), (0,1,0,1), (0,0,1,1)]]
    for a in root_axes:
        w.addItem(a)

    return scatter, lines, axes, root_axes

# === 각 관절에 방향 축(x/y/z) 시각화 선 생성 ===
def create_axes():
    colors = [(1,0,0,1), (0,1,0,1), (0,0,1,1)]
    axis = []
    for c in colors:
        item = gl.GLLinePlotItem(color=c, width=2)
        w.addItem(item)
        axis.append(item)
    return axis

# === 양손 + 헤드 시각화 객체 초기화 ===
scatter_l, lines_l, axes_l, root_axes_l = create_hand_vis((1,0,0,1))  # 왼손: 빨강
scatter_r, lines_r, axes_r, root_axes_r = create_hand_vis((0,0,1,1))  # 오른손: 파랑
axes_h = [create_axes() for _ in range(1)]  # 헤드셋: 좌표축만

# === 상대 pose → 절대 pose로 복원 ===
def recover_world_pose(root_pos, root_rot_q, rel_pos, rel_rot_q):
    rel_pos_world = root_rot_q.apply(rel_pos)  # 회전 적용
    abs_pos = root_pos + rel_pos_world
    abs_rot = root_rot_q * rel_rot_q
    return abs_pos, abs_rot

# === 손 데이터 시각화 업데이트 ===
def update_hand(raw_data, scatter, lines, axes, root_axes):
    # 손목 위치, 회전
    root_pos = raw_data[0:3]
    root_rot = R.from_quat(raw_data[3:7])

    points, rotations = [root_pos], [root_rot]

    ptr = 7
    for _ in range(25):
        rel_pos = raw_data[ptr:ptr+3]; ptr += 3
        rel_rot = R.from_quat(raw_data[ptr:ptr+4]); ptr += 4
        abs_pos, abs_rot = recover_world_pose(root_pos, root_rot, rel_pos, rel_rot)
        points.append(abs_pos)
        rotations.append(abs_rot)

    # Unity → 로봇 좌표계 변환
    points = np.array([receiver.RM_U2R @ p for p in points])
    rot_mats = [receiver.RM_U2R @ r.as_matrix() @ receiver.RM_U2R.T for r in rotations]

    # 점 위치 표시
    scatter.setData(pos=points)

    # 뼈대 연결
    for i, (a, b) in enumerate(bone_connection):
        lines[i].setData(pos=np.array([points[a], points[b]]))

    # 관절 방향 축 표시
    for i in range(26):
        for j in range(3):
            axes[i][j].setData(pos=np.array([points[i], points[i] + rot_mats[i][:, j] * 0.03]))

    # 손목 기준 좌표축 표시
    for j in range(3):
        root_axes[j].setData(pos=np.array([points[0], points[0] + rot_mats[0][:, j] * 0.05]))

# === 헤드셋 위치 및 방향 표시 ===
def update_head(raw_data, axes):
    pos = receiver.RM_U2R @ raw_data[0:3]
    rot = R.from_quat(raw_data[3:7])
    Rmat = receiver.RM_U2R @ rot.as_matrix() @ receiver.RM_U2R.T
    for j in range(3):
        axes[0][j].setData(pos=np.array([pos, pos + Rmat[:, j] * 0.08]))

# === XRHandReceiver 객체 초기화 및 연결 ===
receiver = XRHandReceiver(server_ip="192.168.0.133")
receiver.connect()

# === 디버깅용 시간 기록 변수 ===
is_Time_Check = True;
recv_times = []
last_print_time = time.time()
unity_time_offset = None


# === 주기적 업데이트 함수 (10ms마다 호출) ===
def update():
    parsed = receiver.parse(receiver.get())
    if parsed is None:
        return
    update_hand(parsed["left_raw"], scatter_l, lines_l, axes_l, root_axes_l)
    update_hand(parsed["right_raw"], scatter_r, lines_r, axes_r, root_axes_r)
    update_head(parsed["head_raw"], axes_h)
    w.setWindowTitle(f"XRHand Viewer | t={parsed['timestamp']:.3f}")

    if(is_Time_Check):
        global recv_times, last_print_time,unity_time_offset
        # === 디버깅 시간 기록 ===
        ts_sent = parsed["timestamp"]                # Unity에서 보낸 timestamp
        ts_now = time.time()                         # Python 수신 시각

        if unity_time_offset is None:
            unity_time_offset = ts_now - ts_sent

        # 이후 실제 지연 시간 계산
        delay_ms = (ts_now - (ts_sent + unity_time_offset)) * 1000
        
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
            print(f"[지연 시간] Unity→Python delay: {delay_ms:.2f} ms")
            print(f"[통계] 평균 간격: {mean_interval*1000:.2f} ms | 지터(std): {std_jitter*1000:.2f} ms | 최대 지터: {max_jitter*1000:.2f} ms")
            last_print_time = recv_time


# === 타이머 기반 반복 실행 등록 ===
timer = pg.QtCore.QTimer()
timer.timeout.connect(update)
timer.start(10)  # 10ms 간격 (100Hz)

# === 실행 시작 ===
if __name__ == "__main__":
    sys.exit(app.exec_())
