from os import name
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

        self.bone_link = {
            "root_LINK":[(0, 1),                                   ],# wrist-hand root link
            "thumb_LINK":[(1, 2), (2, 3), (3, 4), (4, 5),           ],# Thumb link
            "index_LINK":[(1, 6), (6, 7), (7, 8), (8, 9), (9, 10),   ],# Index link
            "middle_LINK":[(1, 11), (11, 12), (12, 13), (13, 14), (14, 15),  ],# Middle link
            "ring_LINK":[(1, 16), (16, 17), (17, 18), (18, 19), (19, 20),  ],# Ring link
            "little_LINK":[(1, 21), (21, 22), (22, 23), (23, 24), (24, 25),  ],# Little link
            "palm_LINK":[(2, 6), (6, 11), (11, 16), (16, 21)               ],# palm transverse
        }

        self.bone_indexs = {
            "wrist":    [0],
            "hand":     [1],
            "thumb":    [2, 3, 4, 5],
            "index":    [6, 7, 8, 9, 10],
            "middle":   [11, 12, 13, 14, 15],
            "ring":     [16, 17, 18, 19, 20],
            "little":   [21, 22, 23, 24, 25],
        }

        self.previous_pos_list = [];
        self.previous_quat_list = [];

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
        if np.linalg.norm(position) < 1e-6:
            print("Warning: Zero position vector received. position={}".format(position))
        if np.linalg.norm(quaternion) < 1e-6:
            print("Warning: Zero quaternion vector received. quaternion={}".format(quaternion))

        rot_unity = R.from_quat(quaternion)
        rot_robot = self.RM_U2R @ rot_unity.as_matrix() @ self.RM_U2R.T
        return pos_robot, rot_robot
    
    def get_finger_robotTM_by_parsed(self, parsed:dict, parts_name= "left" ,bone_name = "thumb", index=0):
        """파싱된 데이터에서 특정 손가락의 로봇 좌표계 변환 행렬 반환
         - parts_name: "left" or "right"
         - bone_name: "thumb", "index", "middle", "ring", "little"
         - index: 0~4 (각 손가락의 관절 인덱스)
        """
        data_length = 7;# pos(3) + quat(4)
        
        if not (parts_name == "left" or parts_name == "right"):
            print("Error: parts_name should be 'left' or 'right'");
            return None;
        if bone_name not in self.bone_indexs:
            print("Error: bone_name should be one of ", list(self.bone_indexs.keys()));
            return None;
        if  index >= len(self.bone_indexs[bone_name]):
            print("Error: index out of range for bone ", bone_name);
            return None;
            
        bone_idx = self.bone_indexs[bone_name][index];
        raw_data = parsed[parts_name+"_raw"];


        bone_head_idx = int(data_length*bone_idx);
        Thumb1_pos,Thumb1_quat = raw_data[bone_head_idx:bone_head_idx+3], raw_data[bone_head_idx+3:bone_head_idx+7];
        Thumb1_pos, Thumb1_rotmat = self.convert_unity_pose_to_robot(Thumb1_pos, Thumb1_quat);
        
        Thumb1_TM = np.eye(4);
        Thumb1_TM[0:3,0:3] = Thumb1_rotmat;
        Thumb1_TM[0:3,3] = Thumb1_pos;
        return Thumb1_TM
    
    def get_head_robotTM_by_parsed(self, parsed:dict):
        """파싱된 데이터에서 헤드의 로봇 좌표계 변환 행렬 반환"""
        raw_data = parsed["head_raw"];
        head_pos_u = raw_data[0:3]
        head_rot_u = raw_data[3:7]
        head_pos_r, head_rotmat_r = self.convert_unity_pose_to_robot(head_pos_u, head_rot_u)

        head_TM = np.eye(4);
        head_TM[0:3,0:3] = head_rotmat_r;
        head_TM[0:3,3] = head_pos_r;
        return head_TM

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
    
    def convert_parsed_to_robot_hand_RH56F1(self, parsed:dict, hand_type="right"):
        """
        이 함수는 파싱된 데이터에서 특정 손의 RH56F1 로봇 핸드 관절 각도 벡터 반환한다.

        RH56F1은 Inspire사의 7 DOF 로봇 핸드 모델입니다.
        RH56F1은 7 DOF를 가진 로봇 핸드 모델입니다 (엄지 2 DOF, 다른 손가락 각각 1 DOF)
        
        -------------------------------------------------------------------------
            - parsed: 파싱된 데이터 딕셔너리
            - hand_type: "left" or "right"
        -------------------------------------------------------------------------
        Returns:
            np.ndarray: 관절 각도 벡터 (라디안 단위, 1D 배열)
            np.ndarray: 정규화된 관절 각도 벡터 (0~1 범위)
        -------------------------------------------------------------------------
        
        Description of RH56F1 mapping:
            Dof가 높은 로봇손은 사람손과 위치와 스케일 맞지 않기 때문에 
            본래는 로봇손의 Dof에 맞게 여러 관절을 동시에 제어하기 위해 수치 최적화 기법을 사용해야 합니다.
            하지만, RH56F1는 비교적 단순하기 때문에 비용이 적은 방법으로도 매핑이 가능합니다.
            이때 매핑방법은 사람 손가락의 각 위치와 방향 중 매핑 가능한 기하학적인 위치로부터 관절 각도를 추출합니다.
            따라서 각 손가락의 제어되는 관절은 다음과 같습니다:
            - 엄지손가락: 2 DOF (첫 관절 1개 + 끝단 관절 1개)
            - 검지, 중지, 약지, 새끼손가락: 각각 1 DOF (끝단 관절 1개)
        -------------------------------------------------------------------------
        """

        # 회전n벡터를 기준으로 y축 회전 각도 계산
        def get_y_rotation_angle_based_n(TM):
            n_vec = TM[:3,0]
            x_point,z_point = n_vec[0],n_vec[2];
            y_angle = np.arctan2(z_point, x_point);
            return y_angle
        
        # 위치 기반으로 x축 회전 각도 계산
        def get_x_rotation_angle_based_pos(TM):
            pos_vec = TM[:3,3]
            y_point,z_point = pos_vec[1],pos_vec[2];
            x_angle = np.arctan2(z_point,y_point);
            return x_angle
        # 각도를 0~2pi 범위로 래핑
        def wrapTo2Pi(angle):
            return angle % (2 * np.pi)
        
        # 각도 클램핑 함수들
        def custom0_clamp_angle(angle):
            if angle < np.radians(-10.0):
                angle = np.radians(-10.0);
            elif angle > np.radians(60.0):
                angle = np.radians(60.0);
            return angle
        
        def custom1_clamp_angle(angle):
            if angle > np.radians(270.0):
                angle = np.radians(0.0);
            if angle < np.radians(0.0):
                angle = 0.0;
            elif angle > np.radians(50.0):
                angle = np.radians(50.0);
            return angle
        
        def custom2_clamp_angle(angle):
            if angle > np.radians(270.0):
                angle = np.radians(0.0);
            elif angle < np.radians(0.0):
                angle = 0.0;
            elif angle > np.radians(180.0):
                angle = np.radians(180.0);
            return angle
        

        Thumb1_idx = 1
        Thumb3_idx = 3
        Index4_idx  = 4
        Middle4_idx = 4
        Ring4_idx   = 4
        Little4_idx = 4
        
        Thumb1_TM = self.get_finger_robotTM_by_parsed(parsed, hand_type, "thumb", Thumb1_idx);
        Thumb3_TM = self.get_finger_robotTM_by_parsed(parsed, hand_type, "thumb", Thumb3_idx);
        
        # thumb3 관절을 thumb1 기준 좌표계로 변환
        Thumb3_TM_in_Thumb1 = np.linalg.inv(Thumb1_TM)@Thumb3_TM;

        
        # 각 손가락 4번째 마디 Transformation Matrix 얻기(손목기준, mujoco 좌표계 변환)
        Index4_TM = self.get_finger_robotTM_by_parsed(parsed, hand_type, "index", Index4_idx);
        Middle4_TM = self.get_finger_robotTM_by_parsed(parsed, hand_type, "middle", Middle4_idx);
        Ring4_TM = self.get_finger_robotTM_by_parsed(parsed, hand_type, "ring", Ring4_idx);
        Little4_TM = self.get_finger_robotTM_by_parsed(parsed, hand_type, "little", Little4_idx);
        

        
        # 엄지 손가락 x축과 y축 회전 각도 계산
        if hand_type == "left":
            Thumb1_x_angle = wrapTo2Pi(get_x_rotation_angle_based_pos(Thumb1_TM)-np.radians(180.0));    
            Thumb3_y_angle = -1.0*get_y_rotation_angle_based_n(Thumb3_TM_in_Thumb1);
            #print("left Thumb3_y_angle:",np.degrees(Thumb3_y_angle));
        else:
            Thumb1_x_angle = wrapTo2Pi(-1.0*get_x_rotation_angle_based_pos(Thumb1_TM));    
            Thumb3_y_angle = -1.0*get_y_rotation_angle_based_n(Thumb3_TM_in_Thumb1);
            #print("right Thumb3_y_angle:",np.degrees(Thumb3_y_angle));
        
        Thumb1_x_angle  = custom0_clamp_angle(Thumb1_x_angle);
        Thumb3_y_angle  = custom1_clamp_angle(Thumb3_y_angle);

        # 나머지 손가락 y축 회전 각도 계산
        Index4_y_angle = wrapTo2Pi(-1.0*get_y_rotation_angle_based_n(Index4_TM));
        Middle4_y_angle = wrapTo2Pi(-1.0*get_y_rotation_angle_based_n(Middle4_TM));
        Ring4_y_angle = wrapTo2Pi(-1.0*get_y_rotation_angle_based_n(Ring4_TM));
        Little4_y_angle = wrapTo2Pi(-1.0*get_y_rotation_angle_based_n(Little4_TM));
        Index4_y_angle  = custom2_clamp_angle(Index4_y_angle);
        Middle4_y_angle = custom2_clamp_angle(Middle4_y_angle);
        Ring4_y_angle   = custom2_clamp_angle(Ring4_y_angle);
        Little4_y_angle = custom2_clamp_angle(Little4_y_angle);

        # 관절 각도 벡터 생성
        finger_angle_vec = np.array([   Thumb1_x_angle, Thumb3_y_angle,
                                        Index4_y_angle, Middle4_y_angle,
                                        Ring4_y_angle, Little4_y_angle]);
        # 정규화된 관절 각도 벡터 생성
        norm_finger_angle_vec = finger_angle_vec/np.array([np.radians(70), np.radians(50), np.radians(180), np.radians(180), np.radians(180), np.radians(180)]);# 0~1 정규화

        return finger_angle_vec, norm_finger_angle_vec;
    
    #TODO:JWL2000
    def update_hand(self, raw_data, type="rel"):
        root_pos = raw_data[0:3]
        root_rot = R.from_quat((raw_data[3:7]))

        points_unity = [root_pos]
        rotations_unity = [root_rot]

        ptr = 7
        for _ in range(25):
            rel_pos = raw_data[ptr:ptr+3]; ptr += 3
            rel_rot = raw_data[ptr:ptr+4]; ptr += 4

            rel_rot = R.from_quat(rel_rot)
            abs_pos_u, abs_rot_u = self.recover_world_pose(root_pos, root_rot, rel_pos, rel_rot) # 활성화하면 움직임.

            if type == "rel":
                points_unity.append(rel_pos)
                rotations_unity.append(rel_rot)
            else:
                points_unity.append(abs_pos_u)
                rotations_unity.append(abs_rot_u)  

        # left to right coordinate
        points = [self.RM_U2R @ p for p in points_unity]
        rotations = rotations_unity
        rot_mats = [self.RM_U2R @ r.as_matrix() @ self.RM_U2R.T for r in rotations]
        points = pos_weight(points)
        ref_rot = rotmat_weight(rot_mats[0])
        points = [ref_rot @ p for p in points]
        # ============================CUSTOM=============================
        # EE 인덱스
        tip_indices = [10, 15, 20, 5] # index, middle, ring, thumb
        # 상대 위치 벡터 리스트
        p_EE_w = [points[index] for index in tip_indices]
        points = np.array(points)
        # 필요한 인덱스만 다시 할당
        thumb_points = points[2:6]
        index_points = points[7:11]
        middle_points = points[12:16]
        ring_points = points[17:21]
        points = np.vstack([index_points, middle_points, ring_points, thumb_points]) 
        # ============================CUSTOM=============================  
        return p_EE_w, points