import os
import time
import pyrealsense2 as rs
import numpy as np
import cv2
import threading
from collections import deque

class RealsenseCamera:
    def __init__(self, name_keyword=None, height=480, width=640, fps=15, use_color=True, use_depth=False, use_streo=False,reset_on_start=True):
        self.name_keyword = name_keyword
        self.height = height
        self.width = width
        self.fps = fps
        self.use_color = use_color
        self.use_depth = use_depth
        self.use_streo = use_streo
        self.reset_on_start = reset_on_start

        self.pipeline = None
        self.config = None
        self.device = None
        self.is_opened = False

        self.frame_queue = deque(maxlen=1)  # 또는 maxlen=N으로 지정
        self.reader_thread = None
        self.running = False

        self.color_intrinsics = None;
        self.depth_intrinsics = None;
        self.ir1_intrinsics = None;
        self.ir2_intrinsics = None;

        self._setup()

    def _setup(self):
        ctx = rs.context()
        devices = ctx.query_devices()
        found = False
        for dev in devices:
            name = dev.get_info(rs.camera_info.name)
            if self.name_keyword.lower() in name.lower():  # 이름 검색으로 변경
                self.device = dev
                found = True
                break

        if not found:
            print(f"[WARNING] Camera with name containing '{self.name_keyword}' not found.")
            return

        if self.reset_on_start:
            self.hardware_reset()
            time.sleep(2.5)

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # 디바이스 지정
        serial_number = self.device.get_info(rs.camera_info.serial_number)
        self.config.enable_device(serial_number)

        if self.use_color:
            self.config.enable_stream(rs.stream.color, int(self.width), int(self.height), rs.format.bgr8, int(self.fps))
            print(f"set Color {int(self.width)}W {int(self.height)}H, rs.format.bgr8, {self.fps}fps");
            
        if self.use_depth:
            self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, int(self.fps))

        if self.use_streo:
            self.config.enable_stream(rs.stream.infrared, 1, int(self.width), int(self.height), rs.format.y8, int(self.fps))  # Left IR
            self.config.enable_stream(rs.stream.infrared, 2, int(self.width), int(self.height), rs.format.y8, int(self.fps))  # Right IR
            

        time.sleep(1)
        try:
            
            self.pipeline_profile = self.pipeline.start(self.config)
            self.is_opened = True
            print(f"Camera {self.name_keyword} opened successfully.")

            device = self.pipeline_profile.get_device();
            if self.use_streo:
                laser_power_option = rs.option.laser_power
                # 지원 여부 확인
                if device.first_depth_sensor().supports(laser_power_option):
                    depth_sensor = device.first_depth_sensor()
                    depth_sensor.set_option(laser_power_option, 0)
                    print("Laser Emitter Off (laser_power = 0)")
                else:
                    print("Laser Power control not supported.")
                
                ir1_profile = self.pipeline_profile.get_stream(rs.stream.infrared, 1)
                ir2_profile = self.pipeline_profile.get_stream(rs.stream.infrared, 2)
                        

                self.ir1_intrinsics = ir1_profile.as_video_stream_profile().get_intrinsics()
                self.ir2_intrinsics = ir2_profile.as_video_stream_profile().get_intrinsics()

            if self.use_depth:
                depth_profile = self.pipeline_profile.get_stream(rs.stream.depth).as_video_stream_profile()
                self.depth_intrinsics = depth_profile.get_intrinsics();
            
            if self.use_color:
                color_profile = self.pipeline_profile.get_stream(rs.stream.color).as_video_stream_profile()
                self.color_intrinsics = color_profile.get_intrinsics();


        except Exception as e:
            print(f"[WARNING] Failed to start camera {self.name_keyword}: {e}")

    def hardware_reset(self):
        try:
            print(f"[INFO] Resetting device {self.name_keyword}...")
            self.device.hardware_reset()
        except Exception as e:
            print(f"[WARNING] Hardware reset failed: {e}")

    def read(self):
        if not self.is_opened:
            return False, (None, None)
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=2000)
            color_frame = frames.get_color_frame() if self.use_color else None
            depth_frame = frames.get_depth_frame() if self.use_depth else None
            
            streo1_frame = frames.get_infrared_frame(1) if self.use_streo else None
            streo2_frame = frames.get_infrared_frame(2) if self.use_streo else None

            color = np.asanyarray(color_frame.get_data()) if color_frame else None
            depth = np.asanyarray(depth_frame.get_data()) if depth_frame else None            
            streo1 = np.asanyarray(streo1_frame.get_data()) if streo1_frame else None
            streo2 = np.asanyarray(streo2_frame.get_data()) if streo2_frame else None            
            return True, (color, depth, streo1, streo2)
        except Exception:
            return False, (None, None, None, None)
    
    def start_reader(self):
        if not self.is_opened:
            return
        self.running = True
        self.reader_thread = threading.Thread(target=self._camera_reader)
        self.reader_thread.start()

    def _camera_reader(self):
        while self.is_opened and self.running:
            ret, (color, depth, streo1,streo2) = self.read()
            if ret:
                self.frame_queue.append((color, depth, streo1, streo2))

    def stop_reader(self):
        self.running = False
        if self.reader_thread is not None:
            self.reader_thread.join()
            self.reader_thread = None

    def release(self):
        self.stop_reader()

        if self.pipeline and self.is_opened:
            try:
                self.pipeline.stop()
                print(f"[release] Camera {self.name_keyword} pipeline stopped")
            except Exception as e:
                print(f"[WARNING] Release stop error: {e}")

        self.pipeline = None
        self.config = None
        self.device = None
        self.is_opened = False

if __name__ == "__main__":
    # 기본 검정 화면 미리 생성
    black_image = np.zeros((480, 640, 3), dtype=np.uint8)

    # 카메라 이름 키워드로 찾기
    cam1 = RealsenseCamera(name_keyword="D435", height=480, width=848, fps=30, use_color=True, use_depth=False,use_streo=True, reset_on_start=True)
    #cam1 = RealsenseCamera(name_keyword="D405", height=480, width=848, fps=30, use_color=True, use_depth=False,use_streo=False, reset_on_start=True)
    time.sleep(1)
    cam2 = RealsenseCamera(name_keyword="L515", height=480, width=640, fps=30, use_color=True, use_depth=False, reset_on_start=True)

    if cam1.is_opened:
        cam1.start_reader()
    if cam2.is_opened:
        cam2.start_reader()

    while True:
        if cam1.is_opened:
            if not cam1.frame_queue:
                color1, _,s1,s2 = cam1.frame_queue.popleft()
                if color1 is not None:
                    color1_crop = color1[:,848-640:];
                    cv2.imshow("Camera 1 - D435", color1_crop)
                if s1 is not None:
                    cv2.imshow("Camera 1 - D435 s1", s1)
        else:
            # if cv2.getWindowProperty("Camera 1 - D435", cv2.WND_PROP_VISIBLE) < 1:
            #     cv2.imshow("Camera 1 - D435", black_image)
            pass

        if cam2.is_opened:
            if not cam2.frame_queue:
                color2, _ , _, _= cam2.frame_queue.popleft()
                if color2 is not None:
                    color2_crop = color2;
                    cv2.imshow("Camera 2 - L515", color2_crop)
        else:
            if cv2.getWindowProperty("Camera 2 - L515", cv2.WND_PROP_VISIBLE) < 1:
                cv2.imshow("Camera 2 - L515", black_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cam1.release()
    cam2.release()
    cv2.destroyAllWindows()
