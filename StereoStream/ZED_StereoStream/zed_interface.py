#!/usr/bin/env python3
"""
ZED Camera Manager - Python Interface

Provides two methods of control:
1. TCP Control: Send commands via TCP socket (streaming, recording, etc.)
2. Shared Memory: Direct frame access for real-time processing

Author: Daekwan Ko (kodaekwan)
Interactive Robotics Lab, Dongguk University
"""

import socket
import json
import time
import ctypes
import mmap
import numpy as np
import cv2
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass

# ============================================
# Constants (must match C++ header)
# ============================================
SHM_NAME_STATUS = "/zed_status"
SHM_NAME_FRAME = "/zed_frame"
SHM_NAME_COMMAND = "/zed_command"

MAX_CAMERAS = 4
SERIAL_NUMBER_LEN = 32
CAMERA_NAME_LEN = 64
PATH_LEN = 256
QR_DATA_LEN = 512
TIMESTAMP_LEN = 64
IP_ADDR_LEN = 32

MAX_FRAME_WIDTH = 2560
MAX_FRAME_HEIGHT = 720
MAX_FRAME_CHANNELS = 3

# ============================================
# Enums
# ============================================
class CommandType:
    CMD_NONE = 0
    CMD_START_PREVIEW = 1
    CMD_STOP_PREVIEW = 2
    CMD_START_RECORDING = 3
    CMD_STOP_RECORDING = 4
    CMD_START_STREAMING = 5
    CMD_STOP_STREAMING = 6
    CMD_SET_RESOLUTION = 7
    CMD_SET_FPS = 8
    CMD_SET_STREAM_PARAMS = 9
    CMD_SET_STEREO_PARAMS = 10
    CMD_SHUTDOWN = 11

class CameraState:
    STATE_DISCONNECTED = 0
    STATE_CONNECTED = 1
    STATE_PREVIEW = 2
    STATE_STREAMING = 3
    STATE_RECORDING = 4
    STATE_STREAMING_RECORDING = 5
    STATE_ERROR = 6

# ============================================
# CTypes Structures (must match C++ structs)
# ============================================
class StreamConfigCType(ctypes.Structure):
    _fields_ = [
        ("target_ip", ctypes.c_char * IP_ADDR_LEN),
        ("target_port", ctypes.c_uint16),
        ("width", ctypes.c_uint16),
        ("height", ctypes.c_uint16),
        ("max_payload", ctypes.c_uint16),
        ("jpeg_quality", ctypes.c_uint8),
        ("fps", ctypes.c_uint8),
        ("streaming_enabled", ctypes.c_bool),
    ]

class CameraStatusCType(ctypes.Structure):
    _fields_ = [
        ("serial_number", ctypes.c_char * SERIAL_NUMBER_LEN),
        ("camera_name", ctypes.c_char * CAMERA_NAME_LEN),
        ("state", ctypes.c_uint8),
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("fps", ctypes.c_uint32),
        ("is_connected", ctypes.c_bool),
        ("is_previewing", ctypes.c_bool),
        ("is_recording", ctypes.c_bool),
        ("is_streaming", ctypes.c_bool),
        ("camera_timestamp_us", ctypes.c_int64),
        ("system_timestamp_us", ctypes.c_int64),
        ("global_time_str", ctypes.c_char * TIMESTAMP_LEN),
        ("qr_detected", ctypes.c_bool),
        ("qr_data", ctypes.c_char * QR_DATA_LEN),
        ("qr_timestamp_us", ctypes.c_int64),
        ("qr_pc_time", ctypes.c_char * TIMESTAMP_LEN),
        ("qr_robot_time", ctypes.c_char * TIMESTAMP_LEN),
        ("stream_config", StreamConfigCType),
        ("recording_path", ctypes.c_char * PATH_LEN),
        ("frames_recorded", ctypes.c_uint64),
    ]

class GlobalStatusCType(ctypes.Structure):
    _fields_ = [
        ("update_counter", ctypes.c_uint64),
        ("num_cameras", ctypes.c_uint32),
        ("cameras", CameraStatusCType * MAX_CAMERAS),
        ("last_error", ctypes.c_char * 256),
        ("manager_running", ctypes.c_bool),
        ("control_port", ctypes.c_uint16),
    ]

class FrameHeaderCType(ctypes.Structure):
    _fields_ = [
        ("frame_counter", ctypes.c_uint64),
        ("camera_index", ctypes.c_uint32),
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("channels", ctypes.c_uint32),
        ("stride", ctypes.c_uint32),
        ("camera_timestamp_us", ctypes.c_int64),
        ("system_timestamp_us", ctypes.c_int64),
        ("global_time_str", ctypes.c_char * TIMESTAMP_LEN),
        ("qr_detected", ctypes.c_bool),
        ("qr_pc_time", ctypes.c_char * TIMESTAMP_LEN),
        ("qr_robot_time", ctypes.c_char * TIMESTAMP_LEN),
        ("last_qr_pc_timestamp_us", ctypes.c_int64),
        ("last_qr_robot_timestamp_us", ctypes.c_int64),
        ("last_qr_cam_timestamp_us", ctypes.c_int64),
        ("last_qr_pc_time", ctypes.c_char * TIMESTAMP_LEN),
        ("last_qr_robot_time", ctypes.c_char * TIMESTAMP_LEN),
    ]

# Calculate sizes
FRAME_HEADER_SIZE = ctypes.sizeof(FrameHeaderCType)
FRAME_DATA_SIZE = MAX_FRAME_WIDTH * MAX_FRAME_HEIGHT * MAX_FRAME_CHANNELS
FRAME_SHM_SIZE = FRAME_HEADER_SIZE + FRAME_DATA_SIZE

# ============================================
# Data Classes
# ============================================
@dataclass
class FrameData:
    """Container for frame data from shared memory"""
    valid: bool = False
    frame: Optional[np.ndarray] = None
    frame_counter: int = 0
    camera_index: int = 0
    camera_timestamp_us: int = 0
    system_timestamp_us: int = 0
    global_time_str: str = ""
    qr_detected: bool = False
    qr_pc_time: str = ""
    qr_robot_time: str = ""
    last_qr_pc_timestamp_us: int = 0
    last_qr_robot_timestamp_us: int = 0
    last_qr_cam_timestamp_us: int = 0
    last_qr_pc_time: str = ""
    last_qr_robot_time: str = ""
    
    def get_interpolated_robot_time_us(self) -> int:
        """Calculate interpolated robot time using system clock (microseconds)"""
        if self.last_qr_robot_timestamp_us > 0 and self.last_qr_pc_timestamp_us > 0:
            delta = self.system_timestamp_us - self.last_qr_pc_timestamp_us
            return self.last_qr_robot_timestamp_us + delta
        return 0
    
    def get_interpolated_robot_time_cam_us(self) -> int:
        """Calculate interpolated robot time using camera HW clock (microseconds)"""
        if self.last_qr_robot_timestamp_us > 0 and self.last_qr_cam_timestamp_us > 0:
            delta = self.camera_timestamp_us - self.last_qr_cam_timestamp_us
            return self.last_qr_robot_timestamp_us + delta
        return 0
    
    def get_sync_delta_us(self) -> int:
        """Get time since last QR sync (microseconds)"""
        if self.last_qr_pc_timestamp_us > 0:
            return self.system_timestamp_us - self.last_qr_pc_timestamp_us
        return -1

# ============================================
# TCP Controller Class
# ============================================
class ZedController:
    """
    TCP-based controller for ZED Camera Manager
    
    Usage:
        controller = ZedController("localhost", 12345)
        controller.start_stream("192.168.0.140", port=9003, quality=50)
        controller.start_record(path="./recordings", filename="video")
        status = controller.get_status()
        controller.stop_record()
        controller.stop_stream()
        controller.quit()
    """
    
    def __init__(self, host: str = "localhost", port: int = 0):
        self.host = host
        self.port = port
        self.timeout = 5.0
    
    def _send_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """Send TCP command and receive response"""
        try:
            with socket.create_connection((self.host, self.port), timeout=self.timeout) as sock:
                cmd_str = json.dumps(command)
                sock.sendall(cmd_str.encode('utf-8'))
                sock.shutdown(socket.SHUT_WR)
                
                data = b""
                while True:
                    chunk = sock.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                
                resp_str = data.decode('utf-8', errors='ignore').strip()
                if resp_str:
                    return json.loads(resp_str)
                return {"status": "error", "message": "Empty response"}
                
        except socket.timeout:
            return {"status": "error", "message": "Connection timeout"}
        except ConnectionRefusedError:
            return {"status": "error", "message": "Connection refused"}
        except Exception as e:
            return {"status": "error", "message": str(e)}
    
    def start_stream(self, ip: str, port: int = 9003, quality: int = 50,
                     width: int = 640, height: int = 480) -> Dict[str, Any]:
        """Start UDP streaming to XR device"""
        return self._send_command({
            "action": "start_stream",
            "ip": ip,
            "port": port,
            "quality": quality,
            "width": width,
            "height": height
        })
    
    def stop_stream(self) -> Dict[str, Any]:
        """Stop UDP streaming"""
        return self._send_command({"action": "stop_stream"})
    
    def start_record(self, path: str = "./recordings", filename: str = "recording") -> Dict[str, Any]:
        """Start video recording with metadata CSV"""
        return self._send_command({
            "action": "start_record",
            "path": path,
            "filename": filename
        })
    
    def stop_record(self) -> Dict[str, Any]:
        """Stop video recording"""
        return self._send_command({"action": "stop_record"})
    
    def start_preview(self) -> Dict[str, Any]:
        """Start preview window"""
        return self._send_command({"action": "start_preview"})
    
    def stop_preview(self) -> Dict[str, Any]:
        """Stop preview window"""
        return self._send_command({"action": "stop_preview"})
    
    def get_status(self) -> Dict[str, Any]:
        """Get current status"""
        return self._send_command({"action": "get_status"})
    
    def set_stereo_params(self, target_ip: str, target_port: int = 9004,
                          focus: Optional[float] = None,
                          quad: Optional[float] = None,
                          zoom: Optional[float] = None,
                          add_focus: Optional[bool] = None) -> Dict[str, Any]:
        """Send stereo parameters to XR device"""
        cmd = {
            "action": "set_stereo_params",
            "target_ip": target_ip,
            "target_port": target_port
        }
        if focus is not None:
            cmd["focus"] = focus
        if quad is not None:
            cmd["quad"] = quad
        if zoom is not None:
            cmd["zoom"] = zoom
        if add_focus is not None:
            cmd["add_focus"] = add_focus
        return self._send_command(cmd)
    
    def quit(self) -> Dict[str, Any]:
        """Shutdown the camera manager"""
        return self._send_command({"action": "quit"})
    
    @property
    def is_streaming(self) -> bool:
        status = self.get_status()
        return status.get("streaming") == "true" or status.get("streaming") is True
    
    @property
    def is_recording(self) -> bool:
        status = self.get_status()
        return status.get("recording") == "true" or status.get("recording") is True


# ============================================
# Shared Memory Interface Class
# ============================================
class ZedInterface:
    """
    Shared Memory interface for real-time frame access
    
    Usage:
        interface = ZedInterface()
        interface.connect()
        
        while running:
            frame_data = interface.get_frame(wait_new=True)
            if frame_data.valid:
                cv2.imshow("ZED", frame_data.frame)
        
        interface.disconnect()
    """
    
    def __init__(self):
        self.status_shm = None
        self.frame_shm = None
        self.command_shm = None
        self.status_mmap = None
        self.frame_mmap = None
        self.command_mmap = None
        self.connected = False
        self.last_frame_counter = 0
        self.actual_frame_shm_size = 0
        self.actual_header_size = FRAME_HEADER_SIZE  # Will be updated on connect
    
    def connect(self) -> bool:
        """Connect to shared memory"""
        try:
            import posix_ipc
            import os
            
            # Open status shared memory
            try:
                self.status_shm = posix_ipc.SharedMemory(SHM_NAME_STATUS)
            except posix_ipc.ExistentialError:
                print(f"[ZedInterface] Shared memory '{SHM_NAME_STATUS}' not found")
                print("[ZedInterface] Make sure zed_camera_manager is running first!")
                return False
            
            # Get actual file size
            status_size = os.fstat(self.status_shm.fd).st_size
            print(f"[ZedInterface] Status SHM size: {status_size} bytes (Python struct: {ctypes.sizeof(GlobalStatusCType)})")
            self.status_mmap = mmap.mmap(self.status_shm.fd, status_size)
            
            # Open frame shared memory
            try:
                self.frame_shm = posix_ipc.SharedMemory(SHM_NAME_FRAME)
            except posix_ipc.ExistentialError:
                print(f"[ZedInterface] Shared memory '{SHM_NAME_FRAME}' not found")
                return False
            
            # Get actual file size and use it
            frame_size = os.fstat(self.frame_shm.fd).st_size
            print(f"[ZedInterface] Frame SHM size: {frame_size} bytes (Python expects: {FRAME_SHM_SIZE})")
            print(f"[ZedInterface] Python FrameHeader size: {FRAME_HEADER_SIZE} bytes")
            self.frame_mmap = mmap.mmap(self.frame_shm.fd, frame_size)
            self.actual_frame_shm_size = frame_size
            
            # Open command shared memory
            try:
                self.command_shm = posix_ipc.SharedMemory(SHM_NAME_COMMAND)
            except posix_ipc.ExistentialError:
                print(f"[ZedInterface] Shared memory '{SHM_NAME_COMMAND}' not found")
                return False
            cmd_size = os.fstat(self.command_shm.fd).st_size
            self.command_mmap = mmap.mmap(self.command_shm.fd, cmd_size)
            
            # Calculate actual header size from C++ (may differ from Python struct due to padding)
            self.actual_header_size = frame_size - FRAME_DATA_SIZE
            print(f"[ZedInterface] Actual C++ header size: {self.actual_header_size} bytes")
            
            self.connected = True
            print(f"[ZedInterface] Connected to shared memory")
            return True
        
        except ImportError:
            print("[ZedInterface] posix_ipc not installed. Run: pip install posix_ipc")
            return False
        except Exception as e:
            print(f"[ZedInterface] Failed to connect: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def disconnect(self):
        """Disconnect from shared memory"""
        if self.status_mmap:
            self.status_mmap.close()
        if self.frame_mmap:
            self.frame_mmap.close()
        if self.command_mmap:
            self.command_mmap.close()
        if self.status_shm:
            self.status_shm.close_fd()
        if self.frame_shm:
            self.frame_shm.close_fd()
        if self.command_shm:
            self.command_shm.close_fd()
        
        self.connected = False
        print("[ZedInterface] Disconnected")
    
    def get_status(self) -> Optional[GlobalStatusCType]:
        """Get global status from shared memory"""
        if not self.connected or not self.status_mmap:
            return None
        
        self.status_mmap.seek(0)
        data = self.status_mmap.read(ctypes.sizeof(GlobalStatusCType))
        return GlobalStatusCType.from_buffer_copy(data)
    
    def get_frame(self, wait_new: bool = False, timeout_ms: int = 100) -> FrameData:
        """Get frame from shared memory"""
        result = FrameData()
        
        if not self.connected or not self.frame_mmap:
            return result
        
        start_time = time.time()
        
        # Use actual C++ header size for correct offset
        header_size = self.actual_header_size
        
        while True:
            self.frame_mmap.seek(0)
            header_data = self.frame_mmap.read(min(header_size, FRAME_HEADER_SIZE))
            header = FrameHeaderCType.from_buffer_copy(header_data.ljust(FRAME_HEADER_SIZE, b'\x00'))
            
            if wait_new and header.frame_counter == self.last_frame_counter:
                if (time.time() - start_time) * 1000 > timeout_ms:
                    return result
                time.sleep(0.001)
                continue
            
            self.last_frame_counter = header.frame_counter
            
            if header.width == 0 or header.height == 0:
                return result
            
            # Read frame data using actual C++ header size as offset
            frame_size = header.width * header.height * header.channels
            self.frame_mmap.seek(header_size)
            frame_data = self.frame_mmap.read(frame_size)
            
            # Convert to numpy array
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((header.height, header.width, header.channels))
            
            result.valid = True
            result.frame = frame.copy()
            result.frame_counter = header.frame_counter
            result.camera_index = header.camera_index
            result.camera_timestamp_us = header.camera_timestamp_us
            result.system_timestamp_us = header.system_timestamp_us
            result.global_time_str = header.global_time_str.decode('utf-8', errors='ignore')
            result.qr_detected = header.qr_detected
            result.qr_pc_time = header.qr_pc_time.decode('utf-8', errors='ignore')
            result.qr_robot_time = header.qr_robot_time.decode('utf-8', errors='ignore')
            result.last_qr_pc_timestamp_us = header.last_qr_pc_timestamp_us
            result.last_qr_robot_timestamp_us = header.last_qr_robot_timestamp_us
            result.last_qr_cam_timestamp_us = header.last_qr_cam_timestamp_us
            result.last_qr_pc_time = header.last_qr_pc_time.decode('utf-8', errors='ignore')
            result.last_qr_robot_time = header.last_qr_robot_time.decode('utf-8', errors='ignore')
            
            return result
    
    def get_interpolated_robot_time_us(self) -> int:
        """Get current interpolated robot time (system clock based)"""
        frame = self.get_frame(wait_new=False)
        return frame.get_interpolated_robot_time_us() if frame.valid else 0
    
    def get_time_drift_us(self) -> int:
        """Get time drift between consecutive QR detections"""
        # This requires storing previous QR times - simplified version
        frame = self.get_frame(wait_new=False)
        return frame.get_sync_delta_us() if frame.valid else -1
    
    # Command shortcuts (via shared memory)
    #
    # C++ Command struct memory layout:
    #   offset  0: command_counter  (uint64, 8 bytes)
    #   offset  8: last_processed   (uint64, 8 bytes)
    #   offset 16: type             (uint8,  1 byte)
    #   offset 20: camera_index     (uint32, 4 bytes)  -- aligned
    #   offset 24: params union     (variable)
    #     recording: path[256] + filename[64]
    #     streaming: target_ip[32] + port(u16) + width(u16) + height(u16) + quality(u8)
    #
    COMMAND_PARAMS_OFFSET = 24  # Start of params union in Command struct

    def _send_shm_command(self, cmd_type: int, **kwargs):
        """Send command via shared memory, including parameter data"""
        if not self.connected or not self.command_mmap:
            return False
        
        # Read current counter
        self.command_mmap.seek(0)
        counter = int.from_bytes(self.command_mmap.read(8), 'little')
        
        # Write command type
        self.command_mmap.seek(16)  # offset of 'type'
        self.command_mmap.write(bytes([cmd_type]))
        
        # Write parameters into the params union at offset 24
        params_offset = self.COMMAND_PARAMS_OFFSET
        
        if cmd_type == CommandType.CMD_START_RECORDING:
            # recording struct: path[PATH_LEN=256] + filename[CAMERA_NAME_LEN=64]
            path = kwargs.get('path', '').encode('utf-8')
            filename = kwargs.get('filename', '').encode('utf-8')
            # Pad/truncate to fixed sizes
            path_padded = path[:PATH_LEN].ljust(PATH_LEN, b'\x00')
            filename_padded = filename[:CAMERA_NAME_LEN].ljust(CAMERA_NAME_LEN, b'\x00')
            self.command_mmap.seek(params_offset)
            self.command_mmap.write(path_padded)
            self.command_mmap.write(filename_padded)
        elif cmd_type == CommandType.CMD_START_STREAMING:
            # streaming struct: target_ip[32] + port(u16) + width(u16) + height(u16) + quality(u8)
            ip = kwargs.get('ip', '').encode('utf-8')
            ip_padded = ip[:IP_ADDR_LEN].ljust(IP_ADDR_LEN, b'\x00')
            port = kwargs.get('port', 9003)
            width = kwargs.get('width', 640)
            height = kwargs.get('height', 480)
            quality = kwargs.get('quality', 50)
            self.command_mmap.seek(params_offset)
            self.command_mmap.write(ip_padded)
            self.command_mmap.write(port.to_bytes(2, 'little'))
            self.command_mmap.write(width.to_bytes(2, 'little'))
            self.command_mmap.write(height.to_bytes(2, 'little'))
            self.command_mmap.write(bytes([quality]))
        
        # Increment counter last (acts as a memory fence for the reader)
        self.command_mmap.seek(0)
        new_counter = counter + 1
        self.command_mmap.write(new_counter.to_bytes(8, 'little'))
        
        return True
    
    def start_preview(self) -> bool:
        return self._send_shm_command(CommandType.CMD_START_PREVIEW)
    
    def stop_preview(self) -> bool:
        return self._send_shm_command(CommandType.CMD_STOP_PREVIEW)
    
    def start_recording(self, path: str = "./recordings", filename: str = "") -> bool:
        return self._send_shm_command(CommandType.CMD_START_RECORDING, path=path, filename=filename)
    
    def stop_recording(self) -> bool:
        return self._send_shm_command(CommandType.CMD_STOP_RECORDING)
    
    def shutdown(self) -> bool:
        return self._send_shm_command(CommandType.CMD_SHUTDOWN)


# ============================================
# Combined Interface (TCP + Shared Memory)
# ============================================
class ZedCameraInterface:
    """
    Combined interface providing both TCP control and shared memory access
    
    Usage:
        zed = ZedCameraInterface()
        zed.connect(host="localhost", port=12345)
        
        # TCP control
        zed.start_stream("192.168.0.140")
        zed.start_record()
        
        # Shared memory frame access
        while running:
            frame_data = zed.get_frame(wait_new=True)
            if frame_data.valid:
                robot_time = frame_data.get_interpolated_robot_time_us()
                cv2.imshow("ZED", frame_data.frame)
        
        zed.disconnect()
    """
    
    def __init__(self):
        self.controller = None
        self.shm_interface = ZedInterface()
        self.host = "localhost"
        self.port = 0
    
    def connect(self, host: str = "localhost", port: int = 0, use_shm: bool = True) -> bool:
        """Connect to camera manager"""
        self.host = host
        self.port = port
        
        if port > 0:
            self.controller = ZedController(host, port)
        
        if use_shm:
            if not self.shm_interface.connect():
                print("[Warning] Shared memory not available, using TCP only")
                return self.controller is not None
        
        return True
    
    def disconnect(self):
        """Disconnect from camera manager"""
        self.shm_interface.disconnect()
    
    # TCP commands
    def start_stream(self, ip: str, port: int = 9003, quality: int = 50,
                     width: int = 640, height: int = 480) -> Dict[str, Any]:
        if self.controller:
            return self.controller.start_stream(ip, port, quality, width, height)
        return {"status": "error", "message": "Not connected via TCP"}
    
    def stop_stream(self) -> Dict[str, Any]:
        if self.controller:
            return self.controller.stop_stream()
        return {"status": "error", "message": "Not connected via TCP"}
    
    def start_record(self, path: str = "./recordings", filename: str = "recording") -> Dict[str, Any]:
        if self.controller:
            return self.controller.start_record(path, filename)
        return {"status": "error", "message": "Not connected via TCP"}
    
    def stop_record(self) -> Dict[str, Any]:
        if self.controller:
            return self.controller.stop_record()
        return {"status": "error", "message": "Not connected via TCP"}
    
    def get_status(self) -> Dict[str, Any]:
        if self.controller:
            return self.controller.get_status()
        return {"status": "error", "message": "Not connected via TCP"}
    
    def set_stereo_params(self, target_ip: str, **kwargs) -> Dict[str, Any]:
        if self.controller:
            return self.controller.set_stereo_params(target_ip, **kwargs)
        return {"status": "error", "message": "Not connected via TCP"}
    
    def quit(self) -> Dict[str, Any]:
        if self.controller:
            return self.controller.quit()
        return {"status": "error", "message": "Not connected via TCP"}
    
    # Shared memory access
    def get_frame(self, wait_new: bool = False, timeout_ms: int = 100) -> FrameData:
        return self.shm_interface.get_frame(wait_new, timeout_ms)
    
    def get_shm_status(self) -> Optional[GlobalStatusCType]:
        return self.shm_interface.get_status()
    
    def get_interpolated_robot_time_us(self) -> int:
        return self.shm_interface.get_interpolated_robot_time_us()


# ============================================
# Main (Test/Demo)
# ============================================
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="ZED Camera Interface")
    parser.add_argument("--host", default="localhost", help="TCP host")
    parser.add_argument("--port", type=int, default=0, help="TCP port (0 for shared memory only)")
    parser.add_argument("--action", choices=["status", "record", "stream", "view"], 
                        default="view", help="Action to perform")
    parser.add_argument("--ip", default="192.168.0.140", help="Stream target IP")
    args = parser.parse_args()
    
    if args.action == "view":
        # View frames from shared memory
        interface = ZedInterface()
        if not interface.connect():
            print("Failed to connect to shared memory")
            print("Make sure zed_camera_manager is running")
            exit(1)
        
        # Get control port from shared memory status for TCP commands
        status = interface.get_status()
        controller = None
        streaming = False
        if status and status.control_port > 0:
            controller = ZedController("localhost", status.control_port)
            print(f"[TCP] Connected to control server on port {status.control_port}")
            # Check initial streaming status
            tcp_status = controller.get_status()
            streaming = tcp_status.get("streaming") == "true"
        
        print("\nControls:")
        print("  q - Quit")
        print("  p - Toggle preview")
        print("  r - Toggle recording")
        print("  s - Toggle XR streaming")
        print("  c - Cycle color mode")
        
        color_modes = ["Original (BGR)", "RGB2BGR", "BGR2RGB"]
        color_mode = 0
        recording = False
        rcount = 0
        
        try:
            while True:
                frame_data = interface.get_frame(wait_new=True, timeout_ms=100)
                
                if frame_data.valid:
                    display = frame_data.frame.copy()
                    
                    # Apply color conversion
                    if color_mode == 1:
                        display = cv2.cvtColor(display, cv2.COLOR_RGB2BGR)
                    elif color_mode == 2:
                        display = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
                    
                    # Draw info
                    info_lines = [
                        f"Frame: {frame_data.frame_counter}",
                        f"Camera TS: {frame_data.camera_timestamp_us // 1000} ms",
                        f"System: {frame_data.global_time_str}",
                    ]
                    
                    # Show streaming/recording status
                    status_str = ""
                    if streaming:
                        status_str += "[STREAM] "
                    if recording:
                        status_str += "[REC] "
                    if status_str:
                        info_lines.append(status_str.strip())
                    
                    if frame_data.last_qr_robot_timestamp_us > 0:
                        robot_us = frame_data.get_interpolated_robot_time_us()
                        delta_ms = frame_data.get_sync_delta_us() / 1000
                        info_lines.append(f"Robot (interp): {robot_us // 1000} ms")
                        info_lines.append(f"Sync delta: {delta_ms:.1f} ms")
                    
                    if frame_data.qr_detected:
                        info_lines.append(f"QR: {frame_data.qr_robot_time}")
                    
                    y = 30
                    for line in info_lines:
                        cv2.putText(display, line, (10, y),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        y += 25
                    
                    # Color mode indicator
                    cv2.putText(display, f"Color: {color_modes[color_mode]} (press 'c')",
                               (10, display.shape[0] - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    # Resize for display
                    if display.shape[1] > 1280:
                        scale = 1280 / display.shape[1]
                        display = cv2.resize(display, None, fx=scale, fy=scale)
                    
                    cv2.imshow("ZED Interface - Shared Memory", display)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('c'):
                    color_mode = (color_mode + 1) % 3
                    print(f"Color mode: {color_modes[color_mode]}")
                elif key == ord('p'):
                    interface.start_preview()
                    print("Preview command sent")
                elif key == ord('r'):
                    if not recording:
                        interface.start_recording()
                        recording = True
                        print("Recording started!")
                    else:
                        interface.stop_recording()
                        recording = False
                        print("Recording stopped")
                elif key == ord('s'):
                    if controller:
                        if not streaming:
                            result = controller.start_stream(args.ip)
                            print(f"Stream result: {result}")
                            # start_stream already sends stereo params automatically
                            # Manual call for testing:
                            # result = controller.set_stereo_params(args.ip, focus=1.0, quad=0.5, zoom=1.0, add_focus=False)
                            # print(f"Stereo params result: {result}")
                            streaming = True
                            print(f"Streaming started to {args.ip}")
                        else:
                            controller.stop_stream()
                            streaming = False
                            print("Streaming stopped")
                    else:
                        print("No TCP controller available (need --port)")
        
        except KeyboardInterrupt:
            print("\nInterrupted")
        
        finally:
            interface.disconnect()
            cv2.destroyAllWindows()
    
    elif args.action == "status":
        if args.port > 0:
            controller = ZedController(args.host, args.port)
            print(json.dumps(controller.get_status(), indent=2))
        else:
            interface = ZedInterface()
            if interface.connect():
                status = interface.get_status()
                if status:
                    print(f"Manager running: {status.manager_running}")
                    print(f"Control port: {status.control_port}")
                    print(f"Cameras: {status.num_cameras}")
                    for i in range(status.num_cameras):
                        cam = status.cameras[i]
                        print(f"  Camera {i}: {cam.camera_name.decode()}")
                        print(f"    Serial: {cam.serial_number.decode()}")
                        print(f"    Resolution: {cam.width}x{cam.height} @ {cam.fps}fps")
                        print(f"    Streaming: {cam.is_streaming}")
                        print(f"    Recording: {cam.is_recording}")
                interface.disconnect()
    
    elif args.action == "stream":
        if args.port == 0:
            print("TCP port required for streaming control")
            exit(1)
        controller = ZedController(args.host, args.port)
        result = controller.start_stream(args.ip)
        print(json.dumps(result, indent=2))
    
    elif args.action == "record":
        if args.port == 0:
            print("TCP port required for recording control")
            exit(1)
        controller = ZedController(args.host, args.port)
        result = controller.start_record()
        print(json.dumps(result, indent=2))
