#!/usr/bin/env python3
"""
VisionPro Stream Controller
Python client for controlling ZED_VisionPro_Stream C++ application
"""

import socket
import json
import time
from typing import Optional, Dict, Any


class VisionProController:
    """
    VisionPro 스트리밍 서버 제어 클라이언트
    
    사용법:
        controller = VisionProController("localhost", 12345)
        
        # 스트리밍 시작
        controller.start_stream("192.168.0.100", port=9003, quality=50)
        
        # 녹화 시작
        controller.start_record(path="./videos", filename="test")
        
        # 상태 확인
        status = controller.get_status()
        print(status)
        
        # 녹화 중지
        controller.stop_record()
        
        # 스트리밍 중지
        controller.stop_stream()
        
        # 프로그램 종료
        controller.quit()
    """
    
    def __init__(self, host: str = "localhost", port: int = 0):
        """
        Args:
            host: C++ 서버 호스트 주소
            port: C++ 제어 서버 포트 번호
        """
        self.host = host
        self.port = port
        self.timeout = 5.0
    
    def _send_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """TCP로 명령 전송 후 응답 수신"""
        try:
            with socket.create_connection((self.host, self.port), timeout=self.timeout) as sock:
                # 명령 전송
                cmd_str = json.dumps(command)
                sock.sendall(cmd_str.encode('utf-8'))
                sock.shutdown(socket.SHUT_WR)
                
                # 응답 수신
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
        """
        UDP 스트리밍 시작
        
        Args:
            ip: 스트리밍 대상 IP 주소 (예: VisionPro 또는 Quest)
            port: UDP 포트 (기본: 9003)
            quality: JPEG 품질 1-100 (기본: 50)
            width: 스트리밍 이미지 너비 (기본: 640)
            height: 스트리밍 이미지 높이 (기본: 480)
            
        Returns:
            서버 응답 딕셔너리
        """
        return self._send_command({
            "action": "start_stream",
            "ip": ip,
            "port": port,
            "quality": quality,
            "width": width,
            "height": height
        })
    
    def stop_stream(self) -> Dict[str, Any]:
        """UDP 스트리밍 중지"""
        return self._send_command({"action": "stop_stream"})
    
    def start_record(self, path: str = ".", filename: str = "recording") -> Dict[str, Any]:
        """
        영상 녹화 시작
        
        Args:
            path: 저장 폴더 경로 (기본: 현재 폴더)
            filename: 파일명 (확장자 제외, 기본: recording)
                     같은 이름이 있으면 자동으로 _1, _2 등 추가
                     
        Returns:
            서버 응답 딕셔너리 (filepath 포함)
        """
        return self._send_command({
            "action": "start_record",
            "path": path,
            "filename": filename
        })
    
    def stop_record(self) -> Dict[str, Any]:
        """영상 녹화 중지"""
        return self._send_command({"action": "stop_record"})
    
    def get_status(self) -> Dict[str, Any]:
        """
        현재 상태 조회
        
        Returns:
            {
                "status": "ok",
                "state": "idle" | "streaming" | "recording" | "streaming_recording" | "stopped",
                "streaming": true | false,
                "recording": true | false,
                "recording_file": "path/to/file.mp4" (녹화 중일 때만),
                "control_port": 12345
            }
        """
        return self._send_command({"action": "get_status"})
    
    def set_stereo_params(self, target_ip: str, target_port: int = 9004,
                          focus: Optional[float] = None,
                          quad: Optional[float] = None,
                          zoom: Optional[float] = None,
                          add_focus: Optional[bool] = None) -> Dict[str, Any]:
        """
        외부 기기(VisionPro/Quest)에 스테레오 파라미터 전송
        
        Args:
            target_ip: 외부 기기 IP 주소
            target_port: 파라미터 수신 포트 (기본: 9004)
            focus: 양안 시차 조절 (예: 0.0 ~ 1.0)
            quad: 스크린 거리/Z축 (예: 1.0 ~ 3.0)
            zoom: 화면 확대 비율 (예: 1.0 ~ 2.0)
            add_focus: 포커스 추가 여부
            
        Returns:
            서버 응답 딕셔너리 (device_response 포함)
        """
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
        """C++ 프로그램 종료"""
        return self._send_command({"action": "quit"})
    
    @property
    def is_streaming(self) -> bool:
        """스트리밍 중인지 확인"""
        status = self.get_status()
        return status.get("streaming") == "true" or status.get("streaming") is True
    
    @property
    def is_recording(self) -> bool:
        """녹화 중인지 확인"""
        status = self.get_status()
        return status.get("recording") == "true" or status.get("recording") is True
    
    @property
    def state(self) -> str:
        """현재 상태 문자열 반환"""
        status = self.get_status()
        return status.get("state", "unknown")


def main():
    """테스트 및 예제"""
    import argparse
    
    parser = argparse.ArgumentParser(description="VisionPro Stream Controller")
    parser.add_argument("--host", default="localhost", help="Server host")
    parser.add_argument("--port", type=int, required=True, help="Control server port")
    parser.add_argument("--action", required=True, 
                       choices=["start_stream", "stop_stream", "start_record", 
                               "stop_record", "status", "set_stereo_params", "quit"],
                       help="Action to perform")
    
    # Stream options
    parser.add_argument("--ip", help="Stream target IP")
    parser.add_argument("--stream-port", type=int, default=9003, help="UDP stream port")
    parser.add_argument("--quality", type=int, default=50, help="JPEG quality")
    parser.add_argument("--width", type=int, default=640, help="Stream width")
    parser.add_argument("--height", type=int, default=480, help="Stream height")
    
    # Record options
    parser.add_argument("--path", default=".", help="Recording save path")
    parser.add_argument("--filename", default="recording", help="Recording filename")
    
    # Stereo params options
    parser.add_argument("--target-ip", help="Target device IP for stereo params")
    parser.add_argument("--target-port", type=int, default=9004, help="Target device port")
    parser.add_argument("--focus", type=float, help="Focus parameter (0.0 ~ 1.0)")
    parser.add_argument("--quad", type=float, help="Quad/distance parameter (1.0 ~ 3.0)")
    parser.add_argument("--zoom", type=float, help="Zoom parameter (1.0 ~ 2.0)")
    parser.add_argument("--add-focus", action="store_true", help="Add focus flag")
    
    args = parser.parse_args()
    
    controller = VisionProController(args.host, args.port)
    
    if args.action == "start_stream":
        if not args.ip:
            print("Error: --ip required for start_stream")
            return 1
        result = controller.start_stream(
            ip=args.ip,
            port=args.stream_port,
            quality=args.quality,
            width=args.width,
            height=args.height
        )
    elif args.action == "stop_stream":
        result = controller.stop_stream()
    elif args.action == "start_record":
        result = controller.start_record(path=args.path, filename=args.filename)
    elif args.action == "stop_record":
        result = controller.stop_record()
    elif args.action == "status":
        result = controller.get_status()
    elif args.action == "set_stereo_params":
        if not args.target_ip:
            print("Error: --target-ip required for set_stereo_params")
            return 1
        result = controller.set_stereo_params(
            target_ip=args.target_ip,
            target_port=args.target_port,
            focus=args.focus,
            quad=args.quad,
            zoom=args.zoom,
            add_focus=args.add_focus if args.add_focus else None
        )
    elif args.action == "quit":
        result = controller.quit()
    else:
        result = {"status": "error", "message": "Unknown action"}
    
    print(json.dumps(result, indent=2))
    return 0 if result.get("status") == "ok" else 1


if __name__ == "__main__":
    exit(main())
