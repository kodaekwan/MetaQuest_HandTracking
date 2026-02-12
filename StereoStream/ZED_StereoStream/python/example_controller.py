"""
VisionPro 스트리밍 & 녹화 예제
C++ 서버를 먼저 실행한 후 이 스크립트를 실행하세요.

사용법:
    1. C++ 서버 실행:
       cd /usr/local/zed/samples/camera\ control/cpp/build
       ./ZED_VisionPro_Stream --port 12345
       
    2. 파이썬 클라이언트 실행:
       python3 example_controller.py
"""

import time
from visionpro_controller import VisionProController


def main():
    # C++ 서버에 연결 (포트 번호는 서버 실행 시 출력됨)
    CONTROL_PORT = 12345  # 서버 실행 시 표시된 포트로 변경
    controller = VisionProController("localhost", CONTROL_PORT)
    
    print("=== VisionPro 스트림 컨트롤러 예제 ===\n")
    
    # 1. 상태 확인
    print("1. 현재 상태 확인...")
    status = controller.get_status()
    print(f"   상태: {status}")
    
    # 2. 스트리밍 시작
    print("\n2. UDP 스트리밍 시작...")
    result = controller.start_stream(
        ip="192.168.0.140",      # VisionPro IP
        port=9003,               # UDP 포트
        quality=50,              # JPEG 품질
        width=640,               # 스트림 너비
        height=480               # 스트림 높이
    )
    print(f"   결과: {result}")
    
    # 3. 스트리밍 상태 확인
    print("\n3. 스트리밍 상태 확인...")
    print(f"   스트리밍 중: {controller.is_streaming}")
    print(f"   현재 상태: {controller.state}")
    
    # 4. 녹화 시작 (원본 품질)
    print("\n4. 영상 녹화 시작...")
    result = controller.start_record(
        path="./recordings",     # 저장 폴더
        filename="test_video"    # 파일명 (같은 이름 있으면 _1, _2 등 자동 추가)
    )
    print(f"   결과: {result}")
    if result.get("status") == "ok":
        print(f"   저장 경로: {result.get('filepath')}")
    
    # 5. 잠시 녹화
    print("\n5. 5초간 녹화 중...")
    time.sleep(5)
    
    # 6. 녹화 중지
    print("\n6. 녹화 중지...")
    result = controller.stop_record()
    print(f"   결과: {result}")
    
    # 7. 스트리밍 중지
    print("\n7. 스트리밍 중지...")
    result = controller.stop_stream()
    print(f"   결과: {result}")
    
    # 8. 최종 상태
    print("\n8. 최종 상태...")
    status = controller.get_status()
    print(f"   상태: {status}")
    
    print("\n=== 완료 ===")


def interactive_mode():
    """대화형 모드"""
    print("VisionPro 컨트롤러 - 대화형 모드")
    print("포트 번호를 입력하세요 (C++ 서버 실행 시 표시됨):")
    
    try:
        port = int(input("Port: "))
    except ValueError:
        print("유효하지 않은 포트 번호")
        return
    
    controller = VisionProController("localhost", port)
    
    while True:
        print("\n명령어:")
        print("  1. 상태 확인")
        print("  2. 스트리밍 시작")
        print("  3. 스트리밍 중지")
        print("  4. 녹화 시작")
        print("  5. 녹화 중지")
        print("  6. 스테레오 파라미터 설정")
        print("  7. 프로그램 종료")
        print("  q. 나가기")
        
        cmd = input("\n선택: ").strip()
        
        if cmd == "1":
            result = controller.get_status()
            print(f"상태: {result}")
            
        elif cmd == "2":
            ip = input("대상 IP(기본 192.168.0.140): ").strip()
            port_str = input("UDP 포트 (기본 9003): ").strip()
            quality_str = input("JPEG 품질 1-100 (기본 50): ").strip()
            
            ip = ip if ip else "192.168.0.140"
            port = int(port_str) if port_str else 9003
            quality = int(quality_str) if quality_str else 50
            
            result = controller.start_stream(ip=ip, port=port, quality=quality)
            print(f"결과: {result}")
            
        elif cmd == "3":
            result = controller.stop_stream()
            print(f"결과: {result}")
            
        elif cmd == "4":
            path = input("저장 폴더 (기본 .): ").strip() or "."
            filename = input("파일명 (기본 recording): ").strip() or "recording"
            
            result = controller.start_record(path=path, filename=filename)
            print(f"결과: {result}")
            
        elif cmd == "5":
            result = controller.stop_record()
            print(f"결과: {result}")
            
        elif cmd == "6":
            target_ip = input("대상 기기 IP(기본 192.168.0.140): ").strip()
            target_port_str = input("대상 포트 (기본 9004): ").strip()
            focus_str = input("focus (예: 1.0, 스킵하려면 Enter): ").strip()
            quad_str = input("quad (예: 1.1, 스킵하려면 Enter): ").strip()
            zoom_str = input("zoom (예: 1.0, 스킵하려면 Enter): ").strip()
            add_focus_str = input("add_focus (y/n, 스킵하려면 Enter): ").strip()
            
            target_ip = target_ip if target_ip else "192.168.0.140"
            target_port = int(target_port_str) if target_port_str else 9004
            focus = float(focus_str) if focus_str else None
            quad = float(quad_str) if quad_str else None
            zoom = float(zoom_str) if zoom_str else None
            add_focus = (add_focus_str.lower() == 'y') if add_focus_str else None
            
            result = controller.set_stereo_params(
                target_ip=target_ip,
                target_port=target_port,
                focus=focus,
                quad=quad,
                zoom=zoom,
                add_focus=add_focus
            )
            print(f"결과: {result}")
            
        elif cmd == "7":
            result = controller.quit()
            print(f"결과: {result}")
            break
            
        elif cmd.lower() == "q":
            break


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--interactive", "-i", action="store_true", help="대화형 모드")
    args = parser.parse_args()
    
    if args.interactive:
        interactive_mode()
    else:
        main()
