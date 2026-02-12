# ZED Stereo Stream for Apple Vision Pro (or Meta Quest)

ZED 카메라에서 스테레오 영상을 캡처하여 Apple Vision Pro(또는 Meta Quest)로 UDP 스트리밍하고, 영상을 녹화하는 시스템입니다.

## 📁 프로젝트 구조

```
ZED_StereoStream/
├── cpp/                          # C++ 서버 애플리케이션
│   ├── src/
│   │   ├── main_visionpro.cpp    # 스트리밍 서버 메인 코드
│   │   └── main.cpp              # 카메라 컨트롤 예제 (원본)
│   ├── CMakeLists.txt            # CMake 빌드 설정
│   ├── ZED_VisionPro_Stream      # 빌드된 스트리밍 서버 실행 파일
│   └── recordings/               # 녹화 파일 저장 폴더
│
├── python/                       # Python 컨트롤러 클라이언트
│   ├── visionpro_controller.py   # 컨트롤러 클래스 라이브러리
│   └── example_controller.py     # 사용 예제 스크립트
│
└── README.md                     # 이 문서
```

## 🔧 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          Docker Container                                │
│  ┌─────────────┐      ┌──────────────────────────────────────────────┐  │
│  │ ZED Camera  │─────>│ ZED_VisionPro_Stream (C++ Server)            │  │
│  └─────────────┘      │  • TCP Control Server (포트: 12345)          │  │
│                       │  • UDP Stereo Streaming                      │  │
│                       │  • Video Recording (MP4)                     │  │
│                       └──────────────────────────────────────────────┘  │
└───────────────────────────────────┬─────────────────────────────────────┘
                                    │
                          ┌─────────┴─────────┐
                          │   TCP Commands    │
                          │   UDP Stream      │
                          └─────────┬─────────┘
                                    │
        ┌───────────────────────────┼───────────────────────────┐
        ▼                           ▼                           ▼
┌───────────────────┐    ┌───────────────────┐    ┌───────────────────┐
│ Python Controller │    │ Apple Vision Pro  │    │   Meta Quest      │
│ (명령 전송)       │    │ (영상 수신)       │    │ (영상 수신)       │
└───────────────────┘    └───────────────────┘    └───────────────────┘
```

## 📋 요구 사항

- ZED 카메라 (ZED 2, ZED Mini 등)
- NVIDIA GPU (CUDA 지원)
- Docker
- X11 Display Server (로컬 또는 원격)

---

## 🐳 Docker 환경 설정

### 1. Docker 이미지 다운로드

gl-devel 버전을 사용해야 OpenCV 개발이 가능합니다:

```bash
docker pull stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04
```

### 2. 프로젝트 클론

```bash
git clone https://github.com/kodaekwan/MetaQuest_HandTracking.git
cd MetaQuest_HandTracking
```

### 3. Docker 컨테이너 실행

#### 3-1. 로컬 환경 (X11 디스플레이 직접 연결)

```bash
# X11 접근 권한 허용
xhost +local:root

docker run --gpus all \
    -it \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/bus/usb:/dev/bus/usb \
    -v ./StereoStream/ZED_StereoStream:/usr/local/zed/samples/ZED_StereoStream \
    stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04
```

#### 3-2. 원격 환경 (XLaunch 사용)

Windows의 XLaunch 또는 원격 X11 서버 사용 시:

```bash
docker run --gpus all \
    -it \
    --privileged \
    -e DISPLAY=192.168.0.201:0.0 \
    -v ./StereoStream/ZED_StereoStream:/usr/local/zed/samples/ZED_StereoStream \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/bus/usb:/dev/bus/usb \
    --network=host \
    stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04
```

> ⚠️ `DISPLAY` 환경 변수를 자신의 X11 서버 IP로 변경하세요.

---

## 🔨 빌드 방법

Docker 컨테이너 내부에서 실행:

```bash
# 프로젝트 폴더로 이동
cd /usr/local/zed/samples/ZED_StereoStream/cpp

# 필수 패키지 설치 (최초 1회)
apt update && apt install -y usbutils libopencv-dev

# CMake 빌드
cmake .
make
```

빌드가 완료되면 `ZED_VisionPro_Stream` 실행 파일이 생성됩니다.

---

## 🚀 C++ 서버 실행

```bash
# 기본 실행 (포트 자동 할당)
./ZED_VisionPro_Stream

# 특정 포트로 실행
./ZED_VisionPro_Stream --port 12345

# 미리보기 창 활성화
./ZED_VisionPro_Stream --port 12345 --preview
```

### 실행 옵션

| 옵션 | 설명 |
|------|------|
| `--port <port>` | TCP 제어 서버 포트 (0: 자동 할당) |
| `--preview` | OpenCV 미리보기 창 표시 |
| `--help` | 도움말 출력 |

### 서버 실행 후 상태

서버가 실행되면 대기 상태(IDLE)로 진입하며, Python 컨트롤러의 명령을 기다립니다:

```
=== ZED Camera ===
Model: ZED 2
Serial: 12345678
Resolution: 672x376

========================================
[Control Server] Listening on port: 12345
========================================

[Ready] Waiting for commands...
```

---

## 🐍 Python 컨트롤러 사용법

### 기본 사용법

```python
from visionpro_controller import VisionProController

# 서버에 연결
controller = VisionProController("localhost", 12345)

# 스트리밍 시작
controller.start_stream(
    ip="192.168.0.140",    # Vision Pro IP
    port=9003,              # UDP 포트
    quality=50,             # JPEG 품질 (1-100)
    width=640,              # 스트림 너비
    height=480              # 스트림 높이
)

# 녹화 시작
controller.start_record(
    path="./recordings",    # 저장 폴더
    filename="my_video"     # 파일명 (자동으로 .mp4 추가)
)

# 상태 확인
status = controller.get_status()
print(status)

# 녹화 중지
controller.stop_record()

# 스트리밍 중지
controller.stop_stream()

# 서버 종료
controller.quit()
```

### 대화형 모드

```bash
cd /usr/local/zed/samples/ZED_StereoStream/python
python3 example_controller.py --interactive
```

### 커맨드라인 사용

```bash
# 상태 확인
python3 visionpro_controller.py --port 12345 --action status

# 스트리밍 시작
python3 visionpro_controller.py --port 12345 --action start_stream --ip 192.168.0.140

# 녹화 시작
python3 visionpro_controller.py --port 12345 --action start_record --path ./videos --filename test

# 녹화 중지
python3 visionpro_controller.py --port 12345 --action stop_record

# 스트리밍 중지
python3 visionpro_controller.py --port 12345 --action stop_stream

# 서버 종료
python3 visionpro_controller.py --port 12345 --action quit
```

---

## 📡 TCP 제어 명령어 (JSON)

직접 TCP 소켓으로 제어하려면 아래 JSON 형식 사용:

### 스트리밍 시작
```json
{
    "action": "start_stream",
    "ip": "192.168.0.140",
    "port": 9003,
    "quality": 50,
    "width": 640,
    "height": 480
}
```

### 스트리밍 중지
```json
{"action": "stop_stream"}
```

### 녹화 시작
```json
{
    "action": "start_record",
    "path": "./recordings",
    "filename": "recording"
}
```

### 녹화 중지
```json
{"action": "stop_record"}
```

### 상태 조회
```json
{"action": "get_status"}
```

**응답 예시:**
```json
{
    "status": "ok",
    "state": "streaming_recording",
    "streaming": "true",
    "recording": "true",
    "recording_file": "./recordings/recording.mp4",
    "control_port": "12345"
}
```

### 스테레오 파라미터 설정 (외부 기기로 전달)
```json
{
    "action": "set_stereo_params",
    "target_ip": "192.168.0.140",
    "target_port": 9004,
    "focus": 1.0,
    "quad": 1.8,
    "zoom": 1.0
}
```

### 서버 종료
```json
{"action": "quit"}
```

---

## 📊 상태 (State) 종류

| 상태 | 설명 |
|------|------|
| `idle` | 대기 상태 |
| `streaming` | 스트리밍만 진행 중 |
| `recording` | 녹화만 진행 중 |
| `streaming_recording` | 스트리밍 + 녹화 동시 진행 |
| `stopped` | 서버 종료됨 |

---

## 💾 Docker 이미지 저장 및 재로드

### 현재 컨테이너를 이미지로 저장

Docker 컨테이너에서 OpenCV 등을 설치한 후, 매번 재설치하지 않도록 이미지로 저장합니다:

```bash
# 1. 실행 중인 컨테이너 ID 확인
docker ps

# 2. 컨테이너를 새 이미지로 저장
docker commit <CONTAINER_ID> zed-visionpro:latest

# 예시: docker commit a1b2c3d4e5f6 zed-visionpro:latest
```

### 저장된 이미지로 컨테이너 실행

```bash
xhost +local:root

docker run --gpus all \
    -it \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev/bus/usb:/dev/bus/usb \
    -v ./StereoStream/ZED_StereoStream:/usr/local/zed/samples/ZED_StereoStream \
    zed-visionpro:latest
```

### 이미지를 파일로 내보내기 (백업/이동용)

```bash
# 이미지를 tar 파일로 저장
docker save -o zed-visionpro.tar zed-visionpro:latest

# tar 파일에서 이미지 로드
docker load -i zed-visionpro.tar
```

### 모든 Docker 이미지 목록 확인

```bash
docker images
```

---

## 🔧 문제 해결

### ZED 카메라가 인식되지 않는 경우

```bash
# USB 장치 확인
lsusb | grep -i stereolabs

# 권한 문제 시
chmod 666 /dev/bus/usb/*/*
```

### X11 디스플레이 오류

```bash
# 호스트에서 실행
xhost +local:root

# 환경 변수 확인
echo $DISPLAY
```

### OpenCV 창이 표시되지 않는 경우

`--preview` 옵션 없이 실행하거나, 원격 환경인 경우 DISPLAY 설정을 확인하세요.

---

## 📝 라이센스

MIT License

## 🔗 관련 링크

- [ZED SDK Documentation](https://www.stereolabs.com/docs/)
- [ZED Docker Hub](https://hub.docker.com/r/stereolabs/zed)
- [MetaQuest_HandTracking Repository](https://github.com/kodaekwan/MetaQuest_HandTracking)
