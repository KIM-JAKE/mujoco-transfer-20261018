# 설치와 포함 자산

## 1. 현장 PC 확인

`./field-demo doctor --output reports/arrival.json`은 OS/CPU, Python, 컴파일러, CMake, Eigen, libfranka, RT 커널/권한, 카메라 패키지, 로컬 네트워크 인터페이스와 미설정 항목을 수집합니다. 기본은 원격 네트워크·장치 연결이 없고 로봇 IP를 스캔하거나 추정하지 않습니다.

실기 실행은 호환되는 FR3 FCI와 실시간 제어 PC가 필요합니다. 포함 소스의 전체 운동 실행기는 libfranka >=0.18 / FR3 프로토콜 10을 대상으로 합니다. PC의 SDK가 설치되어 있다는 사실만으로 로봇 시스템과 호환된다는 뜻은 아닙니다.

## 2. Ubuntu 22.04 x86_64 오프라인 경로

Python 3.10, C++17 컴파일러, make 또는 ninja, dpkg-deb가 있는 PC:

```bash
scripts/install.sh --with-realsense --offline --wheelhouse vendor/wheelhouse/cp310-linux-x86_64
scripts/build-offline-ubuntu22.sh
./field-demo doctor --output reports/installed.json
scripts/offline-check.sh
```

설치 스크립트는 시스템 venv가 없으면 포함된 `virtualenv.pyz`로 환경을 만듭니다. SDK 의존성은 시스템 폴더를 바꾸지 않고 `.local-sdk`에 풉니다. 소스 빌드와 CTest가 통과한 뒤에만 환경 마커를 씁니다. 새 터미널에서도 `field-demo`가 해당 로컬 라이브러리를 사용합니다.

파일 위치를 옮겼으면 `build`를 새 위치에서 다시 생성하세요. CMake 캐시와 venv의 절대 경로는 이식용이 아닙니다. 배포 압축에는 이 서버의 build/venv를 넣지 않습니다.

컴파일러 자체나 USB 카메라 드라이버·장치 접근 권한은 번들로 대신하지 않습니다. 필요하면 현장 관리자에게 진단 보고서를 전달합니다. 자동 sudo·시스템 패키지 설치·로봇 펌웨어 변경은 없습니다.

## 3. 다른 Linux PC

```bash
scripts/install.sh
scripts/build.sh --prefix /actual/compatible/libfranka/prefix
```

카메라 패키지는 명시적으로 선택합니다. 인터넷이 필요 없으면 맞는 wheelhouse를 사용합니다.

```bash
# Python 3.12 / Linux x86_64
scripts/install.sh --with-realsense --offline --wheelhouse vendor/wheelhouse/cp312-linux-x86_64
# 또는 현장 네트워크를 사용할 때
scripts/install.sh --with-realsense
```

Python 3.10/3.12 외 버전 또는 ARM PC용 wheel은 포함하지 않습니다. 해당 플랫폼에 맞는 의존성을 준비하고 소스를 빌드합니다. 다른 시스템용 .deb를 강제 설치하지 않습니다.

호환성이 불명확할 때 설치된 SDK로 상태 조회만 빌드할 수 있습니다.

```bash
scripts/build.sh --state-only --prefix /actual/sdk/prefix
```

이 경우 조회 실행기는 `build-state/m80_read_state`입니다. 실제 설정의 `state_binary`도 그 경로로 바꿉니다. 전체 운동 기능이 준비됐다고 표시하지 않습니다.

## 4. 포함된 의존성

- `vendor/debs`: Ubuntu 22.04용 CMake/Eigen/fmt 및 조회·빌드에 필요한 라이브러리, 기존에 소스에서 빌드한 libfranka 0.18.1 패키지. 일부 SDK 의존성은 이전 Ubuntu ABI 라이브러리를 로컬 폴더에만 풉니다. Ubuntu 22.04 x86_64의 **오프라인 링크/검사** 범위로 시험했습니다.
- `vendor/libfranka-0.18.1-source.tar.gz`, `libfranka-LICENSE`, `libfranka-NOTICE`: SDK 소스와 고지. 소스 재빌드에는 원 SDK의 추가 개발 의존성이 필요할 수 있습니다.
- `vendor/wheelhouse`: Python 3.10과 3.12용 NumPy/SciPy/RealSense Python SDK wheels. 센서 펌웨어나 USB 드라이버는 포함하지 않습니다.
- `vendor/virtualenv.pyz`: PyPA 배포 가상환경 생성기. 오프라인 부트스트랩에 사용합니다.

`manifest.json`/`SHA256SUMS`에 버전과 파일 해시를 기록합니다. 센서 설치는 실제 RealSense 모델이 맞을 때 적용합니다.
