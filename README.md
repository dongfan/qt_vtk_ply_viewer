# Qt + VTK PLY Viewer (Scan QA Prototype)

Qt + VTK 기반의 **포인트클라우드(.ply) 뷰어 및 Scan QA 프로토타입**입니다.  
3D 스캔 데이터의 **전처리 품질 검증(Scan QA)**을 목표로 하며,  
향후 **Inspection(측정)**, **Robot Pose(로봇 픽킹 좌표 추출)** 기능까지 확장하는 것을 최종 목표로 합니다.

---

## ✨ 주요 기능 (현재 구현됨)

### 1. PLY 포인트클라우드 로드
- `binary_little_endian` 형식의 PLY 포인트클라우드 지원
- 대용량 포인트클라우드 비동기 로딩 (UI freeze 방지)
- Drag & Drop 로드 지원

### 2. Scan QA – 전처리 파이프라인
실무에서 가장 많이 사용하는 **3단계 전처리**를 UI 기반으로 적용할 수 있습니다.

- **VOXEL Downsampling**
  - 포인트 수 감소 → 처리 속도 및 안정성 향상
  - 기본 ON (mm 단위 조절 가능)

- **Outlier Removal (Radius 기반)**
  - 튀는 점 제거 → 법선/피팅/측정 안정화
  - 픽킹/검사 공정에서 사실상 필수 단계

- **Plane RANSAC (바닥/기준면 추정)**
  - 바닥 평면 자동 검출
  - Plane 제거 ON/OFF 지원
  - Plane 품질 지표 계산

### 3. Scan QA 품질 지표 표시
- 총 포인트 수
- Bounding Box 크기 (X / Y / Z)
- 제거된 Outlier 수
- 제거된 Plane 포인트 수
- Plane Inlier Ratio (%)
- Plane RMSE (mm)

👉 스캔 데이터가 **다음 단계(측정/픽킹)에 사용 가능한지 판단**하기 위한 품질 지표 제공

---

## 🧭 프로젝트 구조

.
├── src/ # C++ 소스 코드
│ ├── main.cpp
│ ├── MainWindow.h
│ └── MainWindow.cpp
├── CMakeLists.txt # CMake 빌드 설정
├── CMakePresets.json # 빌드 프리셋 (VS2022 / Ninja)
├── vcpkg.json # vcpkg manifest (의존성 정의)
├── README.md
├── LICENSE.txt
├── .gitignore
└── (build/) # ❌ Git에 포함되지 않음


> `build/` 폴더 및 실행 파일은 **의도적으로 Git에서 제외**되어 있습니다.  
> (현업 기준의 정상적인 CMake 프로젝트 구조)

---

## 🛠️ 빌드 방법 (Windows / VS2022)

### 사전 준비
- Windows 10 / 11
- **Visual Studio 2022**
  - “Desktop development with C++” 워크로드 설치
- **CMake 3.20 이상**
- **vcpkg**
  - https://github.com/microsoft/vcpkg

> 본 프로젝트는 **vcpkg manifest 모드**를 사용합니다 (`vcpkg.json`).

---

### 빌드 절차

```bash
# 1. 저장소 클론
git clone <this-repository-url>
cd <repository>

# 2. CMake 구성
cmake --preset x64-release

# 3. 빌드
cmake --build --preset x64-release

빌드 완료 후 build/ 디렉토리 내에 실행 파일이 생성됩니다.

▶ 실행

빌드 결과로 생성된 실행 파일을 실행한 뒤:

.ply 파일 열기 (Open 버튼 또는 Drag & Drop)

Workspace: Scan QA

Voxel / Outlier / Plane 옵션 설정

Apply Scan QA 클릭

Raw / Processed 뷰 전환으로 결과 확인

📌 현재 제한 사항

ASCII PLY 미지원 (binary_little_endian 전용)

컬러/노멀 데이터는 현재 미사용

Inspection / Robot Pose 기능은 UI만 존재 (추후 구현 예정)