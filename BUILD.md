# IKFast Solver Build Guide (Internal)

> **2025-11-26 업데이트**: `/MD` 플래그 적용으로 하나의 `.pyd` 파일이 System Python과 Conda Python 모두에서 작동합니다.

This document describes how to build Python 3.10 x64 extension modules (`.pyd`) and native DLLs for distribution.
Note: BUILD.md is not included in the user-facing repository (gitignored). Keep this for internal release prep.

## Prerequisites
- Windows 10/11 x64
- Visual Studio 2022 Build Tools (MSVC, C++ Desktop)
- Python 3.10 x64 (System install) at `C:\Users\<you>\AppData\Local\Programs\Python\Python310\python.exe`
- Conda environment `ikfast` with Python 3.10 x64 (권장)
- vcpkg (optional) with OpenBLAS/LAPACK runtime DLLs

## Output Layout

```
ik-solver/
├── ikfast_solver.cp310-win_amd64.pyd    # Universal Python module (System + Conda)
├── bin/
│   └── IKFastUnity_x64.dll              # C#/Unity DLL
├── src/robots/*.dll                      # Robot plugin DLLs
├── lib/*.dll                             # OpenBLAS/LAPACK dependencies
└── tests/test_python.py                  # Test script
```

## Build: Native DLLs

Unity/native wrapper `IKFastUnity_x64.dll` and robot plugins are already built in this repo. If rebuilding:

- Use existing batch scripts in `src/` or rebuild projects in MSVC.
- Ensure `lib/` contains OpenBLAS/LAPACK and related DLLs as needed by plugins.

## Build: Python Extension (.pyd)

> **중요**: setup.py에 `/MD` 플래그가 포함되어 있어 **하나의 .pyd 파일이 System Python과 Conda Python 모두에서 작동**합니다.

### 권장 빌드 방법 (Conda 환경)

```powershell
conda activate ikfast
cd C:\dev\ikfast-generator\ik-solver
python setup.py build_ext --inplace
```

**출력**: `ikfast_solver.cp310-win_amd64.pyd` (루트 디렉토리)

### System Python 빌드 (대체 방법)

```powershell
cd C:\dev\ikfast-generator\ik-solver
& "C:\Users\<you>\AppData\Local\Programs\Python\Python310\python.exe" setup.py build_ext --inplace
```

### 빌드 플래그 (setup.py에 포함됨)

```python
extra_compile_args=['/std:c++17', '/EHsc', '/MD', '/O2', '/utf-8']
```

- `/MD`: 동적 MSVC 런타임 (System/Conda Python 공통 호환)
- `/O2`: 속도 최적화
- `/utf-8`: UTF-8 소스 인코딩

## Verification

**System Python**:
```powershell
cd C:\dev\ikfast-generator\ik-solver
& "C:\Users\<you>\AppData\Local\Programs\Python\Python310\python.exe" tests\test_python.py
```

**Conda Python** (권장):
```powershell
conda activate ikfast
cd C:\dev\ikfast-generator\ik-solver
python tests\test_python.py
```

**Expected output**: 8 IK solutions for MPX3500_C00X, FK error < 1e-14 m.

**DLL 충돌 발생 시**:
```powershell
$env:IKFAST_ISOLATE_PATH = "1"
python tests\test_python.py
Remove-Item Env:\IKFAST_ISOLATE_PATH
```

## Notes

- ✅ 단일 `.pyd` 파일로 System/Conda Python 모두 지원 (2025-11-26)
- ✅ `tests/test_python.py`가 numpy import 전에 DLL 경로를 자동 설정하여 BLAS 충돌 방지
- ⚠️ Python 3.10 x64 전용 (다른 버전 사용 시 재빌드 필요)
# IKFast Multi-Robot Solver - 빌드 및 유지관리 가이드

> **주의**: 이 문서는 개발자/관리자용입니다. 최종 사용자는 README.md를 참조하세요.

## 📋 목차

- [개발 환경 설정](#개발-환경-설정)
- [전체 빌드 과정](#전체-빌드-과정)
- [새 로봇 모델 추가](#새-로봇-모델-추가)
- [배포 패키지 준비](#배포-패키지-준비)
- [트러블슈팅](#트러블슈팅)

---

## 개발 환경 설정

### 필수 도구

1. **Visual Studio 2022**
   - Build Tools 또는 Community Edition
   - "Desktop development with C++" 워크로드
   - MSVC v143 컴파일러
   - Windows 10/11 SDK

2. **vcpkg** (의존성 관리)
   ```powershell
   cd C:\dev
   git clone https://github.com/Microsoft/vcpkg.git
   cd vcpkg
   .\bootstrap-vcpkg.bat

   # 통합 설치
   .\vcpkg integrate install
   ```

3. **Python 3.7+** (Python 모듈 빌드 시)
   ```powershell
   pip install pybind11 numpy
   ```

4. **CMake 3.15+** (선택사항)

### vcpkg 의존성 설치

```powershell
cd C:\dev\vcpkg

# x64 (필수)
.\vcpkg install openblas:x64-windows
.\vcpkg install lapack-reference:x64-windows

# x86 (선택사항, 32비트 빌드 시)
.\vcpkg install openblas:x86-windows
.\vcpkg install lapack-reference:x86-windows
```

### 환경 변수 설정

```powershell
$env:VCPKG_ROOT = "C:\dev\vcpkg"
```

또는 시스템 환경 변수에 영구 설정:
- 변수명: `VCPKG_ROOT`
- 값: `C:\dev\vcpkg`

---

## 전체 빌드 과정

### 1. 로봇 DLL 빌드

각 로봇의 IKFast 구현을 개별 DLL로 컴파일합니다.

**VS x64 Native Tools Command Prompt 실행**:

```powershell
cd C:\dev\ikfast-generator\ik-solver\src

# 전체 로봇 DLL 빌드 (x64)
.\build_ikfast_dlls.bat
```

**수동 빌드** (개별 로봇):

```powershell
# GP25 예제
cl /LD /O2 /EHsc ^
   /I..\include ^
   /Fe:robots\gp25_ikfast.dll ^
   robots\gp25_ikfast.cpp ^
   /link /DEF:robots\ikfast_robot.def

# LAPACK 필요한 로봇 (KJ125 등)
cl /LD /O2 /EHsc ^
   /I..\include ^
   /I%VCPKG_ROOT%\installed\x64-windows\include ^
   /Fe:robots\kj125_ikfast.dll ^
   robots\kj125_ikfast.cpp ^
   /link /DEF:robots\ikfast_robot.def ^
   /LIBPATH:%VCPKG_ROOT%\installed\x64-windows\lib ^
   lapack.lib blas.lib
```

**출력**: `src/robots/*_ikfast.dll` (7개 파일)

---

### 2. Unity/C# DLL 빌드

**방법 1: 자동 빌드 스크립트**

```powershell
cd C:\dev\ikfast-generator\ik-solver
.\rebuild_dll.bat
```

**방법 2: 수동 빌드**

VS x64 Native Tools Command Prompt:

```powershell
cd C:\dev\ikfast-generator\ik-solver\src

# 코어 + Unity 레이어 컴파일
cl /LD /O2 /EHsc /std:c++17 ^
   /I..\include ^
   /Fe:..\bin\IKFastUnity_x64.dll ^
   ikfast_core.cpp ikfast_unity.cpp ^
   /link /DEF:ikfast_unity.def

# 의존성 DLL 복사
copy "%VCPKG_ROOT%\installed\x64-windows\bin\*.dll" ..\bin\
```

**출력**: `bin/IKFastUnity_x64.dll`

---

### 3. Python 모듈 빌드

> **중요**: Conda Python과 System Python의 MSVC 런타임 호환성을 위해 `/MD` 플래그가 필수입니다. (2025-11-26 업데이트)

**자동 빌드 스크립트** (권장):

```powershell
cd C:\dev\ikfast-generator\ik-solver

# Conda 환경에서 빌드 (권장)
conda activate ikfast
python setup.py build_ext --inplace
```

**수동 빌드** (VS x64 Native Tools Command Prompt):

```powershell
# setup.py 사용 (권장 - /MD 플래그 자동 포함)
python setup.py build_ext --inplace

# 또는 직접 컴파일 (주의: /MD 플래그 필수!)
cl /LD /O2 /EHsc /std:c++17 /MD /utf-8 ^
   /I..\include ^
   /I%PYTHON_ROOT%\include ^
   /I%PYTHON_ROOT%\Lib\site-packages\pybind11\include ^
   /DIKFAST_HAS_LIBRARY ^
   /Fe:ikfast_solver.pyd ^
   src\ikfast_core.cpp src\ikfast_pybind.cpp ^
   /link %PYTHON_ROOT%\libs\python310.lib
```

**출력**: `ikfast_solver.cp310-win_amd64.pyd`

**빌드 플래그 설명**:
- `/MD`: 동적 MSVC 런타임 링크 (**필수** - Conda Python 호환성)
- `/O2`: 속도 최적화
- `/utf-8`: UTF-8 소스 인코딩

**Conda vs System Python**:
- 빌드는 어느 Python에서든 가능합니다
- `/MD` 플래그 덕분에 한 번 빌드한 `.pyd` 파일이 System Python과 Conda Python 모두에서 작동합니다
- 테스트 시 numpy import 전에 DLL 경로가 설정되어 BLAS 충돌이 방지됩니다

---

## 새 로봇 모델 추가

> **전체 워크플로우**: STEP 파일 → URDF → IKFast C++ → DLL 빌드 → 테스트 → 배포

이 섹션에서는 configs/robots.xlsx에 새로운 로봇 정보를 추가하고, STEP 파일에서 최종 DLL 배포까지의 전체 과정을 설명합니다.

### 준비 사항

**필수 환경**:
1. Docker Desktop (ikfast-env:20.04 이미지)
2. Python 3.10+ (conda 환경 권장)
3. Visual Studio 2022 (C++ 빌드 도구)
4. vcpkg (의존성 관리)

**conda 환경 활성화**:
```powershell
conda activate ikfast
```

---

### 1. 로봇 정보 등록

#### 1.1. configs/robots.xlsx 편집

Excel에서 `C:\dev\ikfast-generator\configs\robots.xlsx` 열기

**필수 컬럼**:
| 컬럼명 | 설명 | 예시 |
|--------|------|------|
| manufacturer | 제조사 | `yaskawa`, `kawasaki` |
| model | 모델명 | `gp25`, `kj125` |
| dof | 관절 수 | `6` |
| base_link | 베이스 링크 인덱스 | `0` |
| ee_link | 엔드이펙터 링크 인덱스 | `6` |
| solver_type | IKFast 솔버 타입 | `transform6d`, `translation3d` |

**예제 행**:
```
| yaskawa | gp25 | 6 | 0 | 6 | transform6d |
```

**저장**: `Ctrl+S`로 저장 후 Excel 종료

---

### 2. STEP 파일 배치

STEP 파일을 올바른 디렉토리에 배치:

```
C:\dev\ikfast-generator\robots\{manufacturer}\{model}\
├── {model}.STEP          # 필수: 로봇 3D 모델
└── meshes\               # 선택사항: 메시 파일들
    ├── link0.STL
    ├── link1.STL
    └── ...
```

**예제**:
```powershell
# 새 로봇 디렉토리 생성
cd C:\dev\ikfast-generator\robots
mkdir yaskawa\gp25

# STEP 파일 복사
copy "D:\Downloads\gp25.STEP" yaskawa\gp25\
```

---

### 3. URDF 생성

STEP 파일에서 URDF (Unified Robot Description Format) 생성:

```powershell
cd C:\dev\ikfast-generator

# 자동 생성 (robots.xlsx 참조)
python scripts/generate_robot_package.py

# 또는 수동 생성 (특정 로봇만)
python scripts/generate_robot_package.py --robot yaskawa/gp25
```

**출력**:
- `robots/yaskawa/gp25/gp25.urdf`
- `robots/yaskawa/gp25/meshes/*.stl` (STEP에서 추출)

**검증**:
```powershell
# URDF 파일 존재 확인
dir robots\yaskawa\gp25\gp25.urdf
```

---

### 4. IKFast C++ 코드 생성

Docker를 통해 URDF에서 IKFast C++ 코드 생성:

```powershell
cd C:\dev\ikfast-generator

# URDF → IKFast C++
python scripts/generate_ikfast.py robots/yaskawa/gp25/gp25.urdf
```

**프로세스**:
1. Docker 컨테이너 시작 (ikfast-env:20.04)
2. URDF → COLLADA 변환 (ROS collada_urdf)
3. OpenRAVE IKFast 생성
4. `ikfast_*.cpp` 파일 생성

**출력**:
- `robots/yaskawa/gp25/ikfast_gp25.cpp` (약 10-50MB)

**예상 시간**: 5분 ~ 30분 (로봇 복잡도에 따라)

**문제 해결**:
- Docker 메모리 부족: Docker Desktop → Settings → Resources에서 메모리 증가 (권장 20GB)
- URDF 링크 오류: `robots.xlsx`의 `base_link`, `ee_link` 인덱스 확인

---

### 5. IKFast 코드 통합

생성된 IKFast C++ 파일을 `ik-solver/src/robots/`로 복사:

```powershell
cd C:\dev\ikfast-generator

# 모든 ikfast_*.cpp 파일 통합
python scripts/integrate_all_ikfast.py
```

**프로세스**:
1. `robots/{manufacturer}/{model}/ikfast_*.cpp` 파일 검색
2. 파일명 정규화: `ikfast_*.cpp` → `{model}_ikfast.cpp`
3. `ik-solver/src/robots/`로 복사

**출력**:
```
Copying ikfast_gp25.cpp → ik-solver/src/robots/gp25_ikfast.cpp
```

**검증**:
```powershell
dir ik-solver\src\robots\gp25_ikfast.cpp
```

---

### 6. 로봇 DLL 빌드

#### 6.1. 빌드 스크립트 자동 업데이트

`integrate_all_ikfast.py`가 자동으로 빌드 스크립트를 업데이트합니다.

#### 6.2. DLL 빌드 실행

**VS x64 Native Tools Command Prompt 실행**:

```powershell
cd C:\dev\ikfast-generator\ik-solver\src

# 전체 로봇 DLL 빌드
.\build_ikfast_dlls.bat
```

**또는 개별 빌드** (LAPACK 불필요한 경우):
```powershell
cl /LD /O2 /EHsc ^
   /I..\include ^
   /Fe:robots\gp25_ikfast.dll ^
   robots\gp25_ikfast.cpp ^
   /link /DEF:robots\ikfast_robot.def
```

**LAPACK 필요한 로봇** (7축 이상 또는 복잡한 솔버):
```powershell
cl /LD /O2 /EHsc ^
   /I..\include ^
   /I%VCPKG_ROOT%\installed\x64-windows\include ^
   /Fe:robots\kj125_ikfast.dll ^
   robots\kj125_ikfast.cpp ^
   /link /DEF:robots\ikfast_robot.def ^
   /LIBPATH:%VCPKG_ROOT%\installed\x64-windows\lib ^
   lapack.lib blas.lib
```

**출력**:
- `src/robots/gp25_ikfast.dll`
- `src/robots/gp25_ikfast.exp`
- `src/robots/gp25_ikfast.lib`

---

### 7. Unity/C# DLL 재빌드

새 로봇을 포함하여 통합 DLL 재빌드:

```powershell
cd C:\dev\ikfast-generator\ik-solver

# 자동 빌드 스크립트
.\rebuild_dll.bat
```

**출력**:
- `bin/IKFastUnity_x64.dll` (업데이트됨)

---

### 8. 테스트

#### 8.1. C# 테스트

```powershell
cd C:\dev\ikfast-generator\ik-solver\tests

# Program.cs 편집 - 로봇 이름 변경
# Line 87: string robotName = "gp25";

dotnet run -c Release -p:Platform=x64
```

**확인 사항**:
- ✓ IK plugins loaded successfully
- ✓ Robot '{robotName}' loaded (DOF: 6)
- ✓ Found N solution(s)
- ✓ FK verification: err < 1e-10

#### 8.2. Python 테스트

> **중요**: Conda Python과 System Python 모두 지원됩니다. (2025-11-26 업데이트)

**Conda 환경에서 테스트** (권장):
```powershell
cd C:\dev\ikfast-generator\ik-solver

# tests/test_python.py 편집 - 로봇 이름 변경
# Line 93: robot_name = "mpx3500_c00x"

conda activate ikfast
python tests\test_python.py
```

**System Python에서 테스트**:
```powershell
cd C:\dev\ikfast-generator\ik-solver
& "C:\Users\<YOU>\AppData\Local\Programs\Python\Python310\python.exe" tests\test_python.py
```

**자동 처리 기능**:
- conda 환경 자동 감지 및 DLL 경로 최적화
- numpy import 전에 PATH 설정으로 BLAS 충돌 방지
- `CONDA_DLL_SEARCH_MODIFICATION_ENABLE=1` 자동 적용

**DLL 충돌 발생 시**:
```powershell
# PATH 완전 격리 모드
$env:IKFAST_ISOLATE_PATH = "1"
python tests\test_python.py
Remove-Item Env:\IKFAST_ISOLATE_PATH
```

#### 8.3. 검증 스크립트 (선택사항)

```powershell
cd C:\dev\ikfast-generator

# 모든 로봇 자동 검증
python scripts/verify_ikfast.py
```

---

### 9. 문서 업데이트

#### 9.1. README.md 업데이트

지원 로봇 목록에 추가:

**Google Sheets 링크 업데이트**:
- https://docs.google.com/spreadsheets/d/1bWMIM33Fbh5iHvK675droTZEdjJfaGHxCUr01nXqi9A/

또는 README.md에 직접 기록:
```markdown
사용 가능한 로봇: `"gp25"`, `"gp25_12"`, `"gp4"`, `"gp50"`, `"kj125"`, `"mpx3500_c00x"`, `"mpx3500_c10x"`
```

#### 9.2. BUILD.md 업데이트

**최종 업데이트 날짜** 변경:
```markdown
**최종 업데이트**: 2025-11-26
```

---

### 10. 배포 준비

#### 10.1. 파일 정리

불필요한 파일 제거:
```powershell
cd C:\dev\ikfast-generator\ik-solver

# 빌드 중간 파일 삭제
Remove-Item src\robots\*.obj -ErrorAction SilentlyContinue
Remove-Item src\robots\*.exp -ErrorAction SilentlyContinue
Remove-Item src\robots\*.lib -ErrorAction SilentlyContinue

# 소스 코드 제외 (선택사항 - 배포 시)
# Remove-Item src\robots\*.cpp -Confirm
```

#### 10.2. 의존성 DLL 확인

```powershell
# lib/ 폴더에 모든 의존성 존재 확인
dir lib\*.dll
```

**필수 DLL**:
- liblapack.dll
- openblas.dll
- libgfortran-5.dll
- libquadmath-0.dll
- libwinpthread-1.dll

#### 10.3. 최종 테스트 (Clean Environment)

새 PowerShell 세션에서:
```powershell
cd C:\dev\ikfast-generator\ik-solver

# C# 테스트
cd tests
dotnet run -c Release -p:Platform=x64

# Python 테스트
cd ..
conda activate ikfast
python test_python.py
```

---

### 요약: 빠른 참조

```powershell
# 1. Excel 편집
# configs/robots.xlsx에 로봇 정보 추가

# 2. STEP 파일 배치
mkdir robots\{manufacturer}\{model}
copy "{source}.STEP" robots\{manufacturer}\{model}\

# 3-5. 자동 생성 및 통합
conda activate ikfast
python scripts/generate_robot_package.py --robot {manufacturer}/{model}
python scripts/generate_ikfast.py robots/{manufacturer}/{model}/{model}.urdf
python scripts/integrate_all_ikfast.py

# 6. DLL 빌드 (VS x64 Native Tools Command Prompt)
cd ik-solver\src
.\build_ikfast_dlls.bat

# 7. Unity DLL 재빌드
cd ..
.\rebuild_dll.bat

# 8. 테스트
cd tests
dotnet run -c Release -p:Platform=x64

# 9. 배포 준비 완료!
```

---

### 트러블슈팅 (새 로봇 추가)

#### IKFast 생성 실패

**증상**: `generate_ikfast.py` 실패
```
Error: Could not find IK solutions
```

**원인**:
- URDF 링크 인덱스 오류
- 솔버 타입 부적합

**해결**:
1. `configs/robots.xlsx`에서 `base_link`, `ee_link` 확인
2. URDF 파일 열어서 링크 순서 확인
3. `solver_type` 변경 시도: `transform6d` ↔ `translation3d`

#### DLL 로드 실패

**증상**: 테스트 시 "Robot not loaded"

**원인**:
- 파일명 불일치
- Export 함수 누락

**해결**:
```powershell
# DLL Export 확인
dumpbin /exports src\robots\gp25_ikfast.dll

# 필수 함수 존재 확인:
# - ComputeIk
# - ComputeFk
# - GetNumJoints
```

---

## 배포 패키지 준비

### 1. 파일 정리

**필수 파일**:
```
ik-solver/
├── README.md
├── bin/
│   └── IKFastUnity_x64.dll
├── src/
│   └── robots/
│       ├── gp25_12_ikfast.dll
│       ├── gp25_ikfast.dll
│       ├── gp4_ikfast.dll
│       ├── gp50_ikfast.dll
│       ├── kj125_ikfast.dll
│       ├── mpx3500_c00x_ikfast.dll
│       └── mpx3500_c10x_ikfast.dll
├── dependencies/
│   ├── liblapack.dll
│   ├── openblas.dll
│   ├── libgfortran-5.dll
│   ├── libquadmath-0.dll
│   └── libwinpthread-1.dll
├── tests/
│   ├── Program.cs
│   └── bin/x64/Release/net10.0/
│       └── TestIKFast.exe
├── test_python.py
├── ikfast_solver.pyd
└── examples/
    ├── UnityExample.cs
    └── python_example.py
```

### 2. 의존성 DLL 수집

**vcpkg DLL 복사**:

```powershell
# dependencies/ 디렉토리 생성
mkdir dependencies

# vcpkg DLL 복사
copy "%VCPKG_ROOT%\installed\x64-windows\bin\liblapack.dll" dependencies\
copy "%VCPKG_ROOT%\installed\x64-windows\bin\openblas.dll" dependencies\
copy "%VCPKG_ROOT%\installed\x64-windows\bin\libgfortran-5.dll" dependencies\
copy "%VCPKG_ROOT%\installed\x64-windows\bin\libquadmath-0.dll" dependencies\
copy "%VCPKG_ROOT%\installed\x64-windows\bin\libwinpthread-1.dll" dependencies\
```

### 3. 불필요한 파일 제거

**제거할 디렉토리/파일**:
- `build/` - CMake 빌드 캐시
- `tests/bin/`, `tests/obj/` - C# 테스트 빌드 출력 (실행 파일 제외)
- `.git/` - Git 저장소 (선택사항)
- 소스 파일들:
  - `src/*.cpp`, `src/*.hpp` (사용자에게 불필요)
  - `src/robots/*.cpp` (소스 코드)

**유지할 파일**:
- DLL들 (`*.dll`)
- README.md
- 테스트 실행 파일
- 예제 코드

### 4. 테스트

배포 패키지에서 테스트 실행:

```powershell
# C# 테스트
cd tests
.\bin\x64\Release\net10.0\TestIKFast.exe

# Python 테스트
python test_python.py
```

### 5. 압축 및 배포

```powershell
# ZIP 아카이브 생성
Compress-Archive -Path ik-solver -DestinationPath ikfast-solver-v1.0.zip
```

---

## 트러블슈팅

### 빌드 오류

#### 1. "fatal error C1083: Cannot open include file: 'ikfast.h'"

**원인**: include 경로가 잘못됨

**해결**:
```powershell
# /I 옵션 확인
cl /LD /I..\include ...
```

#### 2. "LNK1120: unresolved external symbol"

**원인**: LAPACK 라이브러리 링크 실패

**해결**:
```powershell
# 링커 옵션에 vcpkg 경로 추가
/link /LIBPATH:%VCPKG_ROOT%\installed\x64-windows\lib lapack.lib blas.lib
```

#### 3. "LINK : fatal error LNK1104: cannot open file 'python39.lib'"

**원인**: Python 개발 라이브러리 경로 문제

**해결**:
```powershell
# Python 경로 확인
where python
# libs 디렉토리에 python39.lib 존재 확인
dir %PYTHON_ROOT%\libs\python39.lib
```

### 런타임 오류

#### 1. DLL 로드 실패

**원인**: 의존성 DLL이 PATH에 없음

**해결**:
```powershell
# Dependency Walker로 누락된 DLL 확인
# 또는 Process Monitor로 DLL 검색 경로 추적

# 의존성 DLL을 실행 파일과 같은 디렉토리에 복사
copy dependencies\*.dll .
```

#### 2. "Robot not loaded"

**원인**:
- `robots/` 디렉토리 경로가 잘못됨
- 로봇 DLL 파일명이 잘못됨 (예: `gp25.dll` 대신 `gp25_ikfast.dll`)

**해결**:
```csharp
// C# 코드에서 절대 경로 사용
string robotsDir = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "robots");
IKU_Init(robotsDir);
```

#### 3. Python 실행 중 멈춤 (Conda 환경)

**증상**:
- Conda Python에서 `solve_ik()` 호출 시 프로그램이 응답 없음
- System Python에서는 정상 작동

**원인**:
- Conda의 scipy-openblas와 프로젝트의 OpenBLAS/LAPACK DLL 충돌
- numpy import 시 conda의 BLAS 라이브러리가 먼저 로드되어 충돌

**해결** (2025-11-26 수정됨):
1. **setup.py에 `/MD` 플래그 추가** (이미 적용됨):
   ```python
   extra_compile_args=['/std:c++17', '/EHsc', '/MD', '/O2', '/utf-8']
   ```

2. **numpy import 전에 DLL 경로 설정** (tests/test_python.py 참조):
   ```python
   import os
   import sys

   # numpy import 전에 PATH 설정 (중요!)
   lib_path = r"path\to\ik-solver\lib"
   robots_path = r"path\to\ik-solver\src\robots"
   os.environ["PATH"] = f"{lib_path};{robots_path};" + os.environ.get("PATH", "")

   if hasattr(os, 'add_dll_directory'):
       os.add_dll_directory(lib_path)
       os.add_dll_directory(robots_path)

   # 이제 numpy import
   import numpy as np
   import ikfast_solver
   ```

3. **완전 격리 모드** (문제 지속 시):
   ```powershell
   $env:IKFAST_ISOLATE_PATH = "1"
   python your_script.py
   ```

---

## 코드 구조

### 아키텍처 개요

```
┌─────────────────────────────────────────────────────────┐
│                   User Application                      │
│                 (C# / Python / Unity)                   │
└───────────────────┬─────────────────────────────────────┘
                    │
                    ↓
┌─────────────────────────────────────────────────────────┐
│              Language Wrappers                          │
│  ┌──────────────────────┐  ┌──────────────────────┐   │
│  │  ikfast_unity.cpp    │  │  ikfast_pybind.cpp   │   │
│  │  (C# P/Invoke)       │  │  (pybind11)          │   │
│  └──────────┬───────────┘  └──────────┬───────────┘   │
└─────────────┼──────────────────────────┼───────────────┘
              │                          │
              └─────────┬────────────────┘
                        ↓
┌─────────────────────────────────────────────────────────┐
│                ikfast_core.cpp                          │
│   - Plugin loader (LoadLibrary)                         │
│   - Robot registry (std::map)                           │
│   - Function pointer cache                              │
│   - solveIK(), computeFK(), solveIKWithConfig()         │
└───────────────────┬─────────────────────────────────────┘
                    │
                    ↓
┌─────────────────────────────────────────────────────────┐
│              Robot Plugin DLLs                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │ gp25_ikfast  │  │ kj125_ikfast │  │ ...          │ │
│  │   .dll       │  │   .dll       │  │              │ │
│  └──────────────┘  └──────────────┘  └──────────────┘ │
│  각 DLL exports:                                        │
│  - ComputeIk()                                          │
│  - ComputeFk()                                          │
│  - GetNumJoints()                                       │
│  - GetNumFreeParameters()                               │
└─────────────────────────────────────────────────────────┘
```

### 핵심 파일

| 파일 | 역할 | 언어 |
|------|------|------|
| `ikfast_core.hpp/cpp` | 플러그인 로더, 코어 로직 | C++ |
| `ikfast_unity.cpp` | C# P/Invoke 레이어 | C++ |
| `ikfast_pybind.cpp` | Python 바인딩 | C++ (pybind11) |
| `robots/*_ikfast.cpp` | 각 로봇의 IKFast 구현 | C++ (IKFast 생성) |

---

## 버전 관리

### Git 전략

**.gitignore 주요 항목**:
```
# 빌드 산출물
build/
bin/
obj/
*.dll
*.pyd
*.exe

# 개발 문서 (배포 시 제외)
BUILD.md
DEVELOPMENT.md
```

**브랜치 전략**:
- `main`: 안정 버전 (배포용)
- `dev`: 개발 버전
- `feature/*`: 새 기능 개발
- `robot/*`: 새 로봇 추가

### 릴리즈 체크리스트

- [ ] 모든 로봇 DLL 빌드 성공
- [ ] Unity/C# DLL 빌드 성공
- [ ] Python 모듈 빌드 성공
- [ ] C# 테스트 통과
- [ ] Python 테스트 통과
- [ ] README.md 업데이트 (새 로봇, 버전)
- [ ] 의존성 DLL 포함 확인
- [ ] 예제 코드 동작 확인
- [ ] 버전 태그 생성: `git tag v1.0.0`

---

## 성능 최적화

### 컴파일 옵션

**Release 빌드** (최적화 활성화):
```powershell
cl /LD /O2 /Ob2 /Oi /Ot /GL /EHsc ...
```

- `/O2`: 속도 최적화
- `/Ob2`: 인라인 확장
- `/Oi`: 내장 함수 사용
- `/Ot`: 코드 속도 우선
- `/GL`: 전체 프로그램 최적화

### 프로파일링

**Visual Studio Profiler**:
1. Performance Profiler 열기
2. CPU Usage 선택
3. 테스트 실행
4. Hot path 분석

---

## 라이선스 및 저작권

- **IKFast**: Apache License 2.0 (OpenRAVE 프로젝트)
- **프로젝트 코드**: 프로젝트 라이선스 따름
- **vcpkg 의존성**: 각 라이브러리의 라이선스 준수

---

**문서 버전**: 1.1
**최종 업데이트**: 2025-11-26 (Conda Python 완전 지원 추가)
**작성자**: IKFast Team
