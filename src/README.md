# IKFast Core Wrapper (src)

IKFast 모듈에서 생성된 cpp 파일을 Wrapping 하는 코드와, 빌드파일이 위치합니다.

## 구조
- `ikfast_core.hpp` / `ikfast_core.cpp`: 메인 wrapper 
- `ikfast_pybind.cpp`: 파이썬 binding
- `ikfast_unity.cpp`: C# P/Invoke wrapper
- `build_ikfast_dlls.bat`: 로봇마다 개별 dll plugin 빌드
- `build_unity_dll.bat`: Unity/C# wrapper 빌드
- `build_all_python_versions.bat` : 파이썬(3.10~3.12)  wrapper 빌드
- `robots/`: 빌드가 완료된 dll이 {로봇제조사}/{모델명}/{모델명}.dll에 위치합니다.


## 빌드 산출물
- `ikfast_core.hpp` : integrate_all_ikfast.py에 의해 자동 생성됩니다.
- `robots/**/{robot}_ikfast.dll` : 배포될 최종 로봇 별 IKfast 모듈입니다.
