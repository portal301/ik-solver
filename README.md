# IK-Solver

IKFast 기반 IK Solver 통합 라이브러리입니다. 플러그인 아키텍처로 다양한 로봇 모델을 동적으로 로드하여 하나의 통합 API로 제공합니다. 여러 제조사의 모델 및 IKFast 이외의 방법을 추후 지원 예정입니다.

## 목차

- [지원 로봇](#지원-로봇)
- [빠른 시작](#빠른-시작)
  - [Python 사용](#python-사용)
  - [C# / Unity 사용](#c--unity-사용)
- [API 레퍼런스](#api-레퍼런스)
  - [초기화 함수](#초기화-함수)
  - [역기구학(IK) 함수](#역기구학ik-함수)
  - [정기구학(FK) 함수](#정기구학fk-함수)
- [사용 예제](#사용-예제)
  - [C# 예제](#c-예제)
  - [Python 예제](#python-예제)
- [테스트 실행](#테스트-실행)
- [문제 해결](#문제-해결)
---

## 지원 로봇

https://docs.google.com/spreadsheets/d/1bWMIM33Fbh5iHvK675droTZEdjJfaGHxCUr01nXqi9A/

ik-fast를 현재 지원중인 로봇 리스트는 엑셀링크를 참고

---

## 시작하기

### 프로젝트 파일 구성

프로젝트 디렉토리 구조:

```
ik-solver/
├── ikfast_solver.cp310-win_amd64.pyd  # Python 3.10 모듈 (System/Conda 공통)
├── bin/
│   └── IKFastUnity_x64.dll            # C#/Unity DLL (C# 사용 시)
├── src/
│   └── robots/                        # 로봇 플러그인 DLL들
│       ├── gp25_12_ikfast.dll
│       ├── gp25_ikfast.dll
│       ├── gp4_ikfast.dll
│       ├── gp50_ikfast.dll
│       ├── kj125_ikfast.dll
│       ├── mpx3500_c00x_ikfast.dll
│       └── mpx3500_c10x_ikfast.dll
└── lib/                               # 의존성 DLL들
    ├── liblapack.dll
    ├── openblas.dll
    ├── libgfortran-5.dll
    ├── libquadmath-0.dll
    └── libwinpthread-1.dll
```

### 1. C# / Unity 사용 시

### 1. C# / Unity 사용 시

자신의 프로젝트 디렉토리에 다음 파일들을 복사하세요:

```
YourProject/
├── IKFastUnity_x64.dll          # 이 저장소의 bin/IKFastUnity_x64.dll
├── robots/                       # 이 저장소의 src/robots/ 중 원하는 모델 dll 복사
│   ├── gp25_12_ikfast.dll
│   ├── gp25_ikfast.dll
│   ├── gp4_ikfast.dll
│   └── ...
└── lib/                          # 이 저장소의 lib/ 전체 복사
    ├── liblapack.dll
    ├── openblas.dll
    ├── libgfortran-5.dll
    ├── libquadmath-0.dll
    └── libwinpthread-1.dll
```

> **Unity 프로젝트**: `IKFastUnity_x64.dll`과 `lib/` 안의 의존성 DLL들을 `Assets/Plugins/x86_64/`에 복사하고, `robots/` 폴더는 빌드 실행 경로에 배치하세요.

#### C# 프로젝트 설정

1. **DLL 참조**: P/Invoke를 사용하여 `IKFastUnity_x64.dll`을 호출합니다. (아래 API 레퍼런스 참조)
2. **초기화**: `IKU_Init(robotsDir)` 호출하여 로봇 플러그인 로드
3. **사용**: IK/FK 함수 호출


### 2. Python 사용 시

자신의 프로젝트 디렉토리에 다음 파일들을 복사하세요:

```
YourProject/
├── ikfast_solver.cp310-win_amd64.pyd  # 이 저장소의 ikfast_solver.cp310-win_amd64.pyd
├── robots/                            # 이 저장소의 src/robots/ 중 원하는 모델 dll 복사
│   ├── gp25_12_ikfast.dll
│   ├── gp25_ikfast.dll
│   └── ...
└── lib/                               # 이 저장소의 lib/ 전체 복사
    ├── liblapack.dll
    ├── openblas.dll
    └── ...
```

> **참고**: Python은 `import ikfast_solver` 실행 시 자동으로 `ikfast_solver.cp310-win_amd64.pyd`를 찾아 로드합니다. 원하면 `ikfast_solver.pyd`로 이름을 변경해도 작동합니다.

#### Python 프로젝트 설정

> **참고**: System Python 3.10과 Conda Python 3.10 모두 지원됩니다.

**방법 1: Python 바인딩 모듈 사용 (권장)**

1. **DLL 검색 경로 추가** (Windows Python 3.8+, numpy import 전에 설정):
   ```python
   import os

   # numpy import 전에 DLL 경로 설정 (중요!)
   if hasattr(os, 'add_dll_directory'):
       os.add_dll_directory("path/to/robots")
       os.add_dll_directory("path/to/lib")

   import numpy as np  # DLL 경로 설정 후 import
   ```

2. **모듈 import**: `import ikfast_solver`

3. **초기화**: `ikfast_solver.load_ik_plugins(robots_dir)` (성공 시 반환값 없음, 실패 시 `RuntimeError` 발생)

**방법 2: ctypes로 DLL 직접 사용**

Python 바인딩 없이 C#용 DLL을 직접 사용할 수도 있습니다:

```python
import ctypes
import os
import numpy as np

# DLL 경로 설정
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory("path/to/lib")
    os.add_dll_directory("path/to/robots")

# DLL 로드
ikfast_dll = ctypes.CDLL("path/to/bin/IKFastUnity_x64.dll")

# 함수 시그니처 정의
ikfast_dll.IKU_Init.argtypes = [ctypes.c_char_p]
ikfast_dll.IKU_Init.restype = ctypes.c_int

ikfast_dll.IKU_GetNumJoints.argtypes = [ctypes.c_char_p]
ikfast_dll.IKU_GetNumJoints.restype = ctypes.c_int

ikfast_dll.IKU_SolveIK.argtypes = [
    ctypes.c_char_p,                     # robot_name
    ctypes.POINTER(ctypes.c_double),     # tcp_pose (12 doubles)
    ctypes.POINTER(ctypes.c_double),     # out_solutions (dof*max_solutions)
    ctypes.c_int                         # max_solutions
]
ikfast_dll.IKU_SolveIK.restype = ctypes.c_int

ikfast_dll.IKU_ComputeFK.argtypes = [
    ctypes.c_char_p,                     # robot_name
    ctypes.POINTER(ctypes.c_double),     # joints (dof doubles)
    ctypes.POINTER(ctypes.c_double),     # out_eetrans (3 doubles)
    ctypes.POINTER(ctypes.c_double)      # out_eerot (9 doubles)
]
ikfast_dll.IKU_ComputeFK.restype = ctypes.c_int

# 초기화
robots_dir = b"path/to/robots"
if ikfast_dll.IKU_Init(robots_dir) == 0:
    raise RuntimeError("Failed to initialize IKFast")

# 로봇 DOF 확인
robot_name = b"gp25"
dof = ikfast_dll.IKU_GetNumJoints(robot_name)
print(f"Robot DOF: {dof}")

# IK 계산
tcp_pose = (ctypes.c_double * 12)(1, 0, 0, 0.5, 0, 1, 0, 0, 0, 0, 1, 0.3)
max_solutions = 48
solutions = (ctypes.c_double * (dof * max_solutions))()

num_solutions = ikfast_dll.IKU_SolveIK(robot_name, tcp_pose, solutions, max_solutions)

if num_solutions > 0:
    print(f"Found {num_solutions} solution(s)")
    for i in range(num_solutions):
        sol = [solutions[i*dof + j] for j in range(dof)]
        print(f"Solution {i+1}: {sol}")

        # FK로 검증
        out_trans = (ctypes.c_double * 3)()
        out_rot = (ctypes.c_double * 9)()
        joints_array = (ctypes.c_double * dof)(*sol)

        if ikfast_dll.IKU_ComputeFK(robot_name, joints_array, out_trans, out_rot) == 1:
            print(f"  FK Position: ({out_trans[0]:.6f}, {out_trans[1]:.6f}, {out_trans[2]:.6f})")
else:
    print("No solution found")

# FK 단독 사용
joints = (ctypes.c_double * dof)(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
out_trans = (ctypes.c_double * 3)()
out_rot = (ctypes.c_double * 9)()

if ikfast_dll.IKU_ComputeFK(robot_name, joints, out_trans, out_rot) == 1:
    print(f"FK Position: ({out_trans[0]:.6f}, {out_trans[1]:.6f}, {out_trans[2]:.6f})")
    print("FK Rotation matrix (row-major):")
    print(f"  [{out_rot[0]:.6f}, {out_rot[1]:.6f}, {out_rot[2]:.6f}]")
    print(f"  [{out_rot[3]:.6f}, {out_rot[4]:.6f}, {out_rot[5]:.6f}]")
    print(f"  [{out_rot[6]:.6f}, {out_rot[7]:.6f}, {out_rot[8]:.6f}]")
```

> **참고**: ctypes 방법은 C# API와 동일한 인터페이스를 사용합니다. Python 바인딩 모듈(`ikfast_solver.pyd`)이 더 간편하고 Pythonic합니다.

---

## API 레퍼런스

### 초기화 함수

### `IKU_Init`

로봇 플러그인 DLL들을 로드합니다. 프로그램 시작 시 **반드시 한 번** 호출해야 합니다.

**C# 선언**:
```csharp
[DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
public static extern int IKU_Init([MarshalAs(UnmanagedType.LPStr)] string robots_dir);
```

**Python**:
```python
ikfast_solver.load_ik_plugins(robots_dir: str)
```

**입력 파라미터**:
- `robots_dir` (string): 로봇 DLL들이 있는 디렉토리 경로 (UTF-8)

**반환값**:
- C#: `1` = 성공, `0` = 실패
- Python: 성공 시 `None`, 실패 시 `RuntimeError` 예외 발생



**예제**:
```csharp
// C#
string robotsDir = @"C:\MyProject\robots";
if (IKU_Init(robotsDir) == 0) {
    Console.WriteLine("Failed to load robot plugins");
}
```

```python
# Python
robots_dir = "C:/MyProject/robots"
try:
    ikfast_solver.load_ik_plugins(robots_dir)
except RuntimeError:
    print("Failed to load robot plugins")
```

---


### IK 함수

### `ikfast_solver.solve_ik`

목표 TCP 자세(위치 + 회전)에 대한 **모든 IK 솔루션**을 계산합니다.

**함수 선언**:
```csharp
// C#
public static (double[][] solutions, bool is_solvable) solve_ik(
    string robot_name,
    double[] tcp_pose
)

// Python
ikfast_solver.solve_ik(
    robot_name: str,
    tcp_pose: np.ndarray
) -> Tuple[List[np.ndarray], bool]
```

**파라미터**:
- `robot_name`: 로봇 이름 (대소문자 구분 없음)
- `tcp_pose`: TCP 자세 4x4 동차 변환 행렬 (처음 3행, 12개 요소)
  - **구조**: 3x3 회전 행렬(R) + 3x1 평행이동 벡터(T)
  - 형식: `[R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz]`
  - 전체 4x4 행렬:
    ```
    [R11  R12  R13  Tx]
    [R21  R22  R23  Ty]
    [R31  R32  R33  Tz]
    [ 0    0    0   1]  ← 생략됨
    ```

**반환값**: `((double[][]) solutions, (bool) is_solvable)` 튜플
- `solutions`: 솔루션 배열 (각 솔루션은 관절 각도 배열)
- `is_solvable`: 솔루션 존재 여부 (bool)

**예제**:
```csharp
// C#: 목표 자세에 대한 모든 IK 솔루션 구하기
double[] tcp_pose = new double[] { 1, 0, 0, 0.5, 0, 1, 0, 0.0, 0, 0, 1, 0.3 };

var (solutions, is_solvable) = ikfast_solver.solve_ik("gp25", tcp_pose);

if (is_solvable) {
    Console.WriteLine($"Found {solutions.Length} solution(s)");
    for (int i = 0; i < solutions.Length; i++) {
        Console.Write($"Solution {i+1}: ");
        foreach (var angle in solutions[i]) {
            Console.Write($"{angle:F4} ");
        }
        Console.WriteLine();
    }
} else {
    Console.WriteLine("No solution found");
}
```

```python
# Python: 목표 자세에 대한 모든 IK 솔루션 구하기
import numpy as np
tcp_pose = np.array([1, 0, 0, 0.5, 0, 1, 0, 0.0, 0, 0, 1, 0.3], dtype=np.float64)

solutions, is_solvable = ikfast_solver.solve_ik("gp25", tcp_pose)

if is_solvable:
    print(f"Found {len(solutions)} solution(s)")
    for i, sol in enumerate(solutions):
        print(f"Solution {i+1}: {np.rad2deg(sol)}")  # 각도를 도(degree)로 변환
else:
    print("No solution found")
```

---

### `ikfast_solver.solve_ik_with_config`

목표 TCP 자세와 **Pose.Config**에 맞는 단일 IK 솔루션을 계산합니다.

**Pose.Config (Configuration)**:
- **Shoulder**: `RIGHT` (0) / `LEFT` (1) - J1 관절 각도의 부호
- **Elbow**: `UP` (2) / `DOWN` (3) - J3 관절 각도의 부호
- **Wrist**: `N_FLIP` (4) / `FLIP` (5) - J5 관절 각도의 부호

**C# 선언**:
```csharp
public enum PoseConfig {
    RIGHT = 0, LEFT = 1,
    UP = 2, DOWN = 3,
    N_FLIP = 4, FLIP = 5
}

public static (double[] joints, bool is_solvable) solve_ik_with_config(
    string robot_name,
    double[] tcp_pose,
    int shoulder_config,
    int elbow_config,
    int wrist_config
)
```

**Python**:
```python
ikfast_solver.solve_ik_with_config(
    robot_name: str,
    tcp_pose: np.ndarray,         # [12]: R11,R12,R13,Tx,R21,R22,R23,Ty,R31,R32,R33,Tz
    shoulder_config: int,         # 0=RIGHT, 1=LEFT
    elbow_config: int,            # 2=UP, 3=DOWN
    wrist_config: int             # 4=N_FLIP, 5=FLIP
) -> Tuple[np.ndarray, bool]      # (joints, is_solvable)
```

**파라미터**:
- `robot_name`: 로봇 이름 (대소문자 구분 없음)
- `tcp_pose`: TCP 4x4 동차 변환 행렬의 첫 3행 (12개 요소)
  - **구조**: 3x3 회전 행렬(R) + 3x1 평행이동 벡터(T)
  - 형식: `[R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz]`

- `shoulder_config`: 어깨 구성 (0=RIGHT, 1=LEFT)
- `elbow_config`: 팔꿈치 구성 (2=UP, 3=DOWN)
- `wrist_config`: 손목 구성 (4=N_FLIP, 5=FLIP)

**반환값**: `((double[][]) solutions, (bool) is_solvable)` 튜플
- `solutions`: 솔루션 배열 (각 솔루션은 관절 각도 배열)
- `is_solvable`: 솔루션 존재 여부 (bool)

**예제**:
```csharp
// C# 특정 configuration IK 솔루션 구하기
double[] tcp_pose = new double[] { 1, 0, 0, 0.5, 0, 1, 0, 0.0, 0, 0, 1, 0.3 };

var (joints, is_solvable) = ikfast_solver.solve_ik_with_config(
    "gp25", tcp_pose,
    (int)PoseConfig.RIGHT,   // shoulder
    (int)PoseConfig.DOWN,    // elbow
    (int)PoseConfig.N_FLIP   // wrist
);

if (is_solvable) {
    Console.WriteLine("Solution found:");
    for (int i = 0; i < joints.Length; i++) {
        Console.WriteLine($"  J{i+1}: {joints[i]:F4} rad");
    }
} else {
    Console.WriteLine("No solution for this configuration");
}
```

```python
# Python 특정 configuration IK 솔루션 구하기
tcp_pose = np.array([1, 0, 0, 0.5, 0, 1, 0, 0.0, 0, 0, 1, 0.3], dtype=np.float64)

joints, is_solvable = ikfast_solver.solve_ik_with_config(
    "gp25", tcp_pose,
    0,  # RIGHT
    3,  # DOWN
    4   # N_FLIP
)

if is_solvable:
    print(f"Solution: {np.rad2deg(joints)}")
else:
    print("No solution for this configuration")
```

---

### `ikfast_solver.solve_ik_with_joint`

목표 TCP 자세에 대해 **현재 관절 각도와 가장 가까운 IK 솔루션**을 계산합니다. 모든 가능한 솔루션 중에서 관절 공간에서 유클리드 거리가 최소인 솔루션을 반환합니다.

**C# 선언**:
```csharp
public static (double[] joints, bool is_solvable) solve_ik_with_joint(
    string robot_name,
    double[] tcp_pose,
    double[] current_joints
)
```

**Python**:
```python
ikfast_solver.solve_ik_with_joint(
    robot_name: str,
    tcp_pose: np.ndarray,           # [12]: R11,R12,R13,Tx,R21,R22,R23,Ty,R31,R32,R33,Tz
    current_joints: np.ndarray      # [dof] current joint angles
) -> Tuple[np.ndarray, bool]        # (joints, is_solvable)
```

**파라미터**:
- `robot_name`: 로봇 이름 (대소문자 구분 없음)
- `tcp_pose`: TCP 4x4 동차 변환 행렬의 첫 3행 (12개 요소)
  - **구조**: 3x3 회전 행렬(R) + 3x1 평행이동 벡터(T)
  - 형식: `[R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz]`

- `current_joints`: 현재 관절 각도 배열 [dof] (라디안)

**반환값**: `((double[][]) solutions, (bool) is_solvable)` 튜플
- `solutions`: 솔루션 배열 (각 솔루션은 관절 각도 배열)
- `is_solvable`: 솔루션 존재 여부 (bool)

**예제**:
```csharp
// C#: 현재 관절 각도와 가장 가까운 IK 솔루션 구하기
double[] tcp_pose = new double[] { 1, 0, 0, 0.5, 0, 1, 0, 0.0, 0, 0, 1, 0.3 };
double[] current_joints = { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 };

var (joints, is_solvable) = ikfast_solver.solve_ik_with_joint(
    "gp25", tcp_pose, current_joints
);

if (is_solvable) {
    Console.WriteLine("Nearest solution to current pose:");
    for (int i = 0; i < joints.Length; i++) {
        Console.WriteLine($"  J{i+1}: {joints[i]:F4} rad");
    }
} else {
    Console.WriteLine("No solution found");
}
```

```python
# Python
tcp_pose = np.array([1, 0, 0, 0.5, 0, 1, 0, 0.0, 0, 0, 1, 0.3], dtype=np.float64)
current_joints = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6], dtype=np.float64)

joints, is_solvable = ikfast_solver.solve_ik_with_joint(
    "gp25", tcp_pose, current_joints
)

if is_solvable:
    print("Nearest solution to current pose:")
    for i, angle in enumerate(joints):
        print(f"  J{i+1}: {angle:.4f} rad ({np.rad2deg(angle):.2f}°)")
else:
    print("No solution found")
```

---

### FK 함수

### `ikfast_solver.compute_fk`

주어진 관절 각도에 대한 **정기구학(Forward Kinematics)**을 계산하여 TCP 자세(위치 + 회전)를 반환합니다.

**C# 선언**:
```csharp
public static (double[] translation, double[] rotation) compute_fk(
    string robot_name,
    double[] joints
)
```

**Python**:
```python
ikfast_solver.compute_fk(
    robot_name: str,
    joints: np.ndarray              # [dof] joint angles
) -> Tuple[np.ndarray, np.ndarray]  # (translation, rotation)
```

**파라미터**:
- `robot_name`: 로봇 이름 (대소문자 구분 없음)
- `joints`: 관절 각도 배열 [dof] (라디안)

**반환값**: `((double[]) translation, (double[]) rotation)` 튜플
- `translation`: 위치 벡터 [3] (x, y, z) 미터 단위
- `rotation`: 회전 행렬 [9] (row-major 3x3 행렬)
  - 형식: `[R11, R12, R13, R21, R22, R23, R31, R32, R33]`
  - 전체 3x3 행렬:
    ```
    [R11  R12  R13]
    [R21  R22  R23]
    [R31  R32  R33]
    ```

**예제**:
```csharp
// C#: 관절 각도로부터 TCP 자세 계산
double[] joints = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

var (translation, rotation) = ikfast_solver.compute_fk("gp25", joints);

if (translation.Length > 0) {
    Console.WriteLine($"Position: ({translation[0]:F6}, {translation[1]:F6}, {translation[2]:F6}) m");
    Console.WriteLine("Rotation matrix (row-major):");
    Console.WriteLine($"  [{rotation[0]:F6}, {rotation[1]:F6}, {rotation[2]:F6}]");
    Console.WriteLine($"  [{rotation[3]:F6}, {rotation[4]:F6}, {rotation[5]:F6}]");
    Console.WriteLine($"  [{rotation[6]:F6}, {rotation[7]:F6}, {rotation[8]:F6}]");
} else {
    Console.WriteLine("FK computation failed");
}
```

```python
# Python: 관절 각도로부터 TCP 자세 계산
joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

translation, rotation = ikfast_solver.compute_fk("gp25", joints)

print(f"Position: ({translation[0]:.6f}, {translation[1]:.6f}, {translation[2]:.6f}) m")
print("Rotation matrix (row-major):")
rotation_matrix = rotation.reshape(3, 3)
print(rotation_matrix)

# IK 검증 예제: FK로 IK 솔루션 확인
tcp_pose = np.array([1, 0, 0, 0.5, 0, 1, 0, 0.0, 0, 0, 1, 0.3], dtype=np.float64)
solutions, is_solvable = ikfast_solver.solve_ik("gp25", tcp_pose)

if is_solvable and len(solutions) > 0:
    # 첫 번째 IK 솔루션을 FK로 검증
    fk_trans, fk_rot = ikfast_solver.compute_fk("gp25", solutions[0])

    # 목표 위치와 FK 결과 비교
    target_pos = np.array([tcp_pose[3], tcp_pose[7], tcp_pose[11]])
    position_error = np.linalg.norm(target_pos - fk_trans)
    print(f"Position error: {position_error:.3e} m")  # 일반적으로 < 1μm
```

---

## 테스트 실행

제공된 테스트 프로그램으로 설치 및 API 동작을 확인할 수 있습니다.

### C# 테스트

```powershell
cd tests
dotnet run -c Release -p:Platform=x64
```

> **참고**: .NET 10 SDK가 필요합니다. 설치: https://dotnet.microsoft.com/download

**DLL 의존성 처리**:
- 테스트 스크립트가 자동으로 `src/robots/` 디렉토리에서 로봇 플러그인을 로드합니다
- `lib/` 폴더의 의존성 DLL들은 자동으로 찾아집니다

### Python 테스트

```powershell
python tests\test_python.py
```

> **참고**: System Python 3.10 또는 Conda Python 3.10이 필요합니다.

**DLL 의존성 처리**:
- 테스트 스크립트가 자동으로 `src/robots/`와 `lib/` 디렉토리를 DLL 검색 경로에 추가합니다
- Conda 환경도 자동 감지되어 별도 설정 없이 작동합니다

### 테스트 내용

두 테스트 모두 다음 세 가지 IK 모드를 테스트합니다:

1. **모든 IK 솔루션 계산** (`IKU_SolveIK`): 목표 자세에 대한 모든 가능한 IK 솔루션 반환
2. **구성 기반 IK** (`IKU_SolveIKWithConfig`): 4가지 구성(Right-Down-NoFlip, Left-Down-NoFlip, Right-Up-NoFlip, Left-Up-Flip)에 대해 특정 구성의 IK 솔루션 계산
3. **가장 가까운 IK 솔루션** (`IKU_SolveIKWithJoint`): 현재 관절 각도와 가장 가까운 IK 솔루션 계산
4. **FK 검증**: 각 IK 솔루션을 FK로 역변환하여 정확도 확인 (오차 < 1μm)

**기본 테스트 로봇**: MPX3500_C00X (코드에서 변경 가능)

**로봇 변경 방법**:
- C#: Program.cs에서 `robotName` 변수 수정
- Python: `test_python.py`에서 `robot_name` 변수 수정

사용 가능한 로봇: `"gp25"`, `"gp25_12"`, `"gp4"`, `"gp50"`, `"kj125"`, `"mpx3500_c00x"`, `"mpx3500_c10x"`

---

## 프로젝트 구조

```
ik-solver/
├── README.md                              # 이 문서
├── ikfast_solver.cp310-win_amd64.pyd      # Python 3.10 모듈 (System/Conda 공통)
├── bin/
│   └── IKFastUnity_x64.dll                # C#/Unity 통합 DLL
├── src/
│   └── robots/                            # 로봇 플러그인 DLL들
│       ├── gp25_12_ikfast.dll
│       ├── gp25_ikfast.dll
│       ├── gp4_ikfast.dll
│       ├── gp50_ikfast.dll
│       ├── kj125_ikfast.dll
│       ├── mpx3500_c00x_ikfast.dll
│       └── mpx3500_c10x_ikfast.dll
├── lib/                                   # 의존성 DLL들
│   ├── liblapack.dll
│   ├── openblas.dll
│   ├── libgfortran-5.dll
│   ├── libquadmath-0.dll
│   └── libwinpthread-1.dll
└── tests/
    ├── Program.cs                         # C# 테스트 소스 코드
    └── test_python.py                     # Python 테스트 스크립트
```

---

## 문제 해결

### 1. DLL 로드 실패 (C#)

**증상**:
```
DllNotFoundException: Unable to load DLL 'IKFastUnity_x64'
```

**해결책**:
- `IKFastUnity_x64.dll`이 실행 파일과 같은 디렉토리에 있는지 확인
- 의존성 DLL들 (`liblapack.dll` 등)도 같은 디렉토리에 있는지 확인
- Visual C++ Redistributable 설치 확인

---

### 2. 로봇 플러그인 로드 실패

**증상**:
```
Robot 'kj125' not loaded or not available
```

**해결책**:
- `robots/` 디렉토리에 `kj125_ikfast.dll`이 있는지 확인
- LAPACK 의존성 DLL들이 있는지 확인 (kj125, mpx3500 계열 필요)
- `IKU_Init()` 호출 시 올바른 경로를 전달했는지 확인

---

### 3. Python 모듈 import 실패

**증상**:
```python
ImportError: DLL load failed while importing ikfast_solver
```

**해결책**:
```python
import os

# DLL 검색 경로 추가 (Python 3.8+ 필수)
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory("path/to/robots")
    os.add_dll_directory("path/to/lib")

import ikfast_solver  # 이제 import 가능
```

---

---

**버전**: 1.0

**최종 업데이트**: 2025-11-26

**로봇의 IK 추가는 순차적으로 이루어집니다. 급하게 필요한 모델이 있으면 정태준에게 문의주세요**

---