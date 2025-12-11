# IK-Solver

IKFast 기반 IK Solver 통합 라이브러리입니다. 플러그인 아키텍처로 다양한 로봇 모델을 동적으로 로드하여 하나의 통합 API로 제공합니다. 여러 제조사의 모델 및 IKFast 이외의 방법을 추후 지원 예정입니다.

## 목차

- [지원 로봇](#지원-로봇)
- [빠른 시작](#빠른-시작)
  - [프로젝트 파일 구성](#프로젝트-파일-구성)
  - [C# / Unity 사용](#1-c--unity-사용-시)
  - [Python 사용](#2-python-사용-시)
- [사용 예제](#사용-예제)
  - [Python 예제](#python-예제)
  - [C# 예제](#c-예제)
  - [Unity 예제](#unity-예제)
- [API 레퍼런스](#api-레퍼런스)
  - [초기화 함수](#초기화-함수)
  - [IK 함수](#ik-함수)
  - [FK 함수](#fk-함수)
- [테스트 실행](#테스트-실행)
- [프로젝트 구조](#프로젝트-구조)
- [문제 해결](#문제-해결)

---

## 지원 로봇 (총 13개)

**상세 정보**: [configs/robots.xlsx](../configs/robots.xlsx) 참고

---

## 빠른 시작

### 프로젝트 파일 구성

프로젝트 디렉토리 구조:

```
ik-solver/
├── ikfast_solver.cp310-win_amd64.pyd      # Python 3.10 모듈
├── ikfast_solver.cp311-win_amd64.pyd      # Python 3.11 모듈
├── ikfast_solver.cp312-win_amd64.pyd      # Python 3.12 모듈 (권장)
├── IKFastUnity_x64.dll                    # C#/Unity DLL (C# 사용 시)
└── src/
    └── robots/                            # 로봇 플러그인 DLL들 (13개) + LAPACK/BLAS
        ├── kawasaki/
        │   ├── KJ125/kj125_ikfast.dll
        │   └── RS007L/rs007l_ikfast.dll
        ├── yaskawa/
        │   ├── GP4/gp4_ikfast.dll
        │   ├── GP7/gp7_ikfast.dll
        │   ├── GP8/gp8_ikfast.dll
        │   ├── GP10/gp10_ikfast.dll
        │   ├── GP12/gp12_ikfast.dll
        │   ├── GP25/gp25_ikfast.dll
        │   ├── GP25-12/gp25_12_ikfast.dll
        │   ├── GP50/gp50_ikfast.dll
        │   ├── GP8L/gp8l_ikfast.dll
        │   ├── MPX3500-C00X/mpx3500_c00x_ikfast.dll
        │   └── MPX3500-C10X/mpx3500_c10x_ikfast.dll
        ├── liblapack.dll                  # Reference LAPACK (vcpkg)
        ├── openblas.dll                   # OpenBLAS (LAPACK 의존성)
        ├── libgfortran-5.dll              # Fortran runtime (LAPACK 의존성)
        ├── libgcc_s_seh-1.dll             # GCC runtime (LAPACK 의존성)
        └── libquadmath-0.dll              # Quad-precision math (LAPACK 의존성)
```

### 1. C# / Unity 사용 시

자신의 프로젝트 디렉토리에 다음 파일들을 복사하세요:

```
YourProject/
├── IKFastUnity_x64.dll          # 이 저장소의 IKFastUnity_x64.dll
└── robots/                       # 이 저장소의 src/robots/ 전체 복사
    ├── kawasaki/                 # Kawasaki 로봇 DLL (2개)
    │   ├── KJ125/kj125_ikfast.dll
    │   └── RS007L/rs007l_ikfast.dll
    ├── yaskawa/                  # Yaskawa 로봇 DLL (11개)
    │   ├── GP4/gp4_ikfast.dll
    │   ├── GP7/gp7_ikfast.dll
    │   ├── ... (총 11개)
    ├── liblapack.dll             # LAPACK 라이브러리
    ├── openblas.dll              # OpenBLAS (LAPACK 의존성)
    └── libgfortran-5.dll, ...    # Fortran 런타임 DLL들
```

#### C# 프로젝트 설정

1. **DLL 참조**: P/Invoke를 사용하여 `IKFastUnity_x64.dll`을 호출합니다. (아래 API 레퍼런스 참조)
2. **초기화**: `IKU_Init(robotsDir)` 호출하여 로봇 플러그인 로드
3. **사용**: IK/FK 함수 호출

#### Unity 프로젝트 설정

**1. 파일 복사**

다음 파일들을 Unity 프로젝트에 복사하세요:

```
YourUnityProject/
└── Assets/
    └── Plugins/
        └── x86_64/                       # 64비트 네이티브 플러그인 폴더
            ├── IKFastUnity_x64.dll       # 이 저장소의 IKFastUnity_x64.dll
            └── robots/                    # 이 저장소의 src/robots/ 전체 복사
                ├── kawasaki/
                │   ├── KJ125/kj125_ikfast.dll
                │   └── RS007L/rs007l_ikfast.dll
                ├── yaskawa/
                │   ├── GP4/gp4_ikfast.dll
                │   ├── GP7/gp7_ikfast.dll
                │   └── ... (총 11개)
                ├── liblapack.dll
                ├── openblas.dll
                ├── libgfortran-5.dll
                ├── libgcc_s_seh-1.dll
                └── libquadmath-0.dll
```

> **중요**: `Assets/Plugins/x86_64/` 경로는 Unity가 64비트 Windows 네이티브 플러그인을 자동으로 인식하는 표준 경로입니다. 하위 폴더 구조(`robots/kawasaki/`, `robots/yaskawa/`)는 그대로 유지하세요.

**2. 예제 스크립트 복사**

다음 스크립트들을 `Assets/Scripts/`에 복사하세요:
- `examples/IKFastWrapper.cs` - Unity용 래퍼 클래스 (필수)
- `examples/RobotIKExample.cs` - MonoBehaviour 예제 (선택)

**3. 플러그인 설정 (Inspector)**

Unity Editor에서 `IKFastUnity_x64.dll`을 선택하고 Inspector에서 다음을 확인:
- **Select platforms for plugin**: ✅ Windows
- **CPU**: x86_64

> **참고**: 로봇 DLL들(`gp25_ikfast.dll` 등)과 LAPACK 의존성 DLL들은 자동으로 로드되므로 별도 설정이 필요 없습니다.

**4. 빌드 설정**

Unity 빌드 시 주의사항:
- **Platform**: Windows
- **Architecture**: x86_64 (Intel 64-bit)
- 빌드 후 `YourGame_Data/Plugins/` 폴더에 모든 DLL이 포함되었는지 확인

**5. 사용 예제**

간단한 사용 예제:

```csharp
using UnityEngine;
using IKFast;

public class SimpleIKTest : MonoBehaviour
{
    void Start()
    {
        // 초기화 (한 번만 호출)
        string robotsPath = Application.dataPath + "/Plugins/x86_64/robots";
        if (IKFastSolver.Initialize(robotsPath))
        {
            Debug.Log("IKFast initialized successfully");
            
            // 목표 자세 설정
            Matrix4x4 targetPose = Matrix4x4.TRS(
                new Vector3(0.5f, 0f, 0.3f),  // 위치 (미터)
                Quaternion.identity,          // 회전 (0도)
                Vector3.one
            );
            
            // IK 계산
            var solutions = IKFastSolver.SolveIK("gp25", targetPose);
            Debug.Log($"Found {solutions.Length} IK solutions");
            
            if (solutions.Length > 0)
            {
                // 첫 번째 솔루션 출력 (라디안 -> 도)
                double[] jointsDeg = IKFastSolver.RadToDeg(solutions[0]);
                Debug.Log($"Solution 1: [{string.Join(", ", jointsDeg)}]");
            }
        }
    }
}
```

전체 예제는 `RobotIKExample.cs`를 참고하세요.


### 2. Python 사용 시

자신의 프로젝트 디렉토리에 다음 파일들을 복사하세요:

```
YourProject/
├── ikfast_solver.cp3XX-win_amd64.pyd  # Python 버전에 맞는 .pyd 파일
└── robots/                            # 이 저장소의 src/robots/ 전체 복사
    ├── kawasaki/                  # Kawasaki 로봇 DLL (2개)
    ├── yaskawa/                   # Yaskawa 로봇 DLL (11개)
    ├── liblapack.dll              # LAPACK 라이브러리
    ├── openblas.dll               # OpenBLAS (LAPACK 의존성)
    └── libgfortran-5.dll, ...     # Fortran 런타임 DLL들
```

**Python 버전별 .pyd 파일 선택**:
- **Python 3.12 (권장)**: `ikfast_solver.cp312-win_amd64.pyd`
- **Python 3.11**: `ikfast_solver.cp311-win_amd64.pyd`
- **Python 3.10**: `ikfast_solver.cp310-win_amd64.pyd`

> **중요**: 사용 중인 Python 버전에 맞는 .pyd 파일을 프로젝트에 복사하세요. Python은 `import ikfast_solver` 실행 시 자동으로 현재 환경의 Python 버전에 맞는 `.cp3XX-win_amd64.pyd` 파일을 찾아 로드합니다. 이름 변경은 필요 없습니다.

#### Python 프로젝트 설정

> **지원 버전**: Python 3.10, 3.11, 3.12 (권장: 3.12)
>
> **빌드 환경**: uv 또는 conda 환경 모두 지원됩니다. 빌드 스크립트가 자동으로 Python 경로와 버전을 감지합니다.

**Python 바인딩 모듈 사용**

1. **DLL 검색 경로 추가** (Windows Python 3.8+):
   ```python
   import os
   import numpy as np  # numpy를 먼저 import

   # vcpkg bin 디렉토리 추가 (Fortran 런타임 등 LAPACK 의존성)
   vcpkg_bin = os.path.join(os.environ.get("VCPKG_ROOT", r"C:\dev\vcpkg"), "installed", "x64-windows", "bin")

   if hasattr(os, 'add_dll_directory'):
       if os.path.isdir(vcpkg_bin):
           os.add_dll_directory(vcpkg_bin)
       os.add_dll_directory("path/to/robots")

   import ikfast_solver
   ```

2. **초기화**: `ikfast_solver.load_ik_plugins(robots_dir)` (성공 시 반환값 없음, 실패 시 `RuntimeError` 발생)

   > **중요**: KJ125, MPX3500 시리즈를 사용하려면 vcpkg bin 디렉토리를 반드시 추가해야 합니다. 이 디렉토리에는 LAPACK이 필요로 하는 Fortran 런타임 라이브러리들(`libgfortran-5.dll`, `libquadmath-0.dll` 등)이 있습니다. (문제 해결 단락 참조)

---

## 사용 예제

### Python 예제

**완전한 IK/FK 사용 예제**

```python
import os
import numpy as np

# 1. DLL 검색 경로 설정 (numpy를 먼저 import)
vcpkg_bin = os.path.join(os.environ.get("VCPKG_ROOT", r"C:\dev\vcpkg"), "installed", "x64-windows", "bin")
robots_dir = os.path.abspath("robots")

if hasattr(os, 'add_dll_directory'):
    if os.path.isdir(vcpkg_bin):
        os.add_dll_directory(vcpkg_bin)
    os.add_dll_directory(robots_dir)

# 2. ikfast_solver 모듈 import 및 초기화
import ikfast_solver

ikfast_solver.load_ik_plugins(robots_dir)

# 3. 로봇 선택
robot_name = "gp25"
dof = ikfast_solver.get_num_joints(robot_name)
print(f"Robot: {robot_name}, DOF: {dof}")

# 4. 목표 TCP 자세 설정 (4x4 동차 변환 행렬의 처음 3행)
# 위치: (0.5, 0.0, 0.3) m, 회전: identity (0도)
tcp_pose = np.array([
    1, 0, 0, 0.5,    # R11, R12, R13, Tx
    0, 1, 0, 0.0,    # R21, R22, R23, Ty
    0, 0, 1, 0.3     # R31, R32, R33, Tz
], dtype=np.float64)

# 5. IK 계산 - 모든 솔루션
solutions, is_solvable = ikfast_solver.solve_ik(robot_name, tcp_pose)

if is_solvable:
    print(f"\n찾은 솔루션 개수: {len(solutions)}")

    # 첫 번째 솔루션 출력 (라디안 -> 도)
    sol1_deg = np.rad2deg(solutions[0])
    print(f"솔루션 1: {[f'{d:.2f}°' for d in sol1_deg]}")

    # FK로 검증
    fk_trans, fk_rot = ikfast_solver.compute_fk(robot_name, solutions[0])
    target_pos = np.array([tcp_pose[3], tcp_pose[7], tcp_pose[11]])
    error = np.linalg.norm(target_pos - fk_trans)
    print(f"FK 검증 오차: {error:.3e} m")
else:
    print("솔루션을 찾을 수 없습니다.")

# 6. 특정 Configuration으로 IK 계산
joints, is_solvable = ikfast_solver.solve_ik_with_config(
    robot_name, tcp_pose,
    0,  # RIGHT shoulder
    1,  # DOWN elbow
    0   # N_FLIP wrist
)

if is_solvable:
    print(f"\nConfig 솔루션: {[f'{d:.2f}°' for d in np.rad2deg(joints)]}")

# 7. 현재 관절 각도에서 가장 가까운 IK 솔루션
current_joints = np.zeros(dof, dtype=np.float64)
joints, is_solvable = ikfast_solver.solve_ik_with_joint(
    robot_name, tcp_pose, current_joints
)

if is_solvable:
    print(f"\n가장 가까운 솔루션: {[f'{d:.2f}°' for d in np.rad2deg(joints)]}")
```

---

### C# 예제

**완전한 IK/FK 사용 예제**

> **Unity 사용자**: 아래 C# 예제 대신 [Unity 예제](#unity-예제) 섹션을 참고하세요. Unity에서는 `IKFastWrapper.cs`를 사용하는 것이 더 편리합니다.


```csharp
using System;
using System.Runtime.InteropServices;

class IKFastExample
{
    // P/Invoke 선언
    [DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    private static extern int IKU_Init([MarshalAs(UnmanagedType.LPStr)] string robots_dir);

    [DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    private static extern int IKU_GetNumJoints([MarshalAs(UnmanagedType.LPStr)] string robot_name);

    [DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    private static extern int IKU_SolveIK(
        [MarshalAs(UnmanagedType.LPStr)] string robot_name,
        double[] tcp_pose,
        double[] out_solutions,
        int max_solutions
    );

    [DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    private static extern int IKU_SolveIKWithConfig(
        [MarshalAs(UnmanagedType.LPStr)] string robot_name,
        double[] tcp_pose,
        int shoulder_config,
        int elbow_config,
        int wrist_config,
        double[] out_joints,
        out int is_solvable
    );

    [DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
    private static extern int IKU_ComputeFK(
        [MarshalAs(UnmanagedType.LPStr)] string robot_name,
        double[] joints,
        double[] out_eetrans,
        double[] out_eerot
    );

    static void Main()
    {
        // 1. 초기화
        string robotsDir = @"robots";
        if (IKU_Init(robotsDir) == 0)
        {
            Console.WriteLine("Failed to load robot plugins");
            return;
        }

        // 2. 로봇 선택
        string robotName = "gp25";
        int dof = IKU_GetNumJoints(robotName);
        Console.WriteLine($"Robot: {robotName}, DOF: {dof}");

        // 3. 목표 TCP 자세 설정 (4x4 동차 변환 행렬의 처음 3행)
        // 위치: (0.5, 0.0, 0.3) m, 회전: identity
        double[] tcpPose = new double[] {
            1, 0, 0, 0.5,   // R11, R12, R13, Tx
            0, 1, 0, 0.0,   // R21, R22, R23, Ty
            0, 0, 1, 0.3    // R31, R32, R33, Tz
        };

        // 4. IK 계산 - 모든 솔루션
        const int maxSolutions = 48;
        double[] solutions = new double[dof * maxSolutions];
        int numSolutions = IKU_SolveIK(robotName, tcpPose, solutions, maxSolutions);

        if (numSolutions > 0)
        {
            Console.WriteLine($"\n찾은 솔루션 개수: {numSolutions}");

            // 첫 번째 솔루션 출력 (라디안 -> 도)
            Console.Write("솔루션 1: ");
            for (int j = 0; j < dof; j++)
            {
                double deg = solutions[j] * 180.0 / Math.PI;
                Console.Write($"{deg:F2}° ");
            }
            Console.WriteLine();

            // FK로 검증
            double[] joints = new double[dof];
            Array.Copy(solutions, 0, joints, 0, dof);

            double[] fkTrans = new double[3];
            double[] fkRot = new double[9];
            IKU_ComputeFK(robotName, joints, fkTrans, fkRot);

            double error = Math.Sqrt(
                Math.Pow(tcpPose[3] - fkTrans[0], 2) +
                Math.Pow(tcpPose[7] - fkTrans[1], 2) +
                Math.Pow(tcpPose[11] - fkTrans[2], 2)
            );
            Console.WriteLine($"FK 검증 오차: {error:E3} m");
        }
        else
        {
            Console.WriteLine("솔루션을 찾을 수 없습니다.");
        }

        // 5. 특정 Configuration으로 IK 계산
        double[] configJoints = new double[dof];
        int isSolvable;

        IKU_SolveIKWithConfig(
            robotName, tcpPose,
            0,  // RIGHT shoulder
            3,  // DOWN elbow
            4,  // N_FLIP wrist
            configJoints,
            out isSolvable
        );

        if (isSolvable == 1)
        {
            Console.Write("\nConfig 솔루션: ");
            for (int j = 0; j < dof; j++)
            {
                double deg = configJoints[j] * 180.0 / Math.PI;
                Console.Write($"{deg:F2}° ");
            }
            Console.WriteLine();
        }
    }
}
```

> **예제 파일 위치**: 위 예제 코드는 [examples/](examples/) 디렉토리에서 확인할 수 있습니다.
> - Python: [example_python.py](examples/example_python.py)
> - C#: [example_csharp.cs](examples/example_csharp.cs)

---

### Unity 예제

**Unity MonoBehaviour 예제**

Unity에서는 `IKFastWrapper.cs` 래퍼 클래스를 사용하여 간편하게 IKFast를 호출할 수 있습니다.

#### 1. 기본 사용법

```csharp
using UnityEngine;
using IKFast;

public class SimpleIKTest : MonoBehaviour
{
    void Start()
    {
        // 1. 초기화 (한 번만 호출)
        string robotsPath = Application.dataPath + "/Plugins/x86_64/robots";
        
        if (!IKFastSolver.Initialize(robotsPath))
        {
            Debug.LogError("Failed to initialize IKFast");
            return;
        }
        
        Debug.Log("IKFast initialized successfully");
        
        // 2. 로봇 정보 조회
        string robotName = "gp25";
        int dof = IKFastSolver.GetNumJoints(robotName);
        Debug.Log($"Robot: {robotName}, DOF: {dof}");
        
        // 3. 관절 제한 조회
        var limits = IKFastSolver.GetJointLimits(robotName);
        if (limits.HasValue)
        {
            Debug.Log("Joint Limits:");
            for (int i = 0; i < dof; i++)
            {
                double lowerDeg = IKFastSolver.RadToDeg(limits.Value.Lower[i]);
                double upperDeg = IKFastSolver.RadToDeg(limits.Value.Upper[i]);
                Debug.Log($"  Joint {i + 1}: [{lowerDeg:F2}°, {upperDeg:F2}°]");
            }
        }
        
        // 4. 목표 TCP 자세 설정
        Matrix4x4 targetPose = Matrix4x4.TRS(
            new Vector3(0.5f, 0f, 0.3f),  // 위치 (미터)
            Quaternion.identity,          // 회전
            Vector3.one
        );
        
        // 5. IK 계산 - 모든 솔루션
        var solutions = IKFastSolver.SolveIK(robotName, targetPose);
        Debug.Log($"Found {solutions.Length} IK solutions");
        
        if (solutions.Length > 0)
        {
            // 첫 번째 솔루션 출력 (라디안 -> 도)
            double[] jointsDeg = IKFastSolver.RadToDeg(solutions[0]);
            string jointsStr = string.Join(", ", System.Array.ConvertAll(jointsDeg, j => $"{j:F2}°"));
            Debug.Log($"Solution 1: [{jointsStr}]");
            
            // FK로 검증
            var fkResult = IKFastSolver.ComputeFK(robotName, solutions[0]);
            if (fkResult.HasValue)
            {
                Vector3 fkPos = fkResult.Value.GetPosition();
                float error = Vector3.Distance(targetPose.GetPosition(), fkPos);
                Debug.Log($"FK verification error: {error:E3} m");
            }
        }
    }
}
```

#### 2. 특정 구성(Configuration)으로 IK 계산

```csharp
// Right-Down-NoFlip 구성의 솔루션 찾기
double[] joints;
bool success = IKFastSolver.SolveIKWithConfig(
    robotName, targetPose,
    RobotConfig.RIGHT,   // 어깨
    RobotConfig.DOWN,    // 팔꿈치
    RobotConfig.N_FLIP,  // 손목
    out joints
);

if (success)
{
    double[] jointsDeg = IKFastSolver.RadToDeg(joints);
    Debug.Log($"Config solution: [{string.Join(", ", jointsDeg)}]");
}
```

#### 3. 현재 위치에서 가장 가까운 솔루션

```csharp
// 현재 관절 각도 (라디안)
double[] currentJoints = new double[dof];  // 0으로 초기화

// 가장 가까운 솔루션 계산
double[] nearestJoints;
bool success = IKFastSolver.SolveIKNearest(
    robotName, targetPose, currentJoints,
    out nearestJoints
);

if (success)
{
    double[] jointsDeg = IKFastSolver.RadToDeg(nearestJoints);
    Debug.Log($"Nearest solution: [{string.Join(", ", jointsDeg)}]");
}
```

#### 4. GameObject를 사용한 전체 예제

`RobotIKExample.cs` 스크립트를 GameObject에 부착하여 사용:

1. **GameObject 생성**: Hierarchy에서 빈 GameObject 생성 (이름: "RobotIKTester")
2. **스크립트 부착**: `RobotIKExample.cs`를 드래그하여 GameObject에 부착
3. **Inspector 설정**:
   - Robot Name: `gp25` (또는 다른 로봇)
   - Target Position: `(0.5, 0, 0.3)`
   - Target Rotation: `(0, 0, 0)`
   - Auto Initialize: ✅
4. **Play**: Play 모드 실행 후 Console 확인

**Context Menu로 테스트**:
- Inspector에서 Component 우클릭
- "Initialize IKFast" - 초기화
- "Solve IK - All Solutions" - 모든 솔루션 계산
- "Solve IK - With Configuration" - 특정 구성 솔루션
- "Solve IK - Nearest" - 가장 가까운 솔루션
- "Compute FK" - FK 계산

> **예제 파일 위치**: 
> - Unity 래퍼: [examples/IKFastWrapper.cs](examples/IKFastWrapper.cs)
> - MonoBehaviour 예제: [examples/RobotIKExample.cs](examples/RobotIKExample.cs)

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
- **Elbow**: `UP` (0) / `DOWN` (1) - J3 관절 각도의 부호
- **Wrist**: `N_FLIP` (0) / `FLIP` (1) - J5 관절 각도의 부호

**C# 선언**:
```csharp
public enum PoseConfig {
    RIGHT = 0, LEFT = 1,
    UP = 0, DOWN = 1,
    N_FLIP = 0, FLIP = 1
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
    elbow_config: int,            # 0=UP, 1=DOWN
    wrist_config: int             # 0=N_FLIP, 1=FLIP
) -> Tuple[np.ndarray, bool]      # (joints, is_solvable)
```

**파라미터**:
- `robot_name`: 로봇 이름 (대소문자 구분 없음)
- `tcp_pose`: TCP 4x4 동차 변환 행렬의 첫 3행 (12개 요소)
  - **구조**: 3x3 회전 행렬(R) + 3x1 평행이동 벡터(T)
  - 형식: `[R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz]`

- `shoulder_config`: 어깨 구성 (0=RIGHT, 1=LEFT)
- `elbow_config`: 팔꿈치 구성 (0=UP, 1=DOWN)
- `wrist_config`: 손목 구성 (0=N_FLIP, 1=FLIP)

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
    1,  # DOWN
    0   # N_FLIP
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

## 빌드 가이드

### 멀티 버전 Python 모듈 빌드

Python 3.10, 3.11, 3.12용 모듈을 한 번에 빌드하려면:

**Windows Batch:**
```powershell
cd ik-solver
build_all_python_versions.bat
```

**PowerShell:**
```powershell
cd ik-solver
.\build_all_python_versions.ps1
```

빌드 후 다음 파일이 생성됩니다:
- `ikfast_solver.cp310-win_amd64.pyd` (Python 3.10)
- `ikfast_solver.cp311-win_amd64.pyd` (Python 3.11)
- `ikfast_solver.cp312-win_amd64.pyd` (Python 3.12, 권장)

> **요구사항**:
> - [uv](https://github.com/astral-sh/uv) 설치 필요
> - Visual Studio Build Tools 2022 (C++ 워크로드)

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

> **참고**: Python 3.12+ 필요. uv 또는 conda 환경 모두 지원됩니다.

**DLL 의존성 처리**:
- 테스트 스크립트가 자동으로 `src/robots/` 디렉토리를 DLL 검색 경로에 추가합니다 (LAPACK 포함)
- Conda 환경도 자동 감지되어 별도 설정 없이 작동합니다

### 테스트 내용

두 테스트 모두 다음 세 가지 IK 모드를 테스트합니다:

1. **모든 IK 솔루션 계산** (`IKU_SolveIK`): 목표 자세에 대한 모든 가능한 IK 솔루션 반환
2. **구성 기반 IK** (`IKU_SolveIKWithConfig`): 4가지 구성(Right-Down-NoFlip, Left-Down-NoFlip, Right-Up-NoFlip, Left-Up-Flip)에 대해 특정 구성의 IK 솔루션 계산
3. **가장 가까운 IK 솔루션** (`IKU_SolveIKWithJoint`): 현재 관절 각도와 가장 가까운 IK 솔루션 계산
4. **FK 검증**: 각 IK 솔루션을 FK로 역변환하여 정확도 확인 (오차 < 1μm)

**기본 테스트**: 모든 로봇 자동 테스트 (13개)

**사용 가능한 로봇** (대소문자 구분 없음):
- Kawasaki: `"kj125"`, `"rs007l"`
- Yaskawa: `"gp4"`, `"gp7"`, `"gp8"`, `"gp8l"`, `"gp10"`, `"gp12"`, `"gp25"`, `"gp25_12"`, `"gp50"`, `"mpx3500_c00x"`, `"mpx3500_c10x"`

---

## 프로젝트 구조

```
ik-solver/
├── README.md                              # 이 문서
├── ikfast_solver.cp310-win_amd64.pyd      # Python 3.10 모듈
├── ikfast_solver.cp311-win_amd64.pyd      # Python 3.11 모듈
├── ikfast_solver.cp312-win_amd64.pyd      # Python 3.12 모듈 (권장)
├── IKFastUnity_x64.dll                    # C#/Unity 통합 DLL
├── build_all_python_versions.bat          # 멀티 버전 빌드 (Windows Batch)
├── build_all_python_versions.ps1          # 멀티 버전 빌드 (PowerShell)
├── src/
│   ├── ikfast_core.hpp                    # 관절 제한 데이터
│   ├── ikfast_core.cpp                    # 플러그인 로더
│   ├── ikfast_pybind.cpp                  # Python 바인딩
│   ├── ikfast_unity.cpp                   # C# 래퍼
│   ├── build_ikfast_dlls.bat              # DLL 빌드 스크립트
│   ├── build_unity_dll.bat                # Unity DLL 빌드
│   └── robots/                            # 로봇 플러그인 DLL들 (13개) + LAPACK/BLAS
│       ├── kawasaki/
│       │   ├── KJ125/kj125_ikfast.dll
│       │   └── RS007L/rs007l_ikfast.dll
│       ├── yaskawa/
│       │   ├── GP4/gp4_ikfast.dll
│       │   ├── GP7/gp7_ikfast.dll
│       │   ├── GP8/gp8_ikfast.dll
│       │   ├── GP8L/gp8l_ikfast.dll
│       │   ├── GP10/gp10_ikfast.dll
│       │   ├── GP12/gp12_ikfast.dll
│       │   ├── GP25/gp25_ikfast.dll
│       │   ├── GP25-12/gp25_12_ikfast.dll
│       │   ├── GP50/gp50_ikfast.dll
│       │   ├── MPX3500-C00X/mpx3500_c00x_ikfast.dll
│       │   └── MPX3500-C10X/mpx3500_c10x_ikfast.dll
│       ├── liblapack.dll                  # Reference LAPACK (vcpkg)
│       ├── openblas.dll                   # OpenBLAS (LAPACK 의존성)
│       ├── libgfortran-5.dll              # Fortran runtime (LAPACK 의존성)
│       ├── libgcc_s_seh-1.dll             # GCC runtime (LAPACK 의존성)
│       └── libquadmath-0.dll              # Quad-precision math (LAPACK 의존성)
├── examples/
│   ├── example_python.py                  # Python 사용 예제
│   ├── example_csharp.cs                  # C# 콘솔 사용 예제
│   ├── IKFastWrapper.cs                   # Unity 래퍼 클래스
│   └── RobotIKExample.cs                  # Unity MonoBehaviour 예제
└── tests/
    ├── unified_test.py                    # Python 통합 테스트
    ├── Program.cs                         # C# 테스트 소스 코드
    ├── build_and_run.bat                  # 빌드 및 실행
    └── run_unified_tests.bat              # 테스트 실행
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
- `robots/` 디렉토리에 로봇 DLL들과 LAPACK/BLAS DLL들이 있는지 확인
- Visual C++ Redistributable 설치 확인

---

### 2. 로봇 플러그인 로드 실패

**증상**:
```
Robot 'kj125' not loaded or not available
```

**해결책**:
- `robots/` 디렉토리에 `kj125_ikfast.dll`이 있는지 확인
- `robots/` 디렉토리에 LAPACK 의존성 DLL들이 있는지 확인 (`liblapack.dll`, `openblas.dll`, Fortran 런타임 DLL들)
- `IKU_Init()` 또는 `load_ik_plugins()` 호출 시 올바른 경로를 전달했는지 확인

---

### 3. Python 모듈 import 실패

**증상**:
```python
ImportError: DLL load failed while importing ikfast_solver
```

**해결책**:
```python
import os
import numpy as np  # numpy를 먼저 import

# DLL 검색 경로 추가 (Python 3.8+ 필수)
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory("path/to/robots")

import ikfast_solver  # 이제 import 가능
```

---

### 4. Unity에서 DLL 로드 실패

**증상**:
```
DllNotFoundException: IKFastUnity_x64
```

**해결책**:

1. **파일 위치 확인**:
   - `Assets/Plugins/x86_64/IKFastUnity_x64.dll` 존재 확인
   - `Assets/Plugins/x86_64/robots/` 디렉토리에 모든 로봇 DLL과 의존성 DLL 존재 확인

2. **플러그인 설정 확인** (Inspector):
   - `IKFastUnity_x64.dll` 선택
   - Platform: ✅ Windows
   - CPU: x86_64
   - Load on startup: ✅ (권장)

3. **빌드 설정 확인**:
   - Build Settings → Architecture: x86_64
   - Player Settings → Configuration → Scripting Backend: Mono (권장) 또는 IL2CPP

4. **초기화 경로 확인**:
   ```csharp
   // Editor에서 실행 시
   string robotsPath = Application.dataPath + "/Plugins/x86_64/robots";
   
   // 빌드된 실행 파일에서는
   string robotsPath = Application.dataPath + "/Plugins/robots";
   ```

5. **Visual C++ Redistributable 설치**:
   - Unity Editor와 빌드된 게임 모두 필요
   - https://aka.ms/vs/17/release/vc_redist.x64.exe

---

### 5. LAPACK 충돌 문제 (KJ125, MPX3500 시리즈)

**증상**:
- 관절 구조가 일반적이지 않은 특정 로봇만 문제 발생

**원인**:
KJ125와 MPX3500 시리즈 등 일반적이지 않은 관절구조를 지닌 로봇은 IKFast 내부에서 eigenvalue 계산을 위해 LAPACK의 `dgeev_` 함수를 호출합니다. Conda/Miniconda 환경에서는 conda의 LAPACK 라이브러리나 vcpkg의 OpenBLAS가 특정 케이스에서 무한 루프를 발생시킬 수 있습니다.

**해결책**:

**vcpkg Reference LAPACK 사용 (권장)**

1. vcpkg에서 reference LAPACK 설치:
   ```powershell
   C:\dev\vcpkg\vcpkg.exe install lapack:x64-windows
   ```

2. Reference LAPACK 및 의존성 DLL들을 robots 디렉토리에 복사:
   ```powershell
   # LAPACK 및 BLAS 라이브러리
   Copy-Item C:\dev\vcpkg\installed\x64-windows\bin\lapack.dll src\robots\liblapack.dll -Force
   Copy-Item C:\dev\vcpkg\installed\x64-windows\bin\openblas.dll src\robots\openblas.dll -Force

   # Fortran 런타임 의존성 (LAPACK이 필요로 함)
   Copy-Item C:\dev\vcpkg\installed\x64-windows\bin\libgfortran-5.dll src\robots\ -Force
   Copy-Item C:\dev\vcpkg\installed\x64-windows\bin\libgcc_s_seh-1.dll src\robots\ -Force
   Copy-Item C:\dev\vcpkg\installed\x64-windows\bin\libquadmath-0.dll src\robots\ -Force
   ```

3. `src/robots/` 디렉토리 구조 확인:
   ```
   src/robots/
   ├── liblapack.dll          # Reference LAPACK (vcpkg)
   ├── openblas.dll           # OpenBLAS (vcpkg, LAPACK 의존성)
   ├── libgfortran-5.dll      # Fortran runtime
   ├── libgcc_s_seh-1.dll     # GCC runtime
   ├── libquadmath-0.dll      # Quad-precision math
   ├── gp25_ikfast.dll        # 로봇 플러그인 DLL들...
   ├── kj125_ikfast.dll
   └── mpx3500_c00x_ikfast.dll
   ```

4. 빌드 및 테스트:
   ```powershell
   # Python 모듈 빌드
   python setup.py build_ext --inplace --force

   # 테스트
   python .\tests\unified_test.py
   ```

**검증**:
정상 작동하면 다음과 같은 메시지가 나타나고, 모든 로봇의 IK 계산이 완료됩니다:
```
Testing 13 robot(s):
--------------------------------------------------------------------------------
GP4       | solve_ik =OK (08 sol, err=0.000000)  ...
...
KJ125     | solve_ik =OK (08 sol, err=0.000000)  ...
--------------------------------------------------------------------------------
Result: 13/13 robots passed all tests
```

**참고**: OpenBLAS 대신 reference LAPACK을 사용하는 이유는 OpenBLAS의 `dgeev_` 구현이 특정 입력에서 불안정할 수 있기 때문입니다. Reference LAPACK은 느리지만 매우 안정적입니다.

---

**최종 업데이트**: 2025-12-08

**지원 로봇**: 13개 (Kawasaki 2개, Yaskawa 11개)
**상태**: Production Ready ✅

**로봇의 IK 추가는 순차적으로 이루어집니다. 급하게 필요한 모델이 있으면 정태준에게 문의주세요**

---