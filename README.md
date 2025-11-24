# IK-Solver Library

**다양한 로봇 모델을 지원하는 ikfast cpp 모듈**

## Quick Start

```cpp
#include "ikfast_solver.h"

using namespace ikfast_robotics;

int main() {
    // Create solver
    IKFastSolver solver("kawasaki_kj125");

    // Define target TCP pose (position in meters, orientation in radians)
    Pose6D target(0.5, 0.0, 0.4, 0.0, 0.0, 0.0);

    // Current joint configuration
    std::vector<double> current_joints = {0, 0, 0, 0, 0, 0};

    // Solve IK
    IKSolution solution;
    if (solver.solveIK(target, current_joints, WristConfig::NONE, solution)) {
        // Use solution.joints (in radians)
        for (double joint : solution.joints) {
            std::cout << joint << " ";
        }
    }

    return 0;
}
```

Compile:
```bash
g++ -o my_program my_program.cpp -likfast_robotics -llapack -lblas
```

---

## API Reference

### Namespace

All API is in namespace `ikfast_robotics`.

---

### 1. IKFastSolver Class

메인 클래스

#### Constructor

```cpp
explicit IKFastSolver(const std::string& robot_name)
```

로봇 모델명 받아서 인스턴스 생성

**Parameters:**
- `robot_name`: Robot model identifier
  - `"kawasaki_kj125"` or `"kj125"` - Kawasaki KJ125
  - `"yaskawa_gp4"` or `"gp4"` - Yaskawa GP4 (if available)
  -  등등

**Example:**
```cpp
IKFastSolver solver("kawasaki_kj125");
```

#### Methods

##### solveIK()

```cpp
bool solveIK(
    const Pose6D& tcp_pose,
    const std::vector<double>& current_joints,
    WristConfig wrist_config,
    IKSolution& solution
)
```

TCP 6D 포즈를 넣으면,`current_joints`와 nearest 한 joints 각을 반환 

**Parameters:**
- `tcp_pose`: 목표 TCP 6D 포즈 (x,y,z,rx,ry,rz)
- `current_joints`: 현재 조인트 각 (radians) ({a,b,c,...} 형식)
- `wrist_config`: wrist configuration (`WristConfig::NONE`, `FLIP`, or `NO_FLIP`) (아직 미사용)
- `solution`: [out] IK 해

**Returns:**
- `true` 해 있음
- `false` 해 없음 (pose unreachable)

**Example:**
```cpp
Pose6D target(0.5, 0.0, 0.4, 0.0, 0.0, 0.0);
std::vector<double> current = {0, 0, 0, 0, 0, 0};
IKSolution solution;

if (solver.solveIK(target, current, WristConfig::NONE, solution)) {
    // solution.joints contains joint angles in radians
    // solution.error contains FK verification error in meters
}
```

##### solveIKAll()

```cpp
int solveIKAll(
    const Pose6D& tcp_pose,
    std::vector<IKSolution>& solutions
)
```

대상 포즈에 대한 모든 IK 다수해

**Parameters:**
- `tcp_pose`: Target TCP 6D pose
- `solutions`: [out] Vector of all valid IK solutions

**Returns:**
- Number of solutions found (typically 0-8 for 6DOF robots)

**Example:**
```cpp
std::vector<IKSolution> all_solutions;
int count = solver.solveIKAll(target, all_solutions);

std::cout << "Found " << count << " solutions\n";
for (const auto& sol : all_solutions) {
    // Process each solution
}
```

##### computeFK()

```cpp
bool computeFK(
    const std::vector<double>& joints,
    Pose6D& tcp_pose
)
```

조인트 각을 입력하면 TCP 포즈를 반환 (검증용으로 사용)

**Parameters:**
- `joints`: Joint angles in radians (must be size == DOF)
- `tcp_pose`: [out] Resulting TCP pose

**Returns:**
- `true` if successful
- `false` if input invalid

**Example:**
```cpp
std::vector<double> joints = {0, 0, 0, 0, 0, 0};
Pose6D tcp_pose;

if (solver.computeFK(joints, tcp_pose)) {
    std::cout << "TCP at: " << tcp_pose.x << ", " << tcp_pose.y << ", " << tcp_pose.z << "\n";
}
```

---

### 2. Pose6D Structure

6D 포즈를 표현하는 구조체

```cpp
struct Pose6D {
    double x, y, z;        // 미터
    double rx, ry, rz;     // rpy값 라디안

    Pose6D();  // Default constructor (all zeros)
    Pose6D(double x, double y, double z, double rx, double ry, double rz);
};
```

**Members:**
- `x, y, z`: 미터표기 위치값 (base 좌표계 기준)
- `rx, ry, rz`: 라디안표기 회전값
  - `rx`: Roll (rotation around X-axis)
  - `ry`: Pitch (rotation around Y-axis)
  - `rz`: Yaw (rotation around Z-axis)

**Coordinate System:** Z-up 오르손 좌표계

**Example:**
```cpp
// Position (500mm, 0, 400mm), no rotation
Pose6D pose(0.5, 0.0, 0.4, 0.0, 0.0, 0.0);

// Access members
std::cout << "X: " << pose.x << " m\n";
std::cout << "Roll: " << pose.rx << " rad\n";
```

---

### 3. IKSolution Structure

IK 해를 표현하기 위한 구조체

```cpp
struct IKSolution {
    std::vector<double> joints;  // 조인트 각
    bool is_valid;               // IK해 있는지 여부
    double error;                // FK와의 오차 (딜레이때문에 빼야하나?)

    IKSolution();  // Default constructor
};
```

**Members:**
- `joints`: 라디안표기 조인트 값
- `is_valid`: IK 해가 하나라도 존재하면 `true`
- `error`: FK 검증값과의 오차

**Example:**
```cpp
IKSolution solution;
if (solver.solveIK(target, current, WristConfig::NONE, solution)) {
    std::cout << "Valid: " << solution.is_valid << "\n";
    std::cout << "Error: " << (solution.error * 1000.0) << " mm\n";

    for (size_t i = 0; i < solution.joints.size(); i++) {
        std::cout << "Joint " << i << ": " << solution.joints[i] << " rad\n";
    }
}
```

---

### 4. WristConfig Enum

Wrist Configuration 표현 위한 Enum

```cpp
enum class WristConfig {
    NONE = 0,      // No preference (find closest solution)
    FLIP = 1,      // Prefer wrist flip configuration
    NO_FLIP = 2    // Prefer wrist no-flip configuration
};
```

**Values:**
- `WristConfig::NONE`: 기본값, `current_joints`와의 nearest
- `WristConfig::FLIP`: wrist configruation이 flip인 해 우선
- `WristConfig::NO_FLIP`: wrist configruation이 no_flip인 해 우선

**Example:**
```cpp
// Find closest solution (recommended)
solver.solveIK(target, current, WristConfig::NONE, solution);

// Prefer specific wrist configuration
solver.solveIK(target, current, WristConfig::FLIP, solution);
```

---

## Building

### Requirements

- **CMake** 3.10+
- **C++ Compiler** (GCC 7+ / Clang 8+ / MSVC 2019+)
- **LAPACK/BLAS** libraries

### Ubuntu/Debian

```bash
sudo apt install cmake g++ liblapack-dev libblas-dev

cd ikfast-robotics
mkdir build && cd build
cmake ..
make
```

### Windows (with vcpkg)

```bash
vcpkg install lapack openblas

mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE="C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake --build . --config Release
```

### Testing

```bash
./test_solver
```

---

## Supported Robots

| Robot Model | Status | Namespace | Notes |
|------------|--------|-----------|-------|
| Kawasaki KJ125 | ✅ Available | `kj125_ikfast` | |
| Yaskawa GP4 | 🔄 작업중 | `gp4_ikfast` | 왜안됨 |

---


## Examples

### Example 1: Basic IK

```cpp
#include "ikfast_solver.h"
#include <iostream>

using namespace ikfast_robotics;

int main() {
    IKFastSolver solver("kawasaki_kj125");

    Pose6D target(0.5, 0.0, 0.4, 0.0, 0.0, 0.0);
    std::vector<double> current = {0, 0, 0, 0, 0, 0};
    IKSolution solution;

    if (solver.solveIK(target, current, WristConfig::NONE, solution)) {
        std::cout << "Success! Joints: ";
        for (double j : solution.joints) {
            std::cout << j << " ";
        }
        std::cout << "\n";
    }

    return 0;
}
```

### Example 2: All Solutions

```cpp
// Get all possible IK solutions
std::vector<IKSolution> all_solutions;
int count = solver.solveIKAll(target, all_solutions);

std::cout << "Found " << count << " solutions:\n";
for (size_t i = 0; i < all_solutions.size(); i++) {
    std::cout << "Solution " << i << ": error = "
              << (all_solutions[i].error * 1000.0) << " mm\n";
}
```

### Example 3: FK Verification

```cpp
// Verify IK solution with FK
if (solver.solveIK(target, current, WristConfig::NONE, solution)) {
    Pose6D fk_result;
    solver.computeFK(solution.joints, fk_result);

    double dx = target.x - fk_result.x;
    double dy = target.y - fk_result.y;
    double dz = target.z - fk_result.z;
    double pos_error = std::sqrt(dx*dx + dy*dy + dz*dz);

    std::cout << "Position error: " << (pos_error * 1000.0) << " mm\n";
}
```

### Example 4: Unit Conversion

```cpp
// Helper functions
double deg_to_rad(double deg) { return deg * M_PI / 180.0; }
double rad_to_deg(double rad) { return rad * 180.0 / M_PI; }
double mm_to_m(double mm) { return mm / 1000.0; }

// Use with conversions
Pose6D target(
    mm_to_m(500.0),      // 500mm → 0.5m
    mm_to_m(0.0),
    mm_to_m(400.0),      // 400mm → 0.4m
    deg_to_rad(0.0),     // 0° → 0 rad
    deg_to_rad(0.0),
    deg_to_rad(0.0)
);
```

---

## Adding New Robots

회사에서 쓰이는 로봇 위주 작업중이지만, 정태준에게 말씀해주시면 우선작업 가능!

---
