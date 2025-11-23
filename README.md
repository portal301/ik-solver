# IKFast Robotics Library

**Unified inverse kinematics solver library for multiple robot models**

Simple, high-performance C++ library providing a single API for computing inverse kinematics across multiple industrial robot models using pre-generated IKFast solvers.

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

Main solver class providing unified interface for all robot models.

#### Constructor

```cpp
explicit IKFastSolver(const std::string& robot_name)
```

Creates a solver instance for the specified robot model.

**Parameters:**
- `robot_name`: Robot model identifier
  - `"kawasaki_kj125"` or `"kj125"` - Kawasaki KJ125
  - `"yaskawa_gp4"` or `"gp4"` - Yaskawa GP4 (if available)
  - More robots as they are added

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

Compute inverse kinematics for target TCP pose, returning the solution closest to `current_joints`.

**Parameters:**
- `tcp_pose`: Target TCP 6D pose (position + orientation)
- `current_joints`: Current joint angles in radians (used to find closest solution)
- `wrist_config`: Preferred wrist configuration (`WristConfig::NONE`, `FLIP`, or `NO_FLIP`)
- `solution`: [out] Output IK solution

**Returns:**
- `true` if solution found
- `false` if no solution (pose unreachable or at singularity)

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

Compute all possible IK solutions for target pose.

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

Compute forward kinematics - get TCP pose from joint angles.

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

##### getRobotName()

```cpp
std::string getRobotName() const
```

Get the robot model name.

**Returns:** Robot name string

##### getDOF()

```cpp
int getDOF() const
```

Get number of degrees of freedom.

**Returns:** DOF (typically 6 for industrial robots)

##### isValid()

```cpp
bool isValid() const
```

Check if solver is properly initialized.

**Returns:**
- `true` if solver is ready to use
- `false` if robot model not found or initialization failed

**Example:**
```cpp
IKFastSolver solver("unknown_robot");
if (!solver.isValid()) {
    std::cerr << "Failed to initialize solver\n";
}
```

##### getSupportedRobots() (static)

```cpp
static std::vector<std::string> getSupportedRobots()
```

Get list of all supported robot models.

**Returns:** Vector of robot name strings

**Example:**
```cpp
auto robots = IKFastSolver::getSupportedRobots();
for (const auto& robot : robots) {
    std::cout << "- " << robot << "\n";
}
```

---

### 2. Pose6D Structure

Represents 6D pose (position + orientation).

```cpp
struct Pose6D {
    double x, y, z;        // Position in meters
    double rx, ry, rz;     // Orientation in radians (roll-pitch-yaw)

    Pose6D();  // Default constructor (all zeros)
    Pose6D(double x, double y, double z, double rx, double ry, double rz);
};
```

**Members:**
- `x, y, z`: Position in meters (base coordinate frame)
- `rx, ry, rz`: Orientation in radians
  - `rx`: Roll (rotation around X-axis)
  - `ry`: Pitch (rotation around Y-axis)
  - `rz`: Yaw (rotation around Z-axis)

**Coordinate System:** Z-up right-handed coordinate system

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

Contains IK solution result.

```cpp
struct IKSolution {
    std::vector<double> joints;  // Joint angles in radians
    bool is_valid;               // Solution validity flag
    double error;                // FK verification error (meters)

    IKSolution();  // Default constructor
};
```

**Members:**
- `joints`: Joint angles in radians (size == DOF)
- `is_valid`: `true` if solution is valid
- `error`: Forward kinematics verification error in meters
  - Measures how accurately the solution reaches the target
  - Typical good solutions: < 0.001 m (1mm)

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

Wrist configuration preference for IK solving.

```cpp
enum class WristConfig {
    NONE = 0,      // No preference (find closest solution)
    FLIP = 1,      // Prefer wrist flip configuration
    NO_FLIP = 2    // Prefer wrist no-flip configuration
};
```

**Values:**
- `WristConfig::NONE`: Default - finds solution closest to `current_joints`
- `WristConfig::FLIP`: Prefers wrist flip configuration (if multiple solutions available)
- `WristConfig::NO_FLIP`: Prefers wrist no-flip configuration

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
| Kawasaki KJ125 | ✅ Available | `kj125_ikfast` | Fully tested |
| Yaskawa GP4 | 🔄 Coming Soon | `gp4_ikfast` | - |
| Yaskawa GP7 | 🔄 Coming Soon | `gp7_ikfast` | - |

---

## Performance

- **IK Computation**: < 1ms (typical)
- **FK Verification**: < 0.1ms
- **Accuracy**: < 0.1mm position error (typical)
- **Solutions per Pose**: 0-8 (6DOF robots)

---

## Error Handling

### No Solution Found

`solveIK()` returns `false` when:
- Target pose is outside robot workspace
- Pose is at or near singularity
- Invalid robot name

**Example:**
```cpp
if (!solver.solveIK(target, current, WristConfig::NONE, solution)) {
    std::cerr << "No IK solution found\n";
    // Check if target is reachable
    // Try different current_joints as seed
}
```

### High FK Error

If `solution.error` is large (> 0.001 m):
- Solution may be at joint limits
- Numerical instability
- Consider trying different seed joints

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

See [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md) for instructions on generating and integrating new robot models.

---

## License

Apache License 2.0

Copyright 2025 IKFast Robotics Team

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

---

## Contact

- **Issues**: GitHub Issues
- **Email**: robotics@your-company.com

## Citation

```bibtex
@software{ikfast_robotics,
  title = {IKFast Robotics Library},
  author = {IKFast Generator Team},
  year = {2025},
  url = {https://github.com/your-company/ikfast-robotics}
}
```
