# IKFast Unified Tests

## Overview
Unified test suite for all robots and all IK solver functions:
- **Python** (`unified_test.py`): Tests all robots via C extension
- **C#** (`Program.cs`): P/Invoke tests for Unity/C# integration

## Features
- Tests all robots simultaneously (no hard-coded model)
- Validates three IK modes:
  1. `solve_ik`: All solutions
  2. `solve_ik_with_config`: Specific configuration  
  3. `solve_ik_with_joint`: Nearest to current
- Input: Random FK within joint limits → IK test → Compare to original joints
- Output: Success rate + error metrics per robot

## Running Tests

### Python Test
```powershell
cd C:\dev\ikfast-generator\ik-solver
python tests\unified_test.py
```
*Requires Python extension `bin\ikfast_solver.cp310-win_amd64.sys.pyd` (build output from `build_python.bat`)*

### C# Test
```powershell
cd C:\dev\ikfast-generator\ik-solver\tests
dotnet build -c Release -p:Platform=x64
dotnet run -c Release -p:Platform=x64
```
*Uses `IKFastUnity_x64.dll` (build output from `src\build_unity_dll.bat`)*

## Test Output Format
```
================================================================================
IKFast Unified Test - All Robots, All IK Functions (Python)
================================================================================

[OK] Plugins loaded

Testing 13 robot(s):
--------------------------------------------------------------------------------
gp25                 | solve_ik              =OK (04 sol, err=0.000123)solve_ik_with_config=OK (err=0.000456)solve_ik_with_joint =OK (err=0.000789)
gp50                 | solve_ik              =OK (02 sol, err=0.000234)solve_ik_with_config=OK (err=0.000345)solve_ik_with_joint =OK (err=0.000567)
...
--------------------------------------------------------------------------------
Result: 13/13 robots passed all tests
================================================================================
```

## Console Output (Minimal)
- One line per robot with function results
- Error metrics for each solution
- Final pass/fail summary

## Key Test Parameters
- **Random Joints**: Uniformly sampled within `joint_limits.json` bounds
- **FK Input**: Original joints → TCP position/orientation
- **IK Test**: TCP pose → Inverse kinematics → Joint comparison
- **Error Metric**: Euclidean distance in joint space between original and solution joints
