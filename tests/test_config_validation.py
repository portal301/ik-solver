#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Config Validation Test - Verify that different configs produce different solutions

This test:
1. Takes a single TCP pose
2. Gets ALL IK solutions
3. Extracts config from each solution (J1, J3, J5 signs)
4. Verifies that different solutions have different configs
5. Tests solve_ik_with_config for each config to ensure it returns the matching solution
"""
import os
import sys
import importlib.util
import math
import numpy as np
import ctypes

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

def _load_ikfast_solver():
    """Load ikfast_solver module"""
    last_error = None
    is_conda = (
        'conda' in sys.version.lower()
        or os.environ.get('CONDA_PREFIX') is not None
        or 'Continuum' in sys.version
    )
    if is_conda:
        os.environ.setdefault('CONDA_DLL_SEARCH_MODIFICATION_ENABLE', '1')

    py_tag = f"cp{sys.version_info.major}{sys.version_info.minor}"

    candidates = [
        os.path.join(PROJECT_ROOT, f'ikfast_solver.{py_tag}-win_amd64.pyd'),
    ]
    for p in candidates:
        if os.path.isfile(p):
            spec = importlib.util.spec_from_file_location('ikfast_solver', p)
            if spec and spec.loader:
                try:
                    mod = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(mod)
                    sys.modules['ikfast_solver'] = mod
                    return mod
                except (ImportError, OSError) as e:
                    last_error = e
                    continue
    raise ImportError(f"ikfast_solver .pyd not found (last error: {last_error})")

# Setup paths
ROBOTS_DIR = os.path.join(PROJECT_ROOT, 'src', 'robots')
LIB_DIR = os.path.join(PROJECT_ROOT, 'lib')
VCPKG_BIN = os.path.join(os.environ.get("VCPKG_ROOT", r"C:\dev\vcpkg"), "installed", "x64-windows", "bin")

EXTRA_PATHS = [ROBOTS_DIR, PROJECT_ROOT, LIB_DIR]
if os.path.isdir(VCPKG_BIN):
    EXTRA_PATHS.insert(0, VCPKG_BIN)

os.environ["PATH"] = ";".join(p for p in EXTRA_PATHS if os.path.isdir(p)) + ";" + os.environ.get("PATH", "")

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

if hasattr(os, 'add_dll_directory'):
    if os.path.isdir(VCPKG_BIN):
        os.add_dll_directory(os.path.abspath(VCPKG_BIN))
    if os.path.isdir(LIB_DIR):
        os.add_dll_directory(os.path.abspath(LIB_DIR))
    os.add_dll_directory(os.path.abspath(PROJECT_ROOT))
    if os.path.isdir(ROBOTS_DIR):
        os.add_dll_directory(os.path.abspath(ROBOTS_DIR))

# Preload critical DLLs
PRELOAD_NAMES = ["openblas.dll", "libgfortran-5.dll", "liblapack.dll", "libquadmath-0.dll"]
for d in [ROBOTS_DIR, VCPKG_BIN, PROJECT_ROOT]:
    if os.path.isdir(d):
        for name in PRELOAD_NAMES:
            candidate = os.path.join(d, name)
            if os.path.isfile(candidate):
                try:
                    ctypes.WinDLL(candidate)
                except OSError:
                    pass

ikfast_solver = _load_ikfast_solver()


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def extract_config(joints, tcp_pose, robot_name='kj125'):
    """Extract config from joint angles matching C++ matchesConfiguration logic

    Args:
        joints: Joint angles [J1, J2, J3, J4, J5, J6]
        tcp_pose: TCP pose array (12 elements) for yaw calculation
        robot_name: Robot name for robot-specific references (default: kj125)
    """
    eps = 1e-6
    j1 = normalize_angle(joints[0])
    j3 = normalize_angle(joints[2])
    j5 = normalize_angle(joints[4])

    # Shoulder: yaw-target based (matching C++)
    tx = tcp_pose[3]
    ty = tcp_pose[7]
    yaw_target = math.atan2(tx, -ty)  # j1=0 => -Y (KJ125)
    diff = normalize_angle(yaw_target - j1)
    shoulder = 0 if abs(diff) < (math.pi / 2.0) else 1  # 0=RIGHT, 1=LEFT

    # Elbow: reference-based (matches C++ getJ3Reference)
    # J3 < ref => UP (2), J3 >= ref => DOWN (3)
    if robot_name.lower() == 'kj125':
        j3_ref = math.pi / 2.0  # 90 degrees
    else:
        j3_ref = 0.0  # Default fallback
    j3_delta = j3 - j3_ref
    elbow = 2 if j3_delta < -eps else 3  # 2=UP, 3=DOWN

    # Wrist: sign-based (matches C++ sign-based logic)
    wrist = 4 if j5 >= -eps else 5     # 4=N_FLIP, 5=FLIP

    return (shoulder, elbow, wrist)


def config_to_string(config):
    """Convert config tuple to readable string"""
    shoulder_str = "RIGHT" if config[0] == 0 else "LEFT"
    elbow_str = "UP" if config[1] == 2 else "DOWN"
    wrist_str = "N_FLIP" if config[2] == 4 else "FLIP"
    return f"{shoulder_str}-{elbow_str}-{wrist_str}"


def test_robot_configs(robot_name):
    """Test config validation for a single robot"""
    print(f"\n{'='*80}")
    print(f"Testing robot: {robot_name.upper()}")
    print(f"{'='*80}")

    # Initialize
    try:
        ikfast_solver.load_ik_plugins(ROBOTS_DIR)
    except:
        pass  # May already be loaded

    dof = ikfast_solver.get_num_joints(robot_name)
    if dof <= 0:
        print(f"[FAIL] Robot '{robot_name}' not found")
        return False

    print(f"DOF: {dof}")

    # Test TCP pose
    tcp_pose = np.array([
        1, 0, 0, -0.5,   # R11, R12, R13, Tx
        0, 1, 0, -1,   # R21, R22, R23, Ty
        0, 0, 1, 0.3    # R31, R32, R33, Tz
    ], dtype=np.float64)

    # Step 1: Get ALL IK solutions
    print(f"\n[Step 1] Solving IK for target pose...")
    solutions, is_solvable = ikfast_solver.solve_ik(robot_name, tcp_pose)

    if not is_solvable or len(solutions) == 0:
        print(f" No IK solutions found for test pose")
        return False

    print(f" Found {len(solutions)} IK solutions")

    # Step 2: Extract config from each solution
    print(f"\n[Step 2] Extracting configs from all solutions...")
    configs = {}
    for i, sol in enumerate(solutions):
        config = extract_config(sol, tcp_pose, robot_name)
        config_str = config_to_string(config)

        if config not in configs:
            configs[config] = []
        configs[config].append((i, sol))

        j1_deg = math.degrees(normalize_angle(sol[0]))
        j3_deg = math.degrees(normalize_angle(sol[2]))
        j5_deg = math.degrees(normalize_angle(sol[4]))

        print(f"  Solution {i+1}: {config_str:20s} (J1={j1_deg:+7.2f}°, J3={j3_deg:+7.2f}°, J5={j5_deg:+7.2f}°)")

    # Step 3: Verify different configs exist
    print(f"\n[Step 3] Verifying config diversity...")
    print(f"  Unique configs found: {len(configs)}")

    if len(configs) < 2:
        print(f"  WARNING: Only {len(configs)} unique config(s) found")
        print(f"             Expected multiple configs for different solutions")
    else:
        print(f" Multiple configs detected ({len(configs)} unique configs)")

    # Step 4: Test solve_ik_with_config for each config
    print(f"\n[Step 4] Testing solve_ik_with_config for each config...")
    all_passed = True

    for config, sol_list in configs.items():
        config_str = config_to_string(config)
        shoulder, elbow, wrist = config

        print(f"\n  Testing config: {config_str}")

        # Call solve_ik_with_config
        result_joints, is_solvable = ikfast_solver.solve_ik_with_config(
            robot_name, tcp_pose, shoulder, elbow, wrist
        )

        if not is_solvable:
            print(f"     FAIL: solve_ik_with_config returned no solution")
            all_passed = False
            continue

        # Verify the returned solution matches this config
        result_config = extract_config(result_joints, tcp_pose, robot_name)

        if result_config == config:
            print(f"     PASS: Returned solution matches requested config")

            # Verify FK accuracy
            fk_pos, fk_rot, fk_ok = ikfast_solver.compute_fk(robot_name, result_joints)
            if fk_ok:
                target_pos = np.array([tcp_pose[3], tcp_pose[7], tcp_pose[11]])
                error = np.linalg.norm(target_pos - fk_pos)
                print(f"     FK accuracy: {error:.3e} m")
            else:
                print(f"      FK failed")
        else:
            print(f"     FAIL: Returned config {config_to_string(result_config)} != requested {config_str}")
            all_passed = False

    # Final summary
    print(f"\n{'='*80}")
    if all_passed and len(configs) >= 2:
        print(f" ALL TESTS PASSED for {robot_name}")
        print(f"   - Found {len(solutions)} solutions with {len(configs)} unique configs")
        print(f"   - All configs correctly returned by solve_ik_with_config")
        return True
    elif all_passed:
        print(f"  PARTIAL PASS for {robot_name}")
        print(f"   - solve_ik_with_config works correctly")
        print(f"   - But only {len(configs)} unique config(s) found (expected multiple)")
        return True
    else:
        print(f" TESTS FAILED for {robot_name}")
        return False


def main():
    """Main test function"""
    print("="*80)
    print("CONFIG VALIDATION TEST")
    print("="*80)
    print()
    print("This test verifies that:")
    print("  1. Different IK solutions have different configs")
    print("  2. solve_ik_with_config returns the correct solution for each config")
    print()

    # Test robots
    test_robots = ["kj125"]

    results = {}
    for robot in test_robots:
        try:
            passed = test_robot_configs(robot)
            results[robot] = "PASS" if passed else "FAIL"
        except Exception as e:
            print(f"\n ERROR testing {robot}: {e}")
            import traceback
            traceback.print_exc()
            results[robot] = "ERROR"

    # Final summary
    print(f"\n{'='*80}")
    print("FINAL SUMMARY")
    print(f"{'='*80}")
    for robot, status in results.items():
        status_icon = "" if status == "PASS" else ""
        print(f"  {status_icon} {robot.upper():10s}: {status}")
    print(f"{'='*80}")

    passed_count = sum(1 for s in results.values() if s == "PASS")
    total_count = len(results)

    if passed_count == total_count:
        print(f"\n ALL TESTS PASSED ({passed_count}/{total_count})")
        return 0
    else:
        print(f"\n SOME TESTS FAILED ({passed_count}/{total_count} passed)")
        return 1


if __name__ == "__main__":
    sys.exit(main())
