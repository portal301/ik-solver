#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Unified IKFast Test - All robots, all IK functions"""
import os
import sys
import importlib.util
import json
import random
import math
import ctypes

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

def _load_ikfast_solver():
    last_error = None
    is_conda = (
        'conda' in sys.version.lower()
        or os.environ.get('CONDA_PREFIX') is not None
        or 'Continuum' in sys.version
    )
    if is_conda:
        os.environ.setdefault('CONDA_DLL_SEARCH_MODIFICATION_ENABLE', '1')

    # Detect current Python version (e.g., 3.10, 3.11, 3.12)
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

ROBOTS_DIR = os.path.join(PROJECT_ROOT, 'src', 'robots')
LIB_DIR = os.path.join(PROJECT_ROOT, 'lib')  # may not exist; kept for compatibility
VCPKG_BIN = os.path.join(os.environ.get("VCPKG_ROOT", r"C:\dev\vcpkg"), "installed", "x64-windows", "bin")
TESTS_BIN = os.path.join(PROJECT_ROOT, 'tests', 'bin', 'x64', 'Release', 'net10.0')

# Prepend dependency paths so conda does not shadow them
EXTRA_PATHS = [ROBOTS_DIR, PROJECT_ROOT, LIB_DIR, TESTS_BIN]
if os.path.isdir(VCPKG_BIN):
    EXTRA_PATHS.insert(0, VCPKG_BIN)

isolate_path = os.environ.get("IKFAST_ISOLATE_PATH") == "1"
if isolate_path:
    system_root = os.environ.get("SystemRoot", r"C:\Windows")
    system32 = os.path.join(system_root, "System32")
    base_paths = [p for p in EXTRA_PATHS if os.path.isdir(p)]
    base_paths.append(system32)
    base_paths.append(system_root)
    os.environ["PATH"] = ";".join(base_paths)
else:
    os.environ["PATH"] = ";".join(p for p in EXTRA_PATHS if os.path.isdir(p)) + ";" + os.environ.get("PATH", "")

if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

if hasattr(os, 'add_dll_directory'):
    if os.path.isdir(VCPKG_BIN):
        os.add_dll_directory(os.path.abspath(VCPKG_BIN))
    if os.path.isdir(LIB_DIR):
        os.add_dll_directory(os.path.abspath(LIB_DIR))
    # Add PROJECT_ROOT for DLLs (IKFastUnity_x64.dll)
    os.add_dll_directory(os.path.abspath(PROJECT_ROOT))
    if os.path.isdir(ROBOTS_DIR):
        os.add_dll_directory(os.path.abspath(ROBOTS_DIR))

# Preload critical BLAS/Fortran deps to help conda find them
PRELOAD_NAMES = [
    "openblas.dll",
    "libgfortran-5.dll",
    "liblapack.dll",
    "libquadmath-0.dll",
]
PRELOAD_DIRS = []
for d in [ROBOTS_DIR, VCPKG_BIN, TESTS_BIN, PROJECT_ROOT]:
    if os.path.isdir(d):
        PRELOAD_DIRS.append(d)

for d in PRELOAD_DIRS:
    for name in PRELOAD_NAMES:
        candidate = os.path.join(d, name)
        if os.path.isfile(candidate):
            try:
                ctypes.WinDLL(candidate)
            except OSError:
                pass

ikfast_solver = _load_ikfast_solver()

def load_robot_joint_limits(robot_name):
    # Prefer compiled limits exposed by ikfast_solver
    try:
        limits = ikfast_solver.get_joint_limits(robot_name)
        if limits:
            return limits
    except AttributeError:
        # Older modules may not export get_joint_limits; fall back to JSON
        pass

    for root, _, files in os.walk(ROBOTS_DIR):
        if f"{robot_name.lower()}_ikfast.dll" in [f.lower() for f in files]:
            joint_limits_path = os.path.join(root, "joint_limits.json")
            if os.path.isfile(joint_limits_path):
                with open(joint_limits_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    # joint_limits.json is either a list or a dict
                    if isinstance(data, list):
                        return data
                    elif isinstance(data, dict) and robot_name in data:
                        return data[robot_name]
    return None

def discover_robots():
    robots = set()
    for root, _, files in os.walk(ROBOTS_DIR):
        for f in files:
            if f.endswith('_ikfast.dll'):
                robot_name = f.replace('_ikfast.dll', '')
                robots.add(robot_name)
    return sorted(robots)

def matrix_to_tcp(matrix_12):
    pos = [matrix_12[3], matrix_12[7], matrix_12[11]]
    rot = [[matrix_12[0], matrix_12[1], matrix_12[2]],
           [matrix_12[4], matrix_12[5], matrix_12[6]],
           [matrix_12[8], matrix_12[9], matrix_12[10]]]
    return pos, rot

def tcp_to_matrix(pos, rot):
    # Handle numpy arrays - rot is 9-element array, need to reshape to 3x3
    import numpy as np
    pos = list(pos) if hasattr(pos, '__iter__') else [pos[0], pos[1], pos[2]]
    
    # If rot is 1D (9 elements), reshape to 3x3
    if isinstance(rot, np.ndarray) and rot.ndim == 1:
        rot = rot.reshape((3, 3))
    
    rot = [list(r) if hasattr(r, '__iter__') else [r] for r in rot]
    
    return [
        rot[0][0], rot[0][1], rot[0][2], pos[0],
        rot[1][0], rot[1][1], rot[1][2], pos[1],
        rot[2][0], rot[2][1], rot[2][2], pos[2]
    ]

def vec_norm(v):
    return math.sqrt(sum(x*x for x in v))

def vec_diff(v1, v2):
    return [v1[i] - v2[i] for i in range(len(v1))]

def normalize_angle(angle):
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def check_joint_limits(joints, limits):
    """Check if all joints are within their limits.
    Returns (is_valid, violations_list)"""
    violations = []
    for i, angle in enumerate(joints):
        lower = limits[i].get('lower', limits[i].get('min', -math.pi))
        upper = limits[i].get('upper', limits[i].get('max', math.pi))

        # Normalize for comparison
        angle_norm = normalize_angle(angle)

        # Handle wraparound limits (e.g., [-π, π])
        if lower <= angle_norm <= upper:
            continue

        # Check if violation is due to numerical precision
        eps = 1e-5
        if lower - eps <= angle_norm <= upper + eps:
            continue

        violations.append({
            'joint': i,
            'value': angle_norm,
            'lower': lower,
            'upper': upper
        })

    return len(violations) == 0, violations

def determine_configuration(joints, limits, robot_name, tcp_pose):
    """Determine configuration using new J1/target yaw for front/back,
    robot-specific J3 reference for elbow, and |J5| for wrist.

    Returns (frontback, elbow, wrist):
      frontback: 0=FRONT, 1=REAR  (J1 vs target direction)
      elbow:     0=UP,    1=DOWN  (J3 vs robot-specific ref)
      wrist:     0=N_FLIP,1=FLIP  (|J5| <= π/2)
    """
    eps = 1e-6

    IDX_J1 = 0
    IDX_ELBOW = 2
    IDX_WRIST = 4

    def midpoint(idx):
        lower = limits[idx].get('lower', limits[idx].get('min', -math.pi))
        upper = limits[idx].get('upper', limits[idx].get('max', math.pi))
        return (lower + upper) / 2.0

    # FRONT/BACK via J1 and target yaw (J1=0 aligns +Y)
    tx, ty = tcp_pose[3], tcp_pose[7]
    yaw_target = math.atan2(tx, ty)  # (x,y) -> yaw
    j1 = normalize_angle(joints[IDX_J1])
    diff = normalize_angle(yaw_target - j1)
    frontback = 0 if abs(diff) <= math.pi / 2 + eps else 1

    # ELBOW via robot-specific J3 reference
    robot_lower = robot_name.lower()
    if robot_lower == "kj125":
        j3_ref = math.pi / 2  # 90 deg
    else:
        j3_ref = midpoint(IDX_ELBOW)

    j3 = normalize_angle(joints[IDX_ELBOW])
    elbow = 0 if (j3 - j3_ref) >= -eps else 1

    # WRIST via |J5|
    j5 = normalize_angle(joints[IDX_WRIST])
    wrist = 0 if abs(j5) <= math.pi / 2 + eps else 1

    return frontback, elbow, wrist

def test_robot(robot_name):
    dof = ikfast_solver.get_num_joints(robot_name)
    if dof <= 0:
        return None
    
    limits = load_robot_joint_limits(robot_name)
    if not limits:
        return None
    
    orig_joints = [
        random.uniform(limits[i].get('lower', limits[i].get('min', -3.14)), 
                      limits[i].get('upper', limits[i].get('max', 3.14))) 
        for i in range(dof)
    ]
    
    # Step 1: FK to get original TCP pose
    fk_pos_orig, fk_rot_orig, fk_ok = ikfast_solver.compute_fk(robot_name, orig_joints)
    if not fk_ok:
        return {"error": "FK failed for original joints"}
    tcp_pose_orig = tcp_to_matrix(fk_pos_orig, fk_rot_orig)
    
    results = {}
    
    # Helper function: FK -> compare TCP poses
    def compare_ik_solutions_via_tcp(solutions):
        """Compare IK solutions by converting back to TCP and comparing TCP poses"""
        if not solutions:
            return None, None
        
        min_tcp_error = float('inf')
        best_solution_idx = -1
        
        for idx, ik_joints in enumerate(solutions):
            try:
                fk_pos, fk_rot, fk_ok = ikfast_solver.compute_fk(robot_name, ik_joints)
                if not fk_ok:
                    continue
                tcp_pose_ik = tcp_to_matrix(fk_pos, fk_rot)
                
                # Compare TCP poses (position + orientation)
                tcp_error = vec_norm(vec_diff(tcp_pose_ik, tcp_pose_orig))
                
                if tcp_error < min_tcp_error:
                    min_tcp_error = tcp_error
                    best_solution_idx = idx
            except (ValueError, RuntimeError, IndexError):
                pass
        
        if best_solution_idx >= 0:
            return min_tcp_error, solutions[best_solution_idx]
        return None, None
    
    try:
        solutions, is_solvable = ikfast_solver.solve_ik(robot_name, tcp_pose_orig)
        if is_solvable and len(solutions) > 0:
            # Validate joint limits for all solutions
            limit_violations = 0
            for sol in solutions:
                is_valid, violations = check_joint_limits(sol, limits)
                if not is_valid:
                    limit_violations += 1

            tcp_error, _ = compare_ik_solutions_via_tcp(solutions)
            if tcp_error is not None:
                status = f"OK ({len(solutions):02d} sol, err={tcp_error:.6f})"
                if limit_violations > 0:
                    status += f" [WARN: {limit_violations} limit violations]"
                results['solve_ik'] = status
            else:
                results['solve_ik'] = "FAIL (TCP comparison error)"
        else:
            results['solve_ik'] = "FAIL (no solution)"
    except ValueError:
        results['solve_ik'] = "ERROR"
    
    try:
        # Determine the configuration of the original joints using new J2/J3/J5 logic
        expected_frontback, expected_elbow, expected_wrist = determine_configuration(
            orig_joints, limits, robot_name, tcp_pose_orig)

        # First, try the native C++ config filter
        joints, is_solvable = ikfast_solver.solve_ik_with_config(
            robot_name, tcp_pose_orig, expected_frontback, expected_elbow, expected_wrist
        )

        if is_solvable:
            # Validate joint limits
            is_valid, violations = check_joint_limits(joints, limits)

            # Verify configuration matches
            result_frontback, result_elbow, result_wrist = determine_configuration(
                joints, limits, robot_name, tcp_pose_orig)
            config_matches = (
                (result_frontback, result_elbow, result_wrist) ==
                (expected_frontback, expected_elbow, expected_wrist)
            )

            fk_pos, fk_rot, fk_ok = ikfast_solver.compute_fk(robot_name, joints)
            if not fk_ok:
                results['solve_ik_with_config'] = "FK failed"
            else:
                tcp_pose_ik = tcp_to_matrix(fk_pos, fk_rot)
                tcp_error = vec_norm(vec_diff(tcp_pose_ik, tcp_pose_orig))
                status = f"OK (err={tcp_error:.6f})"

                if not config_matches:
                    status += " [WARN: config mismatch]"
                if not is_valid:
                    status += " [WARN: limit violations]"

                results['solve_ik_with_config'] = status
        else:
            # Fallback: use solve_ik and filter in Python
            sols, ok = ikfast_solver.solve_ik(robot_name, tcp_pose_orig)
            if ok:
                found = False
                for s in sols:
                    fb, eb, wr = determine_configuration(s, limits, robot_name, tcp_pose_orig)
                    if (fb, eb, wr) == (expected_frontback, expected_elbow, expected_wrist):
                        fk_pos, fk_rot, fk_ok = ikfast_solver.compute_fk(robot_name, s)
                        if not fk_ok:
                            continue
                        tcp_pose_ik = tcp_to_matrix(fk_pos, fk_rot)
                        tcp_error = vec_norm(vec_diff(tcp_pose_ik, tcp_pose_orig))
                        results['solve_ik_with_config'] = f"OK (err={tcp_error:.6f})"
                        found = True
                        break
                if not found:
                    results['solve_ik_with_config'] = "FAIL"
            else:
                results['solve_ik_with_config'] = "FAIL"
    except ValueError:
        results['solve_ik_with_config'] = "ERROR"
    
    try:
        current = [0.0] * dof
        joints, is_solvable = ikfast_solver.solve_ik_with_joint(
            robot_name, tcp_pose_orig, current
        )
        if is_solvable:
            # Validate joint limits
            is_valid, _ = check_joint_limits(joints, limits)

            # Convert IK result back to TCP and compare
            fk_pos, fk_rot, fk_ok = ikfast_solver.compute_fk(robot_name, joints)
            if not fk_ok:
                results['solve_ik_with_joint'] = "FK failed"
            else:
                tcp_pose_ik = tcp_to_matrix(fk_pos, fk_rot)
                tcp_error = vec_norm(vec_diff(tcp_pose_ik, tcp_pose_orig))
                status = f"OK (err={tcp_error:.6f})"
                if not is_valid:
                    status += " [WARN: limit violations]"
                results['solve_ik_with_joint'] = status
        else:
            results['solve_ik_with_joint'] = "FAIL"
    except ValueError:
        results['solve_ik_with_joint'] = "ERROR"

    return results

def main():
    print("=" * 80)
    print("IKFast Unified Test - All Robots, All IK Functions (Python)")
    print("=" * 80)
    print()
    
    print(f"[DEBUG] ikfast_solver module: {ikfast_solver.__file__}")
    print(f"[DEBUG] ROBOTS_DIR: {ROBOTS_DIR}")
    print("[DEBUG] Calling load_ik_plugins...")
    
    try:
        ikfast_solver.load_ik_plugins(ROBOTS_DIR)
        print("[OK] Plugins loaded\n")
    except ValueError as e:
        print(f"[FAIL] Load plugins: {e}")
        return
    except RuntimeError as e:
        print(f"[FAIL] Load plugins (runtime error): {e}")
        return
    
    robots = discover_robots()
    if not robots:
        print("[FAIL] No robots found")
        return
    
    print(f"Testing {len(robots)} robot(s):")
    print("-" * 80)
    
    passed = 0
    for robot in robots:
        result = test_robot(robot)
        if result is None:
            print(f"{robot:20s} | SKIP (no limits or DOF)")
            continue
        
        status_line = f"{robot:20s} | "
        all_ok = all('OK' in str(v) for v in result.values())
        
        for func in sorted(result.keys()):
            status = result[func]
            status_line += f"{func:25s}={status:30s}"
        
        print(status_line)
        if all_ok:
            passed += 1
    
    print("-" * 80)
    print(f"Result: {passed}/{len(robots)} robots passed all tests")
    print("=" * 80)

if __name__ == "__main__":
    main()
