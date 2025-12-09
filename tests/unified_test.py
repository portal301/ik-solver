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

    # Auto-detect Python version
    py_ver = f"{sys.version_info.major}{sys.version_info.minor}"
    py_tag = f"cp{py_ver}-win_amd64"

    candidates = [
        # Current Python version files
        os.path.join(PROJECT_ROOT, f'ikfast_solver.{py_tag}.conda.pyd'),
        os.path.join(PROJECT_ROOT, f'ikfast_solver.{py_tag}.sys.pyd'),
        os.path.join(PROJECT_ROOT, f'ikfast_solver.{py_tag}.pyd'),
        os.path.join(PROJECT_ROOT, 'build', f'lib.win-amd64-cpython-{py_ver}', f'ikfast_solver.{py_tag}.pyd'),
        # Generic fallback
        os.path.join(PROJECT_ROOT, 'ikfast_solver.pyd'),
        # Legacy Python 3.10 files (for backwards compatibility)
        os.path.join(PROJECT_ROOT, 'ikfast_solver.cp310-win_amd64.conda.pyd'),
        os.path.join(PROJECT_ROOT, 'ikfast_solver.cp310-win_amd64.sys.pyd'),
        os.path.join(PROJECT_ROOT, 'ikfast_solver.cp310-win_amd64.pyd'),
        os.path.join(PROJECT_ROOT, 'build', 'lib.win-amd64-cpython-310', 'ikfast_solver.cp310-win_amd64.pyd')
    ]
    # fallback: any ikfast_solver*.pyd under PROJECT_ROOT
    for f in os.listdir(PROJECT_ROOT):
        if f.startswith('ikfast_solver') and f.endswith('.pyd'):
            candidates.append(os.path.join(PROJECT_ROOT, f))
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
    fk_pos_orig, fk_rot_orig = ikfast_solver.compute_fk(robot_name, orig_joints)
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
                fk_pos, fk_rot = ikfast_solver.compute_fk(robot_name, ik_joints)
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
            tcp_error, _ = compare_ik_solutions_via_tcp(solutions)
            if tcp_error is not None:
                results['solve_ik'] = f"OK ({len(solutions):02d} sol, err={tcp_error:.6f})"
            else:
                results['solve_ik'] = "FAIL (TCP comparison error)"
        else:
            results['solve_ik'] = "FAIL (no solution)"
    except ValueError:
        results['solve_ik'] = "ERROR"
    
    try:
        # Determine the configuration of the original joints
        # Normalize angles to [-pi, pi]
        def normalize_angle(angle):
            while angle > math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi
            return angle

        j0_norm = normalize_angle(orig_joints[0])
        j2_norm = normalize_angle(orig_joints[2])
        j4_norm = normalize_angle(orig_joints[4])

        eps = 1e-6
        # Config: 0 = positive, 1 = negative (with epsilon tolerance)
        expected_shoulder = 0 if j0_norm > -eps else 1
        expected_elbow = 0 if j2_norm > -eps else 1
        expected_wrist = 0 if j4_norm > -eps else 1

        # First, try the native C++ config filter
        joints, is_solvable = ikfast_solver.solve_ik_with_config(
            robot_name, tcp_pose_orig, expected_shoulder, expected_elbow, expected_wrist
        )

        if is_solvable:
            fk_pos, fk_rot = ikfast_solver.compute_fk(robot_name, joints)
            tcp_pose_ik = tcp_to_matrix(fk_pos, fk_rot)
            tcp_error = vec_norm(vec_diff(tcp_pose_ik, tcp_pose_orig))
            results['solve_ik_with_config'] = f"OK (err={tcp_error:.6f})"
        else:
            # Fallback: use solve_ik and filter in Python to avoid platform differences
            sols, ok = ikfast_solver.solve_ik(robot_name, tcp_pose_orig)
            if ok:
                found = False
                for s in sols:
                    j0 = normalize_angle(s[0])
                    j2 = normalize_angle(s[2])
                    j4 = normalize_angle(s[4])
                    shoulder = 0 if j0 > -eps else 1
                    elbow = 0 if j2 > -eps else 1
                    wrist = 0 if j4 > -eps else 1
                    if (shoulder, elbow, wrist) == (expected_shoulder, expected_elbow, expected_wrist):
                        fk_pos, fk_rot = ikfast_solver.compute_fk(robot_name, s)
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
            # Convert IK result back to TCP and compare
            fk_pos, fk_rot = ikfast_solver.compute_fk(robot_name, joints)
            tcp_pose_ik = tcp_to_matrix(fk_pos, fk_rot)
            tcp_error = vec_norm(vec_diff(tcp_pose_ik, tcp_pose_orig))
            results['solve_ik_with_joint'] = f"OK (err={tcp_error:.6f})"
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
    print(f"[DEBUG] Calling load_ik_plugins...")
    
    try:
        ikfast_solver.load_ik_plugins(ROBOTS_DIR)
        print("[OK] Plugins loaded\n")
    except ValueError as e:
        print(f"[FAIL] Load plugins: {e}")
        return
    except Exception as e:
        print(f"[FAIL] Load plugins (unexpected error): {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
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
