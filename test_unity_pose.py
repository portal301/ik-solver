#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Test Unity TCP pose to see which configs have solutions"""
import os
import sys
import importlib.util
import math
import numpy as np
import ctypes

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '.'))

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
    """Extract config from joint angles using yaw-target (matching C++)"""
    eps = 1e-6
    j1 = normalize_angle(joints[0])
    j3 = normalize_angle(joints[2])
    j5 = normalize_angle(joints[4])

    # Shoulder: yaw-target based (matching C++)
    tx = tcp_pose[3]
    ty = tcp_pose[7]
    yaw_target = math.atan2(tx, -ty)  # j1=0 => -Y (KJ125)
    diff = normalize_angle(yaw_target - j1)
    shoulder = 0 if abs(diff) < (math.pi / 2.0) else 1  # 0=FRONT, 1=REAR

    # Elbow: reference-based
    if robot_name.lower() == 'kj125':
        j3_ref = math.pi / 2.0  # 90 degrees
    else:
        j3_ref = 0.0
    j3_delta = j3 - j3_ref
    elbow = 0 if j3_delta < -eps else 1  # 0=UP (< ref), 1=DOWN (>= ref)

    # Wrist: sign-based
    wrist = 0 if j5 >= -eps else 1     # 0=N_FLIP (positive), 1=FLIP (negative)

    return (shoulder, elbow, wrist)


def config_to_string(config):
    """Convert config tuple to readable string"""
    shoulder_str = "RIGHT" if config[0] == 0 else "LEFT"
    elbow_str = "UP" if config[1] == 2 else "DOWN"
    wrist_str = "N_FLIP" if config[2] == 4 else "FLIP"
    return f"{shoulder_str}-{elbow_str}-{wrist_str}"


def main():
    print("="*80)
    print("Testing Unity TCP Pose")
    print("="*80)

    # Initialize
    ikfast_solver.load_ik_plugins(ROBOTS_DIR)

    # Unity TCP pose (4x4 matrix, row-major)
    # 0.94360	0.00555	-0.33105	0.07574
    # -0.32753	-0.13075	-0.93575	-0.68577
    # -0.04847	0.99140	-0.12155	0.87847
    # 0.00000	0.00000	0.00000	1.00000

    # Convert to 12-element array (row-major, 3x4)
    tcp_pose = np.array([
        0.94360, 0.00555, -0.33105, 0.07574,    # R11, R12, R13, Tx
        -0.32753, -0.13075, -0.93575, -0.68577, # R21, R22, R23, Ty
        -0.04847, 0.99140, -0.12155, 0.87847    # R31, R32, R33, Tz
    ], dtype=np.float64)

    print("\nTCP Pose:")
    print(f"  R11={tcp_pose[0]:.5f}, R12={tcp_pose[1]:.5f}, R13={tcp_pose[2]:.5f}, Tx={tcp_pose[3]:.5f}")
    print(f"  R21={tcp_pose[4]:.5f}, R22={tcp_pose[5]:.5f}, R23={tcp_pose[6]:.5f}, Ty={tcp_pose[7]:.5f}")
    print(f"  R31={tcp_pose[8]:.5f}, R32={tcp_pose[9]:.5f}, R33={tcp_pose[10]:.5f}, Tz={tcp_pose[11]:.5f}")

    # Solve IK
    print("\n" + "="*80)
    print("Solving IK...")
    solutions, is_solvable = ikfast_solver.solve_ik("kj125", tcp_pose)

    if not is_solvable or len(solutions) == 0:
        print("  No IK solutions found!")
        return 1

    print(f"  Found {len(solutions)} IK solutions\n")

    # Analyze each solution
    configs = {}
    for i, sol in enumerate(solutions):
        config = extract_config(sol, tcp_pose, "kj125")
        config_str = config_to_string(config)

        if config not in configs:
            configs[config] = []
        configs[config].append((i, sol))

        j1_deg = math.degrees(normalize_angle(sol[0]))
        j2_deg = math.degrees(normalize_angle(sol[1]))
        j3_deg = math.degrees(normalize_angle(sol[2]))
        j4_deg = math.degrees(normalize_angle(sol[3]))
        j5_deg = math.degrees(normalize_angle(sol[4]))
        j6_deg = math.degrees(normalize_angle(sol[5]))

        print(f"Solution {i}:")
        print(f"  Config: {config_str}")
        print(f"  J1={j1_deg:+7.2f}°, J2={j2_deg:+7.2f}°, J3={j3_deg:+7.2f}°")
        print(f"  J4={j4_deg:+7.2f}°, J5={j5_deg:+7.2f}°, J6={j6_deg:+7.2f}°")
        print()

    print("="*80)
    print(f"Summary: {len(configs)} unique configurations")
    print("="*80)
    for config, sol_list in configs.items():
        config_str = config_to_string(config)
        print(f"  {config_str}: {len(sol_list)} solution(s)")
    print("="*80)

    # Test specific config: RIGHT-UP-N_FLIP (0, 2, 4)
    print("\nTesting Unity's requested config: RIGHT-UP-OUT")
    print("  Assuming OUT = N_FLIP = 4")

    result_joints, is_solvable = ikfast_solver.solve_ik_with_config(
        "kj125", tcp_pose, 0, 2, 4  # shoulder=0 (RIGHT), elbow=2 (UP), wrist=4 (N_FLIP)
    )

    if is_solvable:
        print("  OK Solution found!")
        result_config = extract_config(result_joints, tcp_pose, "kj125")
        print(f"  Config: {config_to_string(result_config)}")
        j1_deg = math.degrees(normalize_angle(result_joints[0]))
        j3_deg = math.degrees(normalize_angle(result_joints[2]))
        j5_deg = math.degrees(normalize_angle(result_joints[4]))
        print(f"  J1={j1_deg:+7.2f}°, J3={j3_deg:+7.2f}°, J5={j5_deg:+7.2f}°")
    else:
        print("  X No solution found for FRONT-UP-N_FLIP")
        print("  This matches Unity's error!")

    return 0


if __name__ == "__main__":
    sys.exit(main())
