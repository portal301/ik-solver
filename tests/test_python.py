"""
IKFast Python Test - All IK modes (all solutions, config, nearest) with FK verification
"""

# Auto-select correct ikfast_solver .pyd (conda vs system) before import
import os
import sys
import importlib.util

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
BIN_DIR = os.path.join(PROJECT_ROOT, 'bin')

def _load_ikfast_solver():
    is_conda = (
        'conda' in sys.version.lower()
        or os.environ.get('CONDA_PREFIX') is not None
        or 'Continuum' in sys.version
    )
    # For conda, enable modified DLL search so add_dll_directory takes effect
    if is_conda:
        os.environ.setdefault('CONDA_DLL_SEARCH_MODIFICATION_ENABLE', '1')
    candidates = []
    # Prefer bin dual-builds if present
    if is_conda:
        candidates.append(os.path.join(BIN_DIR, 'ikfast_solver.cp310-win_amd64.conda.pyd'))
    else:
        candidates.append(os.path.join(BIN_DIR, 'ikfast_solver.cp310-win_amd64.sys.pyd'))
    # Fallbacks
    candidates.append(os.path.join(BIN_DIR, 'ikfast_solver.pyd'))
    candidates.append(os.path.join(PROJECT_ROOT, 'ikfast_solver.pyd'))

    for p in candidates:
        if os.path.isfile(p):
            spec = importlib.util.spec_from_file_location('ikfast_solver', p)
            if spec and spec.loader:
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                sys.modules['ikfast_solver'] = mod
                return mod
    raise ImportError('ikfast_solver .pyd not found in expected locations')

# Prefer bundled BLAS/LAPACK over conda MKL by prepending PATH BEFORE numpy import
# Resolve project root (ik-solver) from tests directory
ROBOTS_DIR = os.path.join(PROJECT_ROOT, 'src', 'robots')
LIB_DIR = os.path.join(PROJECT_ROOT, 'lib')
VCPKG_BIN = os.path.join(os.environ.get("VCPKG_ROOT", r"C:\dev\vcpkg"), "installed", "x64-windows", "bin")
EXTRA_PATHS = [BIN_DIR, LIB_DIR, ROBOTS_DIR]
if os.path.isdir(VCPKG_BIN):
    EXTRA_PATHS.insert(0, VCPKG_BIN)

# Allow isolation to avoid conda MKL loading (set IKFAST_ISOLATE_PATH=1)
isolate_path = os.environ.get("IKFAST_ISOLATE_PATH") == "1"
if isolate_path:
    system_root = os.environ.get("SystemRoot", r"C:\Windows")
    system32 = os.path.join(system_root, "System32")
    base_paths = [p for p in EXTRA_PATHS if os.path.isdir(p)]
    base_paths.append(system32)
    base_paths.append(system_root)
    os.environ["PATH"] = ";".join(base_paths)
else:
    # Prepend our DLL directories so they are found before conda's MKL/OpenBLAS
    os.environ["PATH"] = ";".join(p for p in EXTRA_PATHS if os.path.isdir(p)) + ";" + os.environ.get("PATH", "")

# Ensure Python can import ikfast_solver.pyd from project root
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# Windows DLL search path (Python 3.8+)
if hasattr(os, 'add_dll_directory'):
    if os.path.isdir(VCPKG_BIN):
        os.add_dll_directory(os.path.abspath(VCPKG_BIN))
    if os.path.isdir(BIN_DIR):
        os.add_dll_directory(os.path.abspath(BIN_DIR))
    if os.path.isdir(ROBOTS_DIR):
        os.add_dll_directory(os.path.abspath(ROBOTS_DIR))
    if os.path.isdir(LIB_DIR):
        os.add_dll_directory(os.path.abspath(LIB_DIR))

# Import numpy FIRST (before loading ikfast_solver which preloads local LAPACK)
import numpy as np

# Now load ikfast_solver module (this will preload local LAPACK)
ikfast_solver = _load_ikfast_solver()


def main():
    robot_name = "kj125"

    # Target pose in Euler angles (for display)
    x, y, z = 0.5, 0.0, 0.3

    # Build 4x4 transformation (12 elements) with identity rotation
    tcp_pose = np.array([
        1, 0, 0, x,
        0, 1, 0, y,
        0, 0, 1, z
    ], dtype=np.float64)

    print("=" * 60)
    print("IKFast Python Test - All IK Modes")
    print("=" * 60)
    print(f"Loading IK plugins from: {ROBOTS_DIR}")

    try:
        ikfast_solver.load_ik_plugins(ROBOTS_DIR)
        print("✓ IK plugins loaded successfully\n")
    except Exception as e:
        print(f"✗ Failed to load IK plugins: {e}")
        return

    run_tests(robot_name, tcp_pose)


def run_tests(robot_name, tcp_pose):
    print("=" * 60)
    print(f"Testing Robot: {robot_name.upper()}")
    print("=" * 60)

    dof = ikfast_solver.get_num_joints(robot_name)
    if dof <= 0:
        print(f"✗ Robot '{robot_name}' not loaded or not available")
        return
    print(f"DOF: {dof}\n")

    # Extract position from matrix for display
    x, y, z = tcp_pose[3], tcp_pose[7], tcp_pose[11]
    rx, ry, rz = 0.0, 0.0, 0.0

    print("Target TCP Pose:")
    print(f"  Position: ({x:.3f}, {y:.3f}, {z:.3f}) m")
    print(f"  Orientation: ({rx:.3f}, {ry:.3f}, {rz:.3f}) rad (Euler ZYX)")
    print()

    # 1) IK all solutions
    print("--- IKU_SolveIK (all solutions) ---")
    try:
        solutions, is_solvable = ikfast_solver.solve_ik(robot_name, tcp_pose)
        print(f"Found {len(solutions)} solution(s), is_solvable: {is_solvable}")
        for idx, sol in enumerate(solutions):
            deg = np.rad2deg(sol)
            print(f"  Solution {idx+1}: {[f'{d:8.2f}°' for d in deg]}")
        if is_solvable and solutions:
            fk_trans, _ = ikfast_solver.compute_fk(robot_name, solutions[0])
            # Compare position from matrix
            target_pos = np.array([tcp_pose[3], tcp_pose[7], tcp_pose[11]])
            err = np.linalg.norm(target_pos - fk_trans)
            print(f"  FK check (sol1): pos=({fk_trans[0]:.6f}, {fk_trans[1]:.6f}, {fk_trans[2]:.6f}), err={err:.3e} m")
    except Exception as e:
        print(f"  ✗ IK all error: {e}")
    print()

    # 2) IK with config
    print("--- IKU_SolveIKWithConfig (config-based) ---")
    configs = [
        (0, 3, 4, "Right-Down-NoFlip"),
        (1, 3, 4, "Left-Down-NoFlip"),
        (0, 2, 4, "Right-Up-NoFlip"),
        (1, 2, 5, "Left-Up-Flip"),
    ]
    for shoulder, elbow, wrist, name in configs:
        print(f"Configuration: {name}")
        try:
            joints, is_solvable = ikfast_solver.solve_ik_with_config(
                robot_name, tcp_pose, shoulder, elbow, wrist
            )
            if is_solvable:
                deg = np.rad2deg(joints)
                for j, (r, d) in enumerate(zip(joints, deg)):
                    print(f"  J{j+1}: {d:8.2f}° ({r:8.4f} rad)")
                fk_trans, _ = ikfast_solver.compute_fk(robot_name, joints)
                target_pos = np.array([tcp_pose[3], tcp_pose[7], tcp_pose[11]])
                err = np.linalg.norm(target_pos - fk_trans)
                print(f"  FK: pos=({fk_trans[0]:.6f}, {fk_trans[1]:.6f}, {fk_trans[2]:.6f}), err={err:.3e} m")
            else:
                print("  ✗ No solution found")
        except Exception as e:
            print(f"  ✗ Error: {e}")
        print()

    # 3) IK nearest to current joints
    print("--- IKU_SolveIKWithJoint (nearest to current) ---")
    current = np.zeros(dof, dtype=np.float64)
    try:
        joints, is_solvable = ikfast_solver.solve_ik_with_joint(
            robot_name, tcp_pose, current
        )
        if is_solvable:
            deg = np.rad2deg(joints)
            print("  ✓ Solution (nearest):")
            for j, (r, d) in enumerate(zip(joints, deg)):
                print(f"    J{j+1}: {d:8.2f}° ({r:8.4f} rad)")
            fk_trans, _ = ikfast_solver.compute_fk(robot_name, joints)
            target_pos = np.array([tcp_pose[3], tcp_pose[7], tcp_pose[11]])
            err = np.linalg.norm(target_pos - fk_trans)
            print(f"    FK: pos=({fk_trans[0]:.6f}, {fk_trans[1]:.6f}, {fk_trans[2]:.6f}), err={err:.3e} m")
        else:
            print("  ✗ No solution found")
    except Exception as e:
        print(f"  ✗ Error: {e}")
    print()

    print("=" * 60)
    print("All IK tests completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
