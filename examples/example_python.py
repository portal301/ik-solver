"""
IKFast Solver Python 사용 예제

이 예제는 ikfast_solver 모듈을 사용하여 로봇의 IK/FK를 계산하는 방법을 보여줍니다.
<<<<<<< HEAD

요구사항:
  - Python 3.10, 3.11, 또는 3.12 (권장: 3.12)

배포 구조:
  example_python.py (이 파일)
  ikfast_solver.pyd (Python 버전에 맞는 .pyd 파일)
  robots/
    ├── kawasaki/
    ├── yaskawa/
    └── (모든 의존성 DLL: libgfortran-5.dll, liblapack.dll 등)
=======
>>>>>>> f4b66b8 (예제 추가 및 오류 수정)
"""

import os
import sys
import numpy as np
<<<<<<< HEAD
from scipy.spatial.transform import Rotation

# ============================================================================
# 경로 설정 (프로젝트 구조와 배포 구조 모두 대응)
# ============================================================================
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# michelo_library 로드 시도 (있으면 우선 사용)
try:
    from michelo_library.robot.kinematics.enums import PoseConfigEnum, RobotModelEnum
    HAS_MICHELO = True
except ImportError:
    HAS_MICHELO = False
    # 상위 디렉토리에서 찾기
    temp_dir = script_dir
    for _ in range(4):
        temp_dir = os.path.dirname(temp_dir)
        if os.path.isdir(os.path.join(temp_dir, "michelo_library")):
            sys.path.insert(0, temp_dir)
            try:
                from michelo_library.robot.kinematics.enums import PoseConfigEnum, RobotModelEnum
                HAS_MICHELO = True
                break
            except ImportError:
                continue

# robots 경로 결정 (배포/프로젝트 구조 모두 대응)
robots_candidates = [
    os.path.join(script_dir, "robots"),                                      # 배포: 같은 디렉토리
    os.path.join(os.path.dirname(script_dir), "robots"),                    # 프로젝트: 상위 (examples 형제)
    os.path.join(os.path.dirname(script_dir), "src", "robots"),             # 프로젝트: src 하위
]
robots_dir = next((p for p in robots_candidates if os.path.isdir(p)), None)
if not robots_dir:
    raise RuntimeError(f"robots 디렉토리를 찾을 수 없습니다. 시도한 경로:\n  " + "\n  ".join(robots_candidates))

# DLL 경로 설정
dll_dirs = [script_dir, robots_dir]

# VCPKG DLL (선택사항)
vcpkg_root = os.environ.get("VCPKG_ROOT")
if vcpkg_root:
    vcpkg_bin = os.path.join(vcpkg_root, "installed", "x64-windows", "bin")
    if os.path.isdir(vcpkg_bin):
        dll_dirs.insert(0, vcpkg_bin)

# DLL 경로 등록
if hasattr(os, 'add_dll_directory'):
    for dll_dir in dll_dirs:
        if os.path.isdir(dll_dir):
            os.add_dll_directory(dll_dir)

os.environ["PATH"] = ";".join(dll_dirs) + ";" + os.environ.get("PATH", "")

# ikfast_solver 로드
import ikfast_solver
ikfast_solver.load_ik_plugins(robots_dir)
print(f"[OK] IK plugins loaded from: {robots_dir}")
if HAS_MICHELO:
    print("[OK] michelo_library available\n")
else:
    print()
=======

# 프로젝트 루트를 sys.path에 추가 (예제 파일이 examples/ 디렉토리에 있음)
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, project_root)

# 1. DLL 검색 경로 설정 (numpy를 먼저 import)
vcpkg_bin = os.path.join(os.environ.get("VCPKG_ROOT", r"C:\dev\vcpkg"), "installed", "x64-windows", "bin")
robots_dir = os.path.join(project_root, "src", "robots")

if hasattr(os, 'add_dll_directory'):
    if os.path.isdir(vcpkg_bin):
        os.add_dll_directory(vcpkg_bin)
    os.add_dll_directory(robots_dir)

# 2. ikfast_solver 모듈 import 및 초기화
import ikfast_solver

ikfast_solver.load_ik_plugins(robots_dir)
print("[OK] IK plugins loaded successfully\n")
>>>>>>> f4b66b8 (예제 추가 및 오류 수정)

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

print(f"목표 TCP 자세:")
print(f"  위치: ({tcp_pose[3]:.3f}, {tcp_pose[7]:.3f}, {tcp_pose[11]:.3f}) m")
print(f"  회전: Identity (0°)\n")

# 5. IK 계산 - 모든 솔루션
print("=" * 60)
print("IK 계산 - 모든 솔루션")
print("=" * 60)

solutions, is_solvable = ikfast_solver.solve_ik(robot_name, tcp_pose)

if is_solvable:
    print(f"찾은 솔루션 개수: {len(solutions)}")

    # 모든 솔루션 출력
    for i, sol in enumerate(solutions):
        sol_deg = np.rad2deg(sol)
        print(f"  솔루션 {i+1}: {[f'{d:7.2f}°' for d in sol_deg]}")

    # 첫 번째 솔루션으로 FK 검증
<<<<<<< HEAD
    fk_trans, fk_rot, fk_ok = ikfast_solver.compute_fk(robot_name, solutions[0])
    if fk_ok:
        target_pos = np.array([tcp_pose[3], tcp_pose[7], tcp_pose[11]])
        error = np.linalg.norm(target_pos - fk_trans)
        print(f"\nFK 검증 (솔루션 1):")
        print(f"  위치: ({fk_trans[0]:.6f}, {fk_trans[1]:.6f}, {fk_trans[2]:.6f}) m")
        print(f"  오차: {error:.3e} m")
    else:
        print("\nFK 검증 실패")
=======
    fk_trans, fk_rot = ikfast_solver.compute_fk(robot_name, solutions[0])
    target_pos = np.array([tcp_pose[3], tcp_pose[7], tcp_pose[11]])
    error = np.linalg.norm(target_pos - fk_trans)
    print(f"\nFK 검증 (솔루션 1):")
    print(f"  위치: ({fk_trans[0]:.6f}, {fk_trans[1]:.6f}, {fk_trans[2]:.6f}) m")
    print(f"  오차: {error:.3e} m")
>>>>>>> f4b66b8 (예제 추가 및 오류 수정)
else:
    print("솔루션을 찾을 수 없습니다.")

print()

# 6. 특정 Configuration으로 IK 계산
print("=" * 60)
print("IK 계산 - 특정 Configuration")
print("=" * 60)

configs = [
<<<<<<< HEAD
    (0, 1, 0, "Front-Down-NoFlip"),
    (1, 1, 0, "Back-Down-NoFlip"),
    (0, 0, 0, "Front-Up-NoFlip"),
    (1, 0, 1, "Back-Up-Flip"),
=======
    (0, 3, 4, "Right-Down-NoFlip"),
    (1, 3, 4, "Left-Down-NoFlip"),
    (0, 2, 4, "Right-Up-NoFlip"),
    (1, 2, 5, "Left-Up-Flip"),
>>>>>>> f4b66b8 (예제 추가 및 오류 수정)
]

for shoulder, elbow, wrist, name in configs:
    joints, is_solvable = ikfast_solver.solve_ik_with_config(
        robot_name, tcp_pose, shoulder, elbow, wrist
    )

    if is_solvable:
        joints_deg = np.rad2deg(joints)
        print(f"{name}: {[f'{d:7.2f}°' for d in joints_deg]}")
    else:
        print(f"{name}: 솔루션 없음")

print()

# 7. 현재 관절 각도에서 가장 가까운 IK 솔루션
print("=" * 60)
print("IK 계산 - 현재 위치에서 가장 가까운 솔루션")
print("=" * 60)

current_joints = np.zeros(dof, dtype=np.float64)
print(f"현재 관절 각도: {[f'{d:7.2f}°' for d in np.rad2deg(current_joints)]}")

joints, is_solvable = ikfast_solver.solve_ik_with_joint(
    robot_name, tcp_pose, current_joints
)

if is_solvable:
    joints_deg = np.rad2deg(joints)
    print(f"가장 가까운 솔루션: {[f'{d:7.2f}°' for d in joints_deg]}")

    # FK 검증
<<<<<<< HEAD
    fk_trans, fk_rot, fk_ok = ikfast_solver.compute_fk(robot_name, joints)
    if fk_ok:
        target_pos = np.array([tcp_pose[3], tcp_pose[7], tcp_pose[11]])
        error = np.linalg.norm(target_pos - fk_trans)
        print(f"FK 검증 오차: {error:.3e} m")
    else:
        print("FK 검증 실패")
=======
    fk_trans, fk_rot = ikfast_solver.compute_fk(robot_name, joints)
    target_pos = np.array([tcp_pose[3], tcp_pose[7], tcp_pose[11]])
    error = np.linalg.norm(target_pos - fk_trans)
    print(f"FK 검증 오차: {error:.3e} m")
>>>>>>> f4b66b8 (예제 추가 및 오류 수정)
else:
    print("솔루션을 찾을 수 없습니다.")

print()

# 8. FK 단독 사용 예제
print("=" * 60)
print("FK 계산 - 관절 각도에서 TCP 자세 구하기")
print("=" * 60)

test_joints = np.array([0, 0, np.pi/2, 0, np.pi/4, 0], dtype=np.float64)
print(f"입력 관절 각도: {[f'{d:7.2f}°' for d in np.rad2deg(test_joints)]}")

<<<<<<< HEAD
fk_trans, fk_rot, fk_ok = ikfast_solver.compute_fk(robot_name, test_joints)
if fk_ok:
    print(f"\nTCP 위치: ({fk_trans[0]:.6f}, {fk_trans[1]:.6f}, {fk_trans[2]:.6f}) m")
    print("TCP 회전 행렬 (row-major):")
    fk_rot_matrix = fk_rot.reshape(3, 3)
    for row in fk_rot_matrix:
        print(f"  [{row[0]:8.5f}, {row[1]:8.5f}, {row[2]:8.5f}]")
else:
    print("FK 계산 실패")
=======
fk_trans, fk_rot = ikfast_solver.compute_fk(robot_name, test_joints)
print(f"\nTCP 위치: ({fk_trans[0]:.6f}, {fk_trans[1]:.6f}, {fk_trans[2]:.6f}) m")
print("TCP 회전 행렬 (row-major):")
fk_rot_matrix = fk_rot.reshape(3, 3)
for row in fk_rot_matrix:
    print(f"  [{row[0]:8.5f}, {row[1]:8.5f}, {row[2]:8.5f}]")
>>>>>>> f4b66b8 (예제 추가 및 오류 수정)

print("\n" + "=" * 60)
print("예제 완료!")
print("=" * 60)
