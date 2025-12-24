/**
 * ikfast_unity.cpp
 * Wrapper for IKFast Unity integration
 *
 * Provides C-style exported functions for Unity to call IKFast core functions.
 * Functions include initialization, IK/FK solving, and joint limit retrieval.
 */


#include "ikfast_core.hpp"
#include <windows.h>
#include <algorithm>
#include <vector>

using IkReal = ikcore::IkReal;

extern "C" {

// 초기화: robots 디렉토리의 DLL 로딩
__declspec(dllexport)
int IKU_Init(const char* robots_dir_utf8) {
    try {
        return ikcore::load_ik_plugins(robots_dir_utf8) ? 1 : 0;
    } catch (...) {
        return 0;
    }
}

// 로봇별 DOF 문의
__declspec(dllexport)
int IKU_GetNumJoints(const char* robot_name_utf8) {
    return ikcore::get_num_joints(robot_name_utf8);
}

// 로봇별 조인트 리미트 조회
// out_lower / out_upper: 길이 max_joints 배열, 반환값은 채운 joint 개수
__declspec(dllexport)
int IKU_GetJointLimits(
    const char* robot_name_utf8,
    double* out_lower,
    double* out_upper,
    int max_joints
) {
    std::vector<ikcore::JointLimit> limits;
    if (!ikcore::get_joint_limits(robot_name_utf8, limits) || limits.empty()) {
        return 0;
    }

    int ncopy = std::min<int>(max_joints, static_cast<int>(limits.size()));
    for (int i = 0; i < ncopy; ++i) {
        out_lower[i] = limits[i].lower;
        out_upper[i] = limits[i].upper;
    }
    return ncopy;
}

// IK 계산 - Returns ALL solutions
// tcp_pose: [12] = 4x4 transformation matrix (row-major, first 3 rows)
// out_solutions: [max_solutions][dof] 연속 메모리 (row-major)
// return: 실제로 채운 솔루션 개수
__declspec(dllexport)
int IKU_SolveIK(
    const char* robot_name_utf8,
    const double* tcp_pose,      // [12]: R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz
    double* out_solutions,       // len = max_solutions * dof
    int max_solutions
) {
    std::vector<ikcore::IkSolutionData> sols;

    bool ok = ikcore::solveIK(robot_name_utf8, tcp_pose, sols);
    if (!ok) return 0;

    int dof = ikcore::get_num_joints(robot_name_utf8);
    if (dof <= 0) return 0;

    int ncopy = std::min<int>(max_solutions, (int)sols.size());
    for (int i = 0; i < ncopy; ++i) {
        const auto& v = sols[i].joints;
        int copylen = std::min<int>(dof, (int)v.size());
        double* dst = out_solutions + i * dof;
        for (int j = 0; j < copylen; ++j)
            dst[j] = v[j];
    }
    return ncopy;
}

// FK 계산
// out_eetrans: [3] position
// out_eerot: [9] rotation matrix (row-major)
__declspec(dllexport)
int IKU_ComputeFK(
    const char* robot_name_utf8,
    const double* joints,    // len = dof
    double* out_eetrans,     // len = 3
    double* out_eerot        // len = 9
) {
    int dof = ikcore::get_num_joints(robot_name_utf8);
    if (dof <= 0) return 0;

    bool ok = ikcore::computeFK(robot_name_utf8, joints, out_eetrans, out_eerot);
    return ok ? 1 : 0;
}

// Configuration-based IK solver
// tcp_pose: [12] = 4x4 transformation matrix (row-major, first 3 rows)
// shoulder_config: 0=FRONT, 1=BACK
// elbow_config: 2=UP, 3=DOWN
// wrist_config: 4=N_FLIP, 5=FLIP
// out_joints: [dof] output joint angles
// Returns: 1 if solution found, 0 otherwise
__declspec(dllexport)
int IKU_SolveIKWithConfig(
    const char* robot_name_utf8,
    const double* tcp_pose,       // [12]: R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz
    int shoulder_config,          // 0=FRONT, 1=BACK
    int elbow_config,             // 2=UP, 3=DOWN
    int wrist_config,             // 4=N_FLIP, 5=FLIP
    double* out_joints,           // [dof] output
    int* out_is_solvable          // 1=found, 0=not found
) {
    bool is_solvable = false;
    bool ok = ikcore::solveIKWithConfig(
        robot_name_utf8,
        tcp_pose,
        shoulder_config,
        elbow_config,
        wrist_config,
        out_joints,
        &is_solvable
    );

    *out_is_solvable = is_solvable ? 1 : 0;
    return ok ? 1 : 0;
}

// Config-based IK with continuity - Returns solution matching config nearest to current_joints
// tcp_pose: [12] = 4x4 transformation matrix
// current_joints: [dof] current joint angles for continuity
// out_joints: [dof] output joint angles
// Returns: 1 if solution found, 0 otherwise
__declspec(dllexport)
int IKU_SolveIKWithConfigEx(
    const char* robot_name_utf8,
    const double* tcp_pose,       // [12]
    int shoulder_config,          // 0=FRONT, 1=BACK
    int elbow_config,             // 2=UP, 3=DOWN
    int wrist_config,             // 4=N_FLIP, 5=FLIP
    const double* current_joints, // [dof] for continuity
    double* out_joints,           // [dof] output
    int* out_is_solvable          // 1=found, 0=not found
) {
    bool is_solvable = false;
    bool ok = ikcore::solveIKWithConfig(
        robot_name_utf8,
        tcp_pose,
        shoulder_config,
        elbow_config,
        wrist_config,
        out_joints,
        &is_solvable,
        current_joints  // Pass current_joints for continuity
    );

    *out_is_solvable = is_solvable ? 1 : 0;
    return ok ? 1 : 0;
}

// Joint-based IK solver - Returns nearest solution to current_joints
// tcp_pose: [12] = 4x4 transformation matrix (row-major, first 3 rows)
// current_joints: [dof] current joint angles
// out_joints: [dof] output joint angles (nearest solution)
// Returns: 1 if solution found, 0 otherwise
__declspec(dllexport)
int IKU_SolveIKWithJoint(
    const char* robot_name_utf8,
    const double* tcp_pose,       // [12]: R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz
    const double* current_joints, // [dof] current joint angles
    double* out_joints,           // [dof] output
    int* out_is_solvable          // 1=found, 0=not found
) {
    bool is_solvable = false;
    bool ok = ikcore::solveIKWithJoint(
        robot_name_utf8,
        tcp_pose,
        current_joints,
        out_joints,
        &is_solvable
    );

    *out_is_solvable = is_solvable ? 1 : 0;
    return ok ? 1 : 0;
}

} // extern "C"
