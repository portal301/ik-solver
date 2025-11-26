// ikfast_solver.cpp
#include <windows.h>
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>
#include <filesystem>
#include <cmath>
#include <limits>

#include "ikfast.h"   // 방금 만든 ikfast.h

namespace fs = std::filesystem;

using IkReal = double; // ikfast.h에서도 typedef double IkReal; 이라서 맞춤

// Configuration enums
enum class PoseConfig {
    UNKNOWN = -1,
    RIGHT = 0,   // J1 범위: [-180, 0) or [0, 180) 로 구분 가능
    LEFT = 1,
    UP = 0,      // J3 + offset > 0
    DOWN = 1,    // J3 + offset < 0
    N_FLIP = 0,  // J5 >= 0
    FLIP = 1     // J5 < 0
};

struct RobotConfiguration {
    PoseConfig shoulder;  // RIGHT or LEFT
    PoseConfig elbow;     // UP or DOWN
    PoseConfig wrist;     // N_FLIP or FLIP
    
    bool operator==(const RobotConfiguration& other) const {
        return shoulder == other.shoulder && 
               elbow == other.elbow && 
               wrist == other.wrist;
    }
};

struct RobotIkPlugin {
    HMODULE dll = nullptr;
    ikfast::IkFastFunctions<IkReal> fns{};
    int num_joints = 0;
    int num_free = 0;
    std::string name;
};

static std::unordered_map<std::string, RobotIkPlugin> g_plugins;

// --- 내부 유틸 ---

template <typename T>
T load_symbol(HMODULE dll, const char* name) {
    FARPROC fp = GetProcAddress(dll, name);
    if (!fp) {
        std::cerr << "GetProcAddress failed for symbol: " << name << std::endl;
        return nullptr;
    }
    return reinterpret_cast<T>(fp);
}

// 파일명에서 로봇 이름 추출: "gp25_ikfast.dll" -> "gp25"
static std::string robot_name_from_path(const fs::path& p) {
    std::string stem = p.stem().string(); // "gp25_ikfast"
    const std::string suffix = "_ikfast";
    if (stem.size() > suffix.size() &&
        stem.compare(stem.size() - suffix.size(), suffix.size(), suffix) == 0) {
        return stem.substr(0, stem.size() - suffix.size());
    }
    // 규칙 안 맞으면 전체 stem 그대로 반환
    return stem;
}

// --- 1) robots 폴더 스캔해서 모든 DLL 로드 ---

bool load_ik_plugins(const std::string& robots_dir) {
    g_plugins.clear();

    fs::path dir(robots_dir);
    if (!fs::exists(dir) || !fs::is_directory(dir)) {
        std::cerr << "robots directory not found: " << robots_dir << std::endl;
        return false;
    }

    for (auto& entry : fs::directory_iterator(dir)) {
        if (!entry.is_regular_file())
            continue;

        fs::path p = entry.path();
        if (p.extension() != ".dll")
            continue;

        std::string robot = robot_name_from_path(p);

        HMODULE dll = LoadLibraryW(p.wstring().c_str());
        if (!dll) {
            std::wcerr << L"Failed to LoadLibrary: " << p.wstring() << std::endl;
            continue;
        }

        RobotIkPlugin plugin;
        plugin.dll = dll;
        plugin.name = robot;

        // ikfast.h에서 정의한 함수 포인터 타입 사용
        using IkFns = ikfast::IkFastFunctions<IkReal>;

        plugin.fns._ComputeIk            = load_symbol<IkFns::ComputeIkFn>(dll, "ComputeIk");
        plugin.fns._ComputeFk            = load_symbol<IkFns::ComputeFkFn>(dll, "ComputeFk");
        plugin.fns._GetNumFreeParameters = load_symbol<IkFns::GetNumFreeParametersFn>(dll, "GetNumFreeParameters");
        plugin.fns._GetFreeParameters    = load_symbol<IkFns::GetFreeParametersFn>(dll, "GetFreeParameters");
        plugin.fns._GetNumJoints         = load_symbol<IkFns::GetNumJointsFn>(dll, "GetNumJoints");
        plugin.fns._GetIkRealSize        = load_symbol<IkFns::GetIkRealSizeFn>(dll, "GetIkRealSize");
        plugin.fns._GetIkFastVersion     = load_symbol<IkFns::GetIkFastVersionFn>(dll, "GetIkFastVersion");
        plugin.fns._GetIkType            = load_symbol<IkFns::GetIkTypeFn>(dll, "GetIkType");
        plugin.fns._GetKinematicsHash    = load_symbol<IkFns::GetKinematicsHashFn>(dll, "GetKinematicsHash");

        // 필수 함수 확인
        if (!plugin.fns._ComputeIk || !plugin.fns._GetNumJoints) {
            std::cerr << "DLL missing mandatory IKFast symbols, skipping: "
                      << p.string() << std::endl;
            FreeLibrary(dll);
            continue;
        }

        plugin.num_joints = plugin.fns._GetNumJoints();
        if (plugin.fns._GetNumFreeParameters) {
            plugin.num_free = plugin.fns._GetNumFreeParameters();
        }

        std::cout << "Loaded IK plugin: " << robot
                  << " (joints=" << plugin.num_joints
                  << ", free=" << plugin.num_free << ")\n";

        g_plugins.emplace(robot, std::move(plugin));
    }

    return !g_plugins.empty();
}

// --- Configuration 분류 함수 ---

// 6축 로봇의 joint 값으로부터 configuration 판별
RobotConfiguration classifyConfiguration(const std::vector<IkReal>& joints) {
    RobotConfiguration config;
    
    if (joints.size() < 6) {
        config.shoulder = PoseConfig::UNKNOWN;
        config.elbow = PoseConfig::UNKNOWN;
        config.wrist = PoseConfig::UNKNOWN;
        return config;
    }
    
    // J1 (shoulder): [-π, 0) = RIGHT, [0, π) = LEFT
    // 정규화된 각도 기준
    IkReal j1_normalized = std::fmod(joints[0] + M_PI, 2.0 * M_PI);
    if (j1_normalized < 0) j1_normalized += 2.0 * M_PI;
    j1_normalized -= M_PI;
    
    config.shoulder = (j1_normalized < 0) ? PoseConfig::RIGHT : PoseConfig::LEFT;
    
    // J3 (elbow): UP if J3 > -π/2, DOWN if J3 < -π/2
    // 일반적으로 elbow up/down은 J3 값으로 구분
    config.elbow = (joints[2] > -M_PI_2) ? PoseConfig::UP : PoseConfig::DOWN;
    
    // J5 (wrist): N_FLIP if J5 >= 0, FLIP if J5 < 0
    config.wrist = (joints[4] >= 0) ? PoseConfig::N_FLIP : PoseConfig::FLIP;
    
    return config;
}

// 두 joint 배열 간의 거리 계산 (가중치 없는 유클리드 거리)
IkReal computeJointDistance(const std::vector<IkReal>& j1, const std::vector<IkReal>& j2) {
    if (j1.size() != j2.size()) {
        return std::numeric_limits<IkReal>::max();
    }
    
    IkReal sum = 0.0;
    for (size_t i = 0; i < j1.size(); ++i) {
        IkReal diff = j1[i] - j2[i];
        // 각도 차이를 [-π, π] 범위로 정규화
        while (diff > M_PI) diff -= 2.0 * M_PI;
        while (diff < -M_PI) diff += 2.0 * M_PI;
        sum += diff * diff;
    }
    
    return std::sqrt(sum);
}

// --- 2) 특정 로봇에 대해 IK 계산하는 래퍼 ---

bool solveIK(
    const std::string& robot_name,
    const IkReal* eetrans,      // [3]
    const IkReal* eerot,        // [9]
    const IkReal* freeparams,   // free DOF 값들 (없으면 nullptr)
    std::vector<std::vector<IkReal>>& out_solutions
) {
    auto it = g_plugins.find(robot_name);
    if (it == g_plugins.end()) {
        std::cerr << "Robot IK plugin not loaded: " << robot_name << std::endl;
        return false;
    }

    RobotIkPlugin& plugin = it->second;
    if (!plugin.fns._ComputeIk) {
        std::cerr << "ComputeIk function not available for robot: "
                  << robot_name << std::endl;
        return false;
    }

    ikfast::IkSolutionList<IkReal> solutions;
    // freeparams 개수는 plugin.num_free에 맞게 사용 (필요하면 체크)
    bool ok = plugin.fns._ComputeIk(eetrans, eerot, freeparams, solutions);
    if (!ok) {
        return false;
    }

    size_t nsol = solutions.GetNumSolutions();
    out_solutions.clear();
    out_solutions.reserve(nsol);

    std::vector<IkReal> solvec;
    std::vector<IkReal> freevec(plugin.num_free, 0.0); // 필요시 입력 freeparams에서 채움

    for (size_t i = 0; i < nsol; ++i) {
        const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        solvec.clear();
        sol.GetSolution(solvec, freevec);
        out_solutions.push_back(solvec);
    }

    return !out_solutions.empty();
}

// --- 3) Configuration 필터링 + Nearest 선택 기능 추가 ---

struct IkSolveResult {
    bool is_solvable = false;
    std::vector<IkReal> joint_solution;
    RobotConfiguration config;
    IkReal distance = std::numeric_limits<IkReal>::max();
};

bool solveIKWithConfig(
    const std::string& robot_name,
    const IkReal* eetrans,                    // [3]
    const IkReal* eerot,                      // [9]
    const IkReal* current_joints,             // [6] 현재 joint 값
    const RobotConfiguration* desired_config, // 원하는 configuration (nullptr면 무시)
    const IkReal* freeparams,                 // free DOF 값들 (없으면 nullptr)
    IkSolveResult& result
) {
    auto it = g_plugins.find(robot_name);
    if (it == g_plugins.end()) {
        std::cerr << "Robot IK plugin not loaded: " << robot_name << std::endl;
        return false;
    }

    RobotIkPlugin& plugin = it->second;
    if (!plugin.fns._ComputeIk) {
        std::cerr << "ComputeIk function not available for robot: "
                  << robot_name << std::endl;
        return false;
    }

    ikfast::IkSolutionList<IkReal> solutions;
    bool ok = plugin.fns._ComputeIk(eetrans, eerot, freeparams, solutions);
    if (!ok || solutions.GetNumSolutions() == 0) {
        result.is_solvable = false;
        return false;
    }

    // 현재 joint 값을 vector로 변환
    std::vector<IkReal> current_joint_vec;
    if (current_joints != nullptr) {
        current_joint_vec.assign(current_joints, current_joints + plugin.num_joints);
    }

    // 모든 해 중에서 조건에 맞는 최적의 해 찾기
    IkReal best_distance = std::numeric_limits<IkReal>::max();
    std::vector<IkReal> best_solution;
    RobotConfiguration best_config;
    bool found = false;

    std::vector<IkReal> freevec(plugin.num_free, 0.0);
    
    for (size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        std::vector<IkReal> candidate;
        sol.GetSolution(candidate, freevec);
        
        // Configuration 분류
        RobotConfiguration candidate_config = classifyConfiguration(candidate);
        
        // desired_config가 지정된 경우, 일치하는지 확인
        if (desired_config != nullptr) {
            if (!(candidate_config == *desired_config)) {
                continue;  // configuration이 일치하지 않으면 스킵
            }
        }
        
        // 현재 joint와의 거리 계산
        IkReal distance = 0.0;
        if (current_joints != nullptr && !current_joint_vec.empty()) {
            distance = computeJointDistance(candidate, current_joint_vec);
        }
        
        // 더 가까운 해를 찾으면 업데이트
        if (distance < best_distance) {
            best_distance = distance;
            best_solution = candidate;
            best_config = candidate_config;
            found = true;
        }
    }

    if (found) {
        result.is_solvable = true;
        result.joint_solution = best_solution;
        result.config = best_config;
        result.distance = best_distance;
        return true;
    }

    result.is_solvable = false;
    return false;
}

// --- 4) 여러 configuration 우선순위로 시도하는 래퍼 ---

bool solveIKWithConfigPriority(
    const std::string& robot_name,
    const IkReal* eetrans,                               // [3]
    const IkReal* eerot,                                 // [9]
    const IkReal* current_joints,                        // [6] 현재 joint 값
    const std::vector<RobotConfiguration>& config_priority, // 우선순위 순서대로 시도할 config 리스트
    const IkReal* freeparams,                            // free DOF 값들 (없으면 nullptr)
    IkSolveResult& result
) {
    // config_priority 순서대로 시도
    for (const auto& config : config_priority) {
        if (solveIKWithConfig(robot_name, eetrans, eerot, current_joints, &config, freeparams, result)) {
            return true;
        }
    }
    
    // 모든 우선순위 config에서 해를 못 찾으면 config 제한 없이 시도
    return solveIKWithConfig(robot_name, eetrans, eerot, current_joints, nullptr, freeparams, result);
}
