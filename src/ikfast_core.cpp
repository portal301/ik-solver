// ikfast_core.cpp
#include "ikfast_core.hpp"
#include <windows.h>
#include <unordered_map>
#include <filesystem>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <vector>
#include <array>
#include <limits>
#include <memory>
#include "ikfast.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846  // NOLINT(cppcoreguidelines-macro-usage)
#endif

namespace fs = std::filesystem;

namespace ikcore { // Entire code is within ikcore namespace

using IkReal = double;

/**
 * Robot IK plugin data structure
 * @struct RobotIkPlugin
 */
struct RobotIkPlugin {
    HMODULE dll = nullptr; /// Handle to loaded DLL
    ikfast::IkFastFunctions<IkReal> fns; /// Function pointers
    int num_joints = 0;
    int num_free = 0;
    int ikreal_size = sizeof(IkReal); /// size reported by plugin
    std::string name;
    std::vector<JointLimit> joint_limits;  /// Joint limits loaded from JSON
};

/// Global map of loaded IK plugins (robot name -> plugin data)
static std::unordered_map<std::string, RobotIkPlugin> g_plugins;// NOLINT

/*
==============================================================================
    Helper (Basic)
==============================================================================
*/

/**
 * Load a symbol from a DLL and cast to the specified type.
 
 * @param[in] dll Handle to the loaded DLL.
 * @param[in] name Name of the symbol to load.
 * @return Pointer to the loaded symbol cast to type T, or nullptr on failure.
 */
template <typename T>
T load_symbol(HMODULE dll, const char* name) {
    FARPROC fp = GetProcAddress(dll, name);
    if (!fp) {
        std::cerr << "GetProcAddress failed for symbol: " << name << '\n';
        return nullptr;
    }
    return reinterpret_cast<T>(fp);
}

/**
 * Skip known dependency DLLs that are not IKFast plugins
 *
 * @param[in] p Filesystem path to the IKFast plugin DLL.
 * @return True if the DLL is a known dependency to skip.
 */
static bool is_dependency_dll(const fs::path& p) {
    std::string name = p.filename().string();
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);

    static const std::vector<std::string> skip_names = {
        "liblapack.dll",
        "openblas.dll",
        "libgfortran-5.dll",
        "libgcc_s_seh-1.dll",
        "libquadmath-0.dll",
        "libwinpthread-1.dll"
    };

    return std::find(skip_names.begin(), skip_names.end(), name) != skip_names.end();
}

/**
 * utf8 to wstring conversion for Korean path.
 *
 * @param[in] utf8_str utf8 string to convert.
 * @return Converted wide string.
 */
static std::wstring utf8_to_wstring(const std::string& utf8_str) {
    if (utf8_str.empty()) { return {};
    }
    
    int size_needed = MultiByteToWideChar(CP_UTF8, 0, utf8_str.c_str(), -1, nullptr, 0);
    if (size_needed <= 0) { return {};
    }
    
    std::wstring wstr(size_needed - 1, 0);  // -1 to exclude null terminator
    MultiByteToWideChar(CP_UTF8, 0, utf8_str.c_str(), -1, wstr.data(), size_needed);
    return wstr;
}

/**
 * Extract robot name from the given file path by removing the "_ikfast" suffix if present.
 *
 * @param[in] p Filesystem path to the IKFast plugin DLL.
 * @return Extracted robot name.
 */
static std::string robot_name_from_path(const fs::path& p) {
    std::string stem = p.stem().string(); // "gp25_ikfast"
    const std::string suffix = "_ikfast";
    if (stem.size() > suffix.size() &&
        stem.compare(stem.size() - suffix.size(), suffix.size(), suffix) == 0) {
        return stem.substr(0, stem.size() - suffix.size());
    }
    return stem;
}

/**
 * Normalize robot name to lowercase for consistent lookup
 *
 * @param[in] name Robot name to normalize.
 * @return Lowercase normalized robot name.
 */
static std::string normalize_robot_name(const std::string& name) {
    std::string normalized = name;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), ::tolower);
    return normalized;
}

/*
==============================================================================
    Helpers (Joints)
==============================================================================
*/

// Load joint limits from compiled data
/**
 * Load joint limits for the specified robot from compiled data.
 * 
 * @param[in] robot_name Name of the robot.
 * @param[out] out_limits Output vector to store loaded joint limits.
 * @return True if joint limits were found and loaded, false otherwise.
 */
static bool load_joint_limits_from_data(
    const std::string& robot_name,
    std::vector<JointLimit>& out_limits
) {
    out_limits.clear();

    // Normalize robot name to match generated keys (e.g., gp25_12 -> GP25-12)
    auto normalize_key = [](const std::string& name) {
        std::string key;
        key.reserve(name.size());
        for (unsigned char c : name) {
            if (c == '_') {
                key.push_back('-');
            } else {
                key.push_back(static_cast<char>(std::toupper(c)));
            }
        }
        return key;
    };

    std::string normalized = normalize_key(robot_name);

    // Lookup in compiled data map using normalized key first
    auto it = joint_limits_data::ROBOT_JOINT_LIMITS.find(normalized);
    if (it == joint_limits_data::ROBOT_JOINT_LIMITS.end()) {
        // Fallback: try raw name in case the key already matches
        it = joint_limits_data::ROBOT_JOINT_LIMITS.find(robot_name);
    }
    if (it == joint_limits_data::ROBOT_JOINT_LIMITS.end()) {
        return false;  // Robot not found in compiled data
    }

    out_limits = it->second;
    return !out_limits.empty();
}


/** 
 * Check if a joint is fixed (non-moving)
 * @param[in] limit JointLimit structure to check.
 * @return True if the joint is fixed, false otherwise.
 */
static bool is_fixed_joint(const JointLimit& limit) {
    const IkReal epsilon = 1e-6;  // ~0.00006 degrees
    return std::abs(limit.upper - limit.lower) < epsilon;
}

/**
 * Check if joint angles are within specified limits.
 *
 * @param[in] joints Pointer to array of joint angles.
 * @param[in] num_joints Number of joints.
 * @param[in] limits Vector of JointLimit structures.
 * @return True if all joints are within limits, false otherwise.
 */
static bool check_joint_limits(
    const IkReal* joints,
    int num_joints,
    const std::vector<JointLimit>& limits
) {
    // joint limit 정의가 없으면 그대로 통과
    if (limits.empty()) {
        return true;
    }

    if (num_joints <= 0) {
        std::cerr << "[Joint Limit] Invalid num_joints: " << num_joints << '\n';
        return false;
    }

    // limits 개수가 모자라면 설정 오류로 보고 false
    if (limits.size() < static_cast<size_t>(num_joints)) {
        std::cerr << "[Joint Limit] Mismatch: limits=" << limits.size()
                  << ", expected_joints=" << num_joints
                  << ". Treating as limit violation.\n";
        return false;
    }

    for (int i = 0; i < num_joints; ++i) {
        const JointLimit& limit = limits[i];
        
        // Skip fixed joints (e.g., limits [0, 0])
        if (is_fixed_joint(limit)) {
            continue;  // Fixed joint - don't check limits
        }

        IkReal angle = joints[i];

        // Check if angle is within [lower, upper]
        if (angle < limit.lower || angle > limit.upper) {
            return false;  // Out of bounds
        }
    }

    return true;  // All joints within limits
}

/**
 * Get joint limits for the specified robot.
 * @param[in] robot_name Name of the robot.
 * @param[out] out_limits Output vector to store joint limits.
 * @return True if joint limits were found, false otherwise.
 */
bool get_joint_limits(const std::string& robot_name, std::vector<JointLimit>& out_limits) {
    return load_joint_limits_from_data(robot_name, out_limits);
}

/*
==============================================================================
    Helpers (Math & Transforms)
==============================================================================
*/

/**
 * Multiply two 4x4 transforms in 12-element format.
 * @param[in] T_AB First transform (A to B).
 * @param[in] T_BC Second transform (B to C).
 * @param[out] T_AC Output transform (A to C).
 * @return void
 */
static void multiply_transforms(const Transform& T_AB, const Transform& T_BC, Transform& T_AC) { //NOLINT
    // Extract rotation matrices (3x3)
    std::array<IkReal, 9> R_AB = {T_AB[0], T_AB[1], T_AB[2], //NOLINT
                      T_AB[4], T_AB[5], T_AB[6],
                      T_AB[8], T_AB[9], T_AB[10]};

    std::array<IkReal, 9> R_BC = {T_BC[0], T_BC[1], T_BC[2], //NOLINT
                      T_BC[4], T_BC[5], T_BC[6],
                      T_BC[8], T_BC[9], T_BC[10]};

    // Extract translation vectors
    std::array<IkReal, 3> t_AB = {T_AB[3], T_AB[7], T_AB[11]}; //NOLINT
    std::array<IkReal, 3> t_BC = {T_BC[3], T_BC[7], T_BC[11]}; //NOLINT

    // Compute R_AC = R_AB * R_BC (3x3 matrix multiplication)
    std::array<IkReal,9> R_AC{}; //NOLINT
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_AC[i * 3 + j] = 0.0; //NOLINT
            for (int k = 0; k < 3; ++k) {
                R_AC[i * 3 + j] += R_AB[i * 3 + k] * R_BC[k * 3 + j]; //NOLINT
            }
        }
    }

    // Compute t_AC = R_AB * t_BC + t_AB
    std::array<IkReal, 3> t_AC{}; //NOLINT
    for (int i = 0; i < 3; ++i) {
        t_AC[i] = t_AB[i]; //NOLINT
        for (int j = 0; j < 3; ++j) {
            t_AC[i] += R_AB[i * 3 + j] * t_BC[j]; //NOLINT
        }
    }

    // Pack result into 12-element format
    T_AC[0]  = R_AC[0];  T_AC[1]  = R_AC[1];  T_AC[2]  = R_AC[2];  T_AC[3]  = t_AC[0];
    T_AC[4]  = R_AC[3];  T_AC[5]  = R_AC[4];  T_AC[6]  = R_AC[5];  T_AC[7]  = t_AC[1];
    T_AC[8]  = R_AC[6];  T_AC[9]  = R_AC[7];  T_AC[10] = R_AC[8];  T_AC[11] = t_AC[2];
}

/**
 * Extract translation and rotation from 4x4 matrix in 12-element format.
 * @param[in] matrix_12 Input 12-element matrix (4x4 format).
 * @param[out] eetrans Output translation array (3 elements).
 * @param[out] eerot Output rotation array (9 elements).
 * @return void
 */
static void extract_transform_from_matrix(const IkReal* matrix_12, std::array<IkReal, 3>& eetrans, std::array<IkReal, 9>& eerot) {
    // Extract translation (Tx, Ty, Tz)
    eetrans[0] = matrix_12[3];  // Tx
    eetrans[1] = matrix_12[7];  // Ty
    eetrans[2] = matrix_12[11]; // Tz

    // Extract rotation matrix (row-major 3x3)
    eerot[0] = matrix_12[0];  // R11
    eerot[1] = matrix_12[1];  // R12
    eerot[2] = matrix_12[2];  // R13
    eerot[3] = matrix_12[4];  // R21
    eerot[4] = matrix_12[5];  // R22
    eerot[5] = matrix_12[6];  // R23
    eerot[6] = matrix_12[8];  // R31
    eerot[7] = matrix_12[9];  // R32
    eerot[8] = matrix_12[10]; // R33
}

/**
 * Wrapper to normalize angle to [-pi, pi]
 * @param[in] angle Input angle in radians.
 * @return Normalized angle in radians within [-pi, pi].
 */
static IkReal normalize_angle(IkReal angle) {
    while (angle > M_PI) { angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) { angle += 2.0 * M_PI;
    }
    return angle;
}

/**
 * Compute Euclidean distance between two joints arrays.
 * @param[in] joints1 First joint array.
 * @param[in] joints2 Second joint array.
 * @param[in] dof Number of joints (degrees of freedom).
 * @param[in] limits Joint limits for each joint.
 * @return Euclidean distance between the two joint configurations.
 */
static IkReal compute_joint_distance(
    const IkReal* joints1,
    const IkReal* joints2,
    int dof,
    const std::vector<JointLimit>& limits
) {
    IkReal sum = 0.0;
    for (int i = 0; i < dof; ++i) {
        IkReal diff = joints1[i] - joints2[i];

        // Apply angle wrapping for revolute joints
        // Heuristic: If joint limit range > 6.0 rad (≈2π), treat as revolute with wrapping
        bool is_revolute = true;
        if (!limits.empty() && i < static_cast<int>(limits.size())) {
            IkReal range = limits[i].upper - limits[i].lower;
            // If range >= 6.0, likely revolute joint that can wrap around
            is_revolute = (range >= 6.0);
        }

        if (is_revolute) {
            diff = normalize_angle(diff);
        }

        sum += diff * diff;
    }
    return std::sqrt(sum);
}

/**
 * Safely compute midpoint of joint limits, handling infinite limits.
 * @param[in] joint_min Array of joint minimum limits.
 * @param[in] joint_max Array of joint maximum limits.
 * @param[in] idx Index of the joint.
 * @return Midpoint value or 0.0 if limits are infinite.
 */
static IkReal safe_midpoint(const IkReal* joint_min, const IkReal* joint_max, int idx) { //NOLINT
    IkReal mn = joint_min[idx];
    IkReal mx = joint_max[idx];
    if (std::isfinite(mn) && std::isfinite(mx)) {
        return 0.5 * (mn + mx);
    }
    return 0.0;
}

/*
==============================================================================
    Helpers (Path & Plugin Loading)
==============================================================================
*/

/**
 * Detect robot manufacturer based on robot name prefix.
 * @param[in] robot_name Name of the robot.
 * @return Manufacturer string ("kawasaki", "yaskawa", or "").
 */
std::string detect_manufacturer(const std::string& robot_name) {
    std::string robot_upper = robot_name;
    std::transform(robot_upper.begin(), robot_upper.end(), robot_upper.begin(), ::toupper);

    // Kawasaki: KJ, RS prefix
    if (robot_upper.find("KJ") == 0 || robot_upper.find("RS") == 0) {
        return "kawasaki";
    }

    // Yaskawa: GP, MPX prefix
    if (robot_upper.find("GP") == 0 || robot_upper.find("MPX") == 0) {
        return "yaskawa";
    }

    // Default: no specific manufacturer
    return "";
}

/**
 * Check if tcp_pose contains any NaN or Inf values
 *
 * @param[in] tcp_pose Pointer to the tcp_pose array.
 * @param[in] size Number of elements in tcp_pose (default 12).
 * @return True if all values are finite, false if any NaN or Inf detected.
 */
static bool is_tcp_pose_valid(const IkReal* tcp_pose, int size = 12) {
    if (tcp_pose == nullptr) { return false;
    }
    for (int i = 0; i < size; ++i) {
        if (!std::isfinite(tcp_pose[i])) {
            return false;  // NaN or Inf detected
        }
    }
    return true;
}


/**
 * Try to load a single IKFast plugin from a DLL file.
 *
 * @param[in] dll_path Path to the DLL file.
 * @param[out] out_plugin Output RobotIkPlugin structure to populate.
 * @return True if plugin was successfully loaded, false otherwise.
 */
static bool try_load_plugin_from_dll(const fs::path& dll_path, RobotIkPlugin& out_plugin) {
    std::string robot = robot_name_from_path(dll_path);

    // Use LOAD_WITH_ALTERED_SEARCH_PATH to search in the DLL's directory
    HMODULE dll = LoadLibraryExW(dll_path.wstring().c_str(), nullptr,
                                  LOAD_WITH_ALTERED_SEARCH_PATH);
    if (dll == nullptr) {
        DWORD err = GetLastError();
        std::wcerr << L"[ERROR] Failed to LoadLibrary: " << dll_path.wstring()
                   << L" (error code: " << err << L")" << '\n';
        return false;
    }

    RobotIkPlugin plugin;
    plugin.dll = dll;
    plugin.name = robot;

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

    if ((plugin.fns._ComputeIk == nullptr) || (plugin.fns._GetNumJoints == nullptr)) {
        std::cerr << "DLL missing mandatory IKFast symbols, skipping: "
                  << dll_path.string() << '\n';
        FreeLibrary(dll);
        return false;
    }

    plugin.num_joints = plugin.fns._GetNumJoints();
    if (plugin.fns._GetNumFreeParameters != nullptr) {
        plugin.num_free = plugin.fns._GetNumFreeParameters();
    }

    // Validate IkReal size compatibility (must match host sizeof(IkReal))
    if (plugin.fns._GetIkRealSize != nullptr) {
        plugin.ikreal_size = plugin.fns._GetIkRealSize();
    }
    if (plugin.ikreal_size != static_cast<int>(sizeof(IkReal))) {
        std::cerr << "IkReal size mismatch for plugin '" << robot << "': plugin="
                  << plugin.ikreal_size << ", host=" << sizeof(IkReal)
                  << ". Plugin will be skipped. Rebuild plugin with matching precision." << '\n';
        FreeLibrary(dll);
        return false;
    }

    // Load joint limits from compiled data
    bool has_limits = load_joint_limits_from_data(robot, plugin.joint_limits);

    std::cout << "Loaded IK plugin: " << robot
              << " (joints=" << plugin.num_joints
              << ", free=" << plugin.num_free
              << ", IkRealSize=" << plugin.ikreal_size;

    if (has_limits) {
        std::cout << ", limits=" << plugin.joint_limits.size() << ")";
    } else {
        std::cout << ", limits=none)";
    }
    std::cout << "\n";

    out_plugin = std::move(plugin);
    return true;
}

/**
 * Scan robots directory and load all IKFast plugin DLLs.
 *
 * @param[in] robots_dir Path to the robots directory.
 * @return True if at least one plugin was loaded successfully.
 */
static bool scan_and_load_all_plugins(const fs::path& robots_dir) {
    std::cerr << "[INIT] Starting recursive directory search..." << '\n';

    try {
        int dll_count = 0;
        for (const auto& entry : fs::recursive_directory_iterator(robots_dir)) {
            if (!entry.is_regular_file()) {
                continue;
            }
            const fs::path& dll_path = entry.path();
            if (dll_path.extension() != ".dll") {
                continue;
            }
            dll_count++;
            std::wcerr << L"[INIT] Found DLL #" << dll_count << L": " << dll_path.wstring() << '\n';

            if (is_dependency_dll(dll_path)) {
                // Avoid treating dependency DLLs as IK plugins
                continue;
            }

            RobotIkPlugin plugin;
            if (try_load_plugin_from_dll(dll_path, plugin)) {
                // Store with lowercase name for case-insensitive lookup
                std::string robot_lower = normalize_robot_name(plugin.name);
                g_plugins.emplace(robot_lower, std::move(plugin));
            }
        }
        std::cerr << "[INIT] Finished directory search. Found " << g_plugins.size() << " plugins." << '\n';
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception during directory iteration: " << e.what() << '\n';
        return false;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception during directory iteration" << '\n';
        return false;
    }

    return !g_plugins.empty();
}

/**
 * Add DLL search paths for loading IKFast plugins and dependencies.
 * @param[in] robots_dir Path to the robots directory containing IKFast plugins.
 * @return void
 */
static void configure_dll_search_paths(const fs::path& robots_dir) {
    using SetDefaultDllDirectoriesFn = BOOL (WINAPI*)(DWORD);
    using AddDllDirectoryFn = DLL_DIRECTORY_COOKIE (WINAPI*)(PCWSTR);

    // Build candidate DLL search paths
    std::vector<fs::path> dll_dirs;
    dll_dirs.push_back(robots_dir);

    fs::path deps = robots_dir.parent_path() / "dependencies";
    if (fs::exists(deps) && fs::is_directory(deps)) {
        dll_dirs.push_back(deps);
    }

    char* vcpkg_buffer = nullptr;
    size_t vcpkg_size = 0;
    std::string vcpkg_root = "C:\\dev\\vcpkg";
    if (_dupenv_s(&vcpkg_buffer, &vcpkg_size, "VCPKG_ROOT") == 0 && vcpkg_buffer != nullptr) {
        std::unique_ptr<char, decltype(&free)> vcpkg_guard(vcpkg_buffer, free);
        vcpkg_root = vcpkg_buffer;
    }
    fs::path vcpkg_bin = fs::path(vcpkg_root) / "installed" / "x64-windows" / "bin";
    if (fs::exists(vcpkg_bin) && fs::is_directory(vcpkg_bin)) {
        dll_dirs.push_back(vcpkg_bin);
    }

    HMODULE kernel32 = GetModuleHandleW(L"kernel32.dll");
    auto set_default_dll_dirs = reinterpret_cast<SetDefaultDllDirectoriesFn>(
        GetProcAddress(kernel32, "SetDefaultDllDirectories"));
    auto add_dll_directory = reinterpret_cast<AddDllDirectoryFn>(
        GetProcAddress(kernel32, "AddDllDirectory"));

    // Prefer modern API if available (Win8+)
    if ((set_default_dll_dirs != nullptr) && (add_dll_directory != nullptr)) {
        set_default_dll_dirs(LOAD_LIBRARY_SEARCH_DEFAULT_DIRS | LOAD_LIBRARY_SEARCH_USER_DIRS);
        for (const auto& dir : dll_dirs) {
            add_dll_directory(dir.wstring().c_str());
        }
    } else {
        // Fallback for older systems: set robots dir as DLL dir
        SetDllDirectoryW(robots_dir.wstring().c_str());

        // Best-effort: prepend additional dirs to PATH so dependency resolution still works
        char* path_buffer = nullptr;
        size_t path_size = 0;
        std::string current_path;
        if (_dupenv_s(&path_buffer, &path_size, "PATH") == 0 && path_buffer != nullptr) {
            std::unique_ptr<char, decltype(&free)> path_guard(path_buffer, free);
            current_path = path_buffer;
        }
        for (const auto& dir : dll_dirs) {
            std::string dir_str = dir.string();
            if (current_path.find(dir_str) == std::string::npos) {
                dir_str += ";";
                dir_str += current_path;
                current_path = dir_str;
            }
        }
        SetEnvironmentVariableA("PATH", current_path.c_str());
    }
}

/**
 * Load IKFast plugins from the specified robots directory using recursive directory iteration.
 * @param[in] robots_dir Path to the robots directory.
 * @return True if at least one plugin was loaded successfully, false otherwise.
 */
bool load_ik_plugins(const std::string& robots_dir) {
    g_plugins.clear();

    // Convert UTF-8 path to wide string for proper Korean/Unicode support
    std::wstring wide_path = utf8_to_wstring(robots_dir);

    fs::path dir(wide_path);

    if (!fs::exists(dir)) {
        std::wcerr << L"[ERROR] Wide path does not exist: " << wide_path << '\n';
        return false;
    }

    if (!fs::is_directory(dir)) {
        std::cerr << "[ERROR] robots path is not a directory: " << robots_dir << '\n';
        return false;
    }

    std::cerr << "[INIT] robots directory found and validated" << '\n';

    // Configure DLL search paths (robots dir + dependencies)
    configure_dll_search_paths(dir);

    // CRITICAL: Load liblapack.dll and openblas.dll from robots directory FIRST
    // This prevents conda's LAPACK from being loaded when robot DLLs call dgeev_
    fs::path local_openblas = dir / "openblas.dll";
    fs::path local_lapack = dir / "liblapack.dll";

    if (fs::exists(local_openblas)) {
        HMODULE h = LoadLibraryW(local_openblas.wstring().c_str());
        if (h != nullptr) {
            std::array<char, MAX_PATH> path{};
            GetModuleFileNameA(h, path.data(), MAX_PATH);
            std::cerr << "Preloaded local OpenBLAS: " << path.data() << '\n';
        } else {
            DWORD err = GetLastError();
        }
    }

    if (fs::exists(local_lapack)) {
        // Load with full path to ensure this specific DLL is loaded
        HMODULE h = LoadLibraryW(local_lapack.wstring().c_str());
        if (h != nullptr) {
            std::array<char, MAX_PATH> path{};
            GetModuleFileNameA(h, path.data(), MAX_PATH);
            std::cerr << "Preloaded local LAPACK: " << path.data() << '\n';
        } else {
            DWORD err = GetLastError();
        }
    }

    // Also try vcpkg as fallback
    char* vcpkg_buffer = nullptr;
    size_t vcpkg_size = 0;
    std::string vcpkg_root = "C:\\dev\\vcpkg";
    if (_dupenv_s(&vcpkg_buffer, &vcpkg_size, "VCPKG_ROOT") == 0 && vcpkg_buffer != nullptr) {
        std::unique_ptr<char, decltype(&free)> vcpkg_guard(vcpkg_buffer, free);
        vcpkg_root = vcpkg_buffer;
    }
    fs::path vcpkg_bin = fs::path(vcpkg_root) / "installed" / "x64-windows" / "bin";

    fs::path vcpkg_openblas = vcpkg_bin / "openblas.dll";
    if (fs::exists(vcpkg_openblas) && !fs::exists(local_openblas)) {
        HMODULE h = LoadLibraryW(vcpkg_openblas.wstring().c_str());
        if (h != nullptr) {
            std::array<char, MAX_PATH> path{};
            GetModuleFileNameA(h, path.data(), MAX_PATH);
            std::cerr << "Preloaded vcpkg OpenBLAS: " << path.data() << '\n';
        }
    }

    // Scan and load all IKFast plugin DLLs
    return scan_and_load_all_plugins(dir);
}


/**
 * Find loaded RobotIkPlugin by robot name (case-insensitive).
 * @param[in] name Robot name to search for.
 * @return Pointer to RobotIkPlugin if found, nullptr otherwise. 
 */
static RobotIkPlugin* find_robot(const std::string& name) {
    std::string normalized = normalize_robot_name(name);
    auto it = g_plugins.find(normalized);
    if (it == g_plugins.end()) { return nullptr;
    }
    return &it->second;
}

/*
==============================================================================
    Core API
==============================================================================
*/

/**
 * Solve multiple IK for the specified robot and TCP pose.
 * @param[in] robot_name Name of the robot.
 * @param[in] tcp_pose Pointer to 12-element TCP pose array (4x4 matrix format).
 * @param[out] out_solutions Output vector to store found IK solutions.
 * @return True if at least one valid solution was found, false otherwise.
 */
bool solveIK(
    const std::string& robot_name,
    const IkReal* tcp_pose,
    std::vector<IkSolutionData>& out_solutions
) {
    RobotIkPlugin* plugin = find_robot(robot_name);
    if (plugin == nullptr) {
        std::cerr << "Robot IK plugin not loaded: " << robot_name << '\n';
        return false;
    }
    if (plugin->ikreal_size != static_cast<int>(sizeof(IkReal))) {
        std::cerr << "IkReal size mismatch for robot '" << robot_name << "': plugin="
                  << plugin->ikreal_size << ", host=" << sizeof(IkReal)
                  << ". Aborting solveIK." << '\n';
        return false;
    }
    if (plugin->fns._ComputeIk == nullptr) {
        std::cerr << "ComputeIk not available\n";
        return false;
    }

    // PATCH: Input validation - check tcp_pose for NaN/Inf
    if (!is_tcp_pose_valid(tcp_pose, 12)) {
        std::cerr << "[IK Safety] tcp_pose contains NaN or Inf values. Aborting solveIK.\n";
        return false;
    }

    // Apply manufacturer-specific coordinate transformation if available
    Transform transformed_pose{};
    const IkReal* pose_to_use = tcp_pose;

    std::string manufacturer = detect_manufacturer(robot_name);
    std::cout<<"manufacturer: "<<manufacturer<<'\n';
    if (!manufacturer.empty()) {
        auto it = coordinate_transform_data::MANUFACTURER_TRANSFORMS.find(manufacturer);
        if (it != coordinate_transform_data::MANUFACTURER_TRANSFORMS.end()) {
            // Apply transformation: transformed_pose = tcp_pose * coord_transform
            multiply_transforms(Transform(tcp_pose), Transform(it->second.data()), transformed_pose);
            pose_to_use = transformed_pose.data.data();

            // Debug log (can be disabled in production)
            static bool debug_transform = (std::getenv("IK_DEBUG_TRANSFORM") != nullptr);
            if (debug_transform) {
                std::cout << "[Coordinate Transform] Applied '" << manufacturer
                          << "' transform for robot '" << robot_name << "'\n";
            }
        }
    }

    // Extract position and rotation from 4x4 matrix (12 elements)
    std::array<IkReal, 3> eetrans{};
    std::array<IkReal, 9> eerot{};
    extract_transform_from_matrix(pose_to_use, eetrans, eerot);

    ikfast::IkSolutionList<IkReal> solutions;
    bool ok = plugin->fns._ComputeIk(eetrans.data(), eerot.data(), nullptr, solutions);

    if (!ok) { return false;
    }

    size_t nsol = solutions.GetNumSolutions();
    out_solutions.clear();
    out_solutions.reserve(nsol);

    std::vector<IkReal> freevec;  // No free parameters

    // Filter solutions by joint limits
    int filtered_count = 0;
    for (size_t i = 0; i < nsol; ++i) {
        const ikfast::IkSolutionBase<IkReal>& sol = solutions.GetSolution(i);
        std::vector<IkReal> joints;
        sol.GetSolution(joints, freevec);

        if (static_cast<int>(joints.size()) != plugin->num_joints) {
            std::cerr << "[Joint Limit] Solution DOF mismatch for robot '" << robot_name
                      << "': solution_dof=" << joints.size()
                      << ", expected=" << plugin->num_joints
                      << ". Aborting solveIK.\n";
            return false;
        }

        if (!check_joint_limits(joints.data(), plugin->num_joints, plugin->joint_limits)) {
            // Debug which joint(s) violated limits (optional, controlled by environment variable)
            static bool debug_limits = (std::getenv("IK_DEBUG_LIMITS") != nullptr);
            if (debug_limits) {
                int dof = plugin->num_joints;
                const auto& limits = plugin->joint_limits;
                // Note: limits count already validated in check_joint_limits()
                for (int ji = 0; ji < dof && ji < static_cast<int>(limits.size()); ++ji) {
                    const JointLimit& lim = limits[ji];
                    if (is_fixed_joint(lim)) { continue;  // skip fixed joints
                    }

                    IkReal angle = joints[ji];
                    if (angle < lim.lower || angle > lim.upper) {
                        std::cerr << "[Joint Limit DEBUG] Robot '" << robot_name
                                    << "' J" << (ji + 1) << " (index=" << ji << ")"
                                    << " value=" << angle
                                    << " outside [" << lim.lower << ", " << lim.upper << "]\n";
                    }
                }
            }
            ++filtered_count;
            continue;
        }

        IkSolutionData s;
        s.joints = std::move(joints);
        out_solutions.push_back(std::move(s));
    }

    if (filtered_count > 0 && !plugin->joint_limits.empty()) {
        std::cerr << "[Joint Limit Filter] " << robot_name
                  << ": filtered " << filtered_count << "/" << nsol << " solutions\n";
    }

    return !out_solutions.empty();
}

/**
 * Get number of joints for the specified robot.
 * @param[in] robot_name Name of the robot.
 * @return Number of joints, or -1 if robot not found.
 */
int get_num_joints(const std::string& robot_name) {
    RobotIkPlugin* p = find_robot(robot_name);
    return (p != nullptr) ? p->num_joints : -1;
}

/**
 * Get number of DOF for the specified robot.
 * @param[in] robot_name Name of the robot.
 * @return Number of DOF, or -1 if robot not found.
 */
int get_num_free_parameters(const std::string& robot_name) {
    RobotIkPlugin* p = find_robot(robot_name);
    return (p != nullptr) ? p->num_free : -1;
}

/**
 * Compute Forward Kinematics for the specified robot and joint angles.
 * @param[in] robot_name Name of the robot.
 * @param[in] joints Pointer to array of joint angles.
 * @param[out] eetrans Output array for end-effector translation (3 elements).
 * @param[out] eerot Output array for end-effector rotation (9 elements).
 * @return True if FK computation was successful, false otherwise.
 */
bool computeFK(
    const std::string& robot_name,
    const IkReal* joints,
    IkReal* eetrans, //NOLINT
    IkReal* eerot //NOLINT
) {
    RobotIkPlugin* plugin = find_robot(robot_name);
    if (plugin == nullptr) {
        std::cerr << "Robot IK plugin not loaded: " << robot_name << '\n';
        return false;
    }
    if (plugin->fns._ComputeFk == nullptr) {
        std::cerr << "ComputeFk not available for robot: " << robot_name << '\n';
        return false;
    }

    // PATCH: FK 입력 조인트가 joint limit을 넘으면 false 반환
    if (!plugin->joint_limits.empty()) {
        if (!check_joint_limits(joints, plugin->num_joints, plugin->joint_limits)) {
            std::cerr << "[Joint Limit] FK input violates limits for robot '"
                      << robot_name << "'\n";
            return false;
        }
    }

    // Compute FK in tool coordinate frame
    std::array<IkReal, 3> fk_trans_tool{};
    std::array<IkReal, 9> fk_rot_tool{};
    plugin->fns._ComputeFk(joints, fk_trans_tool.data(), fk_rot_tool.data());

    // Apply inverse coordinate transformation to convert back to base frame
    std::string manufacturer = detect_manufacturer(robot_name);
    if (!manufacturer.empty()) {
        auto it = coordinate_transform_data::MANUFACTURER_TRANSFORMS.find(manufacturer);
        if (it != coordinate_transform_data::MANUFACTURER_TRANSFORMS.end()) {
            // Build FK result as 12-element matrix (tool frame)
            std::array<IkReal, 12> fk_pose_tool = {
                fk_rot_tool[0], fk_rot_tool[1], fk_rot_tool[2], fk_trans_tool[0],
                fk_rot_tool[3], fk_rot_tool[4], fk_rot_tool[5], fk_trans_tool[1],
                fk_rot_tool[6], fk_rot_tool[7], fk_rot_tool[8], fk_trans_tool[2]
            };

            // Compute inverse transform: T_base = T_tool * T_inv
            // For rotation-only transform: T_inv = T^T (transpose)
            const auto& T_forward = it->second; //NOLINT
            std::array<IkReal, 12> T_inverse = { //NOLINT
                T_forward[0], T_forward[4], T_forward[8],  0.0,  // R^T row 1
                T_forward[1], T_forward[5], T_forward[9],  0.0,  // R^T row 2
                T_forward[2], T_forward[6], T_forward[10], 0.0   // R^T row 3
            };

            Transform fk_pose_base{};
            multiply_transforms(Transform(fk_pose_tool.data()), Transform(T_inverse.data()), fk_pose_base);

            // Extract back to eetrans and eerot (base frame)
            eetrans[0] = fk_pose_base[3];
            eetrans[1] = fk_pose_base[7];
            eetrans[2] = fk_pose_base[11];

            eerot[0] = fk_pose_base[0];
            eerot[1] = fk_pose_base[1];
            eerot[2] = fk_pose_base[2];
            eerot[3] = fk_pose_base[4];
            eerot[4] = fk_pose_base[5];
            eerot[5] = fk_pose_base[6];
            eerot[6] = fk_pose_base[8];
            eerot[7] = fk_pose_base[9];
            eerot[8] = fk_pose_base[10];

            // Debug log
            char* debug_buffer = nullptr;
            size_t debug_size = 0;
            static bool debug_transform = (_dupenv_s(&debug_buffer, &debug_size, "IK_DEBUG_TRANSFORM") == 0);
            if (debug_transform) {  
                std::cout << "[Coordinate Transform] Applied inverse '" << manufacturer
                          << "' transform for FK of robot '" << robot_name << "'\n";
            }
        } else {
            // No transform - direct copy
            eetrans[0] = fk_trans_tool[0];
            eetrans[1] = fk_trans_tool[1];
            eetrans[2] = fk_trans_tool[2];
            std::copy(fk_rot_tool.begin(), fk_rot_tool.end(), eerot);
        }
    } else {
        // No manufacturer - direct copy
        eetrans[0] = fk_trans_tool[0];
        eetrans[1] = fk_trans_tool[1];
        eetrans[2] = fk_trans_tool[2];
        std::copy(fk_rot_tool.begin(), fk_rot_tool.end(), eerot);
    }

    return true;
}


/**
 * Get robot-specific J3 reference angle for Elbow UP/DOWN configuration.
 * @param[in] robot_lower Lowercase robot name.
 * @param[in] joint_min Array of joint minimum limits.
 * @param[in] joint_max Array of joint maximum limits.
 * @param[in] idx_elbow Index of the elbow joint (Default : J3).
 * @return Reference angle for J3.
 */
static IkReal get_j3_reference(const std::string& robot_lower,
                             const IkReal* joint_min,
                             const IkReal* joint_max,
                             int idx_elbow) {
    // Robot-specific reference (hard-coded for now)
    if (robot_lower == "kj125") {
        return M_PI / 2.0;  // 90 degrees
    }

    // Fallback: midpoint
    return safe_midpoint(joint_min, joint_max, idx_elbow);
}


/** 
 * Check if the given joint configuration matches the desired front/back, elbow, and wrist configurations.
 * @param[in] joints Array of joint angles [0..5].
 * @param[in] joint_min Array of joint minimum limits [0..5].
 * @param[in] joint_max Array of joint maximum limits [0..5].
 * @param[in] frontback_cfg Desired front/back configuration (0=FRONT, 1=REAR).
 * @param[in] elbow_cfg Desired elbow configuration (0=UP, 1=DOWN).
 * @param[in] wrist_cfg Desired wrist configuration (0=N_FLIP, 1=FLIP).
 * @param[in] tcp_pose (Optional) TCP pose (12 elements) for front/back classification.
 * @param[in] robot_lower Lowercase robot name.
 * @param[in] eps Epsilon tolerance for comparisons.
 * @return True if the joint configuration matches the desired configuration, false otherwise.
 */
static bool matches_configuration(
    const IkReal* joints,      // [0..5] //NOLINT
    const IkReal* joint_min,   // [0..5] //NOLINT
    const IkReal* joint_max,   // [0..5]
    int frontback_cfg,         // 0 = FRONT, 1 = REAR
    int elbow_cfg,             // 0 = UP,    1 = DOWN
    int wrist_cfg,             // 0 = N_FLIP,1 = FLIP
    const IkReal* tcp_pose,    // 12 elements [R|T]
    const std::string& robot_lower,
    IkReal eps
) {
    // Default epsilon if zero/negative provided
    if (eps <= 0) { eps = 1e-5;
    }

    constexpr int IDX_FRONTBACK = 0;  // joint1 (J1) - index 0 //NOLINT
    constexpr int IDX_ELBOW     = 2;  // joint3 (J3) - index 2 //NOLINT
    constexpr int IDX_WRIST     = 4;  // joint5 (J5) - index 4 //NOLINT

    IkReal j1 = normalize_angle(joints[IDX_FRONTBACK]);
    IkReal j3 = normalize_angle(joints[IDX_ELBOW]);
    IkReal j5 = joints[IDX_WRIST];

    // Translation from tcp_pose
    IkReal tx = (tcp_pose != nullptr) ? tcp_pose[3]  : 0.0;
    IkReal ty = (tcp_pose != nullptr) ? tcp_pose[7]  : 0.0;
    // IkReal tz = tcp_pose ? tcp_pose[11] : 0.0; // not needed for yaw

    // 1) FRONT/BACK using j1 against target direction
    // j1 = 0 aligns TCP toward -Y (reference) for KJ125
    // FRONT: j1와 TCP가 같은 방향 (각도 차이 < 90도)
    // REAR: j1와 TCP가 반대 방향 (각도 차이 >= 90도)
    IkReal yaw_target = std::atan2(tx, -ty);     // Note: (x,-y) -> yaw; j1=0 => -Y (KJ125)
    IkReal diff = normalize_angle(yaw_target - j1);

    // DEBUG LOG
    static bool debug_enabled = (std::getenv("IK_DEBUG_CONFIG") != nullptr);
    if (debug_enabled) {
        std::cout << "[C++ matchConfig DEBUG] Requested config: (" << frontback_cfg
                  << "," << elbow_cfg << "," << wrist_cfg << ")\n";
        std::cout << "  J1=" << j1 * 180.0 / M_PI << " deg, J3=" << j3 * 180.0 / M_PI
                  << " deg, J5=" << j5 * 180.0 / M_PI << " deg\n";
        std::cout << "  yaw_target=" << yaw_target * 180.0 / M_PI << " deg, diff="
                  << diff * 180.0 / M_PI << " deg, |diff|=" << std::abs(diff) * 180.0 / M_PI << " deg\n";
    }

    // FRONT (0): |diff| < 90도 (같은 쪽 반원)
    // REAR (1): |diff| >= 90도 (반대쪽 반원)
    bool frontback_match =
        (frontback_cfg == 0)  // 0 = FRONT
            ? (std::abs(diff) < (M_PI / 2.0))
            : (std::abs(diff) >= (M_PI / 2.0));

    if (debug_enabled) {
        std::cout << "  FRONT/BACK: |diff| <= pi/2 = " << (std::abs(diff) <= (M_PI / 2.0 + eps)) //NOLINT
                  << ", requested=" << frontback_cfg << ", match=" << frontback_match << "\n";
    }

    // 2) UP/DOWN using J3 reference (robot-specific)
    // J3 < ref => UP (2), J3 >= ref => DOWN (3)
    IkReal j3_ref = get_j3_reference(robot_lower, joint_min, joint_max, IDX_ELBOW);
    IkReal j3_delta = j3 - j3_ref;

    bool elbow_match =
        (elbow_cfg == 2)  // 2 = UP
            ? (j3_delta < -eps)           // UP: J3 < ref
            : (j3_delta >= -eps);         // DOWN: J3 >= ref

    if (debug_enabled) {
        std::cout << "  ELBOW: J3_ref=" << j3_ref * 180.0 / M_PI << " deg, delta="
                  << j3_delta * 180.0 / M_PI << " deg, delta < 0 = " << (j3_delta < -eps)
                  << ", requested=" << elbow_cfg << ", match=" << elbow_match << "\n";
    }

    // 3) FLIP / N_FLIP (joint5, sign-based: J5 >= 0 => N_FLIP)
    IkReal j5n = normalize_angle(j5);
    bool is_nonflip = (j5n >= -eps);  // Use small tolerance for boundary case

    bool wrist_match =
        (wrist_cfg == 4)  // 4 = N_FLIP
            ?  is_nonflip
            : !is_nonflip;               // FLIP

    if (debug_enabled) {
        std::cout << "  WRIST: J5_norm=" << j5n * 180.0 / M_PI << " deg, J5 >= 0 = "
                  << is_nonflip << ", requested=" << wrist_cfg
                  << ", match=" << wrist_match << "\n";
        std::cout << "  => TOTAL MATCH: " << (frontback_match && elbow_match && wrist_match) << "\n";
    }

    return frontback_match && elbow_match && wrist_match;
}

/**
 * Solve IK with specified configuration options.
 * @param[in] robot_name Name of the robot.
 * @param[in] tcp_pose Pointer to 12-element TCP pose array (4x4 matrix format).
 * @param[in] shoulder_config Desired shoulder configuration (0=FRONT, 1=BACK).
 * @param[in] elbow_config Desired elbow configuration (2=UP, 3=DOWN).
 * @param[in] wrist_config Desired wrist configuration (4=N_FLIP, 5=FLIP).
 * @param[out] out_joints Output array for joint angles.
 * @param[out] out_is_solvable Output flag indicating if a solution was found.
 * @param[in] current_joints (Optional) Current joint angles for continuity (nullptr = old behavior).
 * @return True if a solution was found, false otherwise.
 */

bool solveIKWithConfig(
    const std::string& robot_name,
    const IkReal* tcp_pose,
    int shoulder_config,       // 0 = FRONT, 1 = BACK
    int elbow_config,          // 2 = UP, 3 = DOWN
    int wrist_config,          // 4 = N_FLIP, 5 = FLIP
    IkReal* out_joints,
    bool* out_is_solvable,
    const IkReal* current_joints  // NEW: optional for continuity (nullptr = old behavior)
) {
    *out_is_solvable = false;

    // Convert case-insensitive robot name
    std::string robot_lower = robot_name;
    std::transform(robot_lower.begin(), robot_lower.end(), robot_lower.begin(), ::tolower);

    // Get robot plugin to access joint limits
    RobotIkPlugin* plugin = find_robot(robot_lower);
    if (plugin == nullptr) {
        std::cerr << "Robot IK plugin not loaded: " << robot_lower << '\n';
        return false;
    }
    int dof = plugin->num_joints;
    if (dof <= 0) {
        return false;  // Invalid DOF
    }

    // Prepare joint limit arrays
    std::vector<IkReal> joint_min(dof, -std::numeric_limits<IkReal>::infinity());
    std::vector<IkReal> joint_max(dof,  std::numeric_limits<IkReal>::infinity());

    // Load joint limits if available
    if (!plugin->joint_limits.empty() && plugin->joint_limits.size() >= static_cast<size_t>(dof)) {
        for (int i = 0; i < dof; ++i) {
            joint_min[i] = plugin->joint_limits[i].lower;
            joint_max[i] = plugin->joint_limits[i].upper;
        }
    }

    // Solve IK to get all solutions
    std::vector<IkSolutionData> all_solutions;
    bool ok = solveIK(robot_lower, tcp_pose, all_solutions);
    if (!ok || all_solutions.empty()) {
        return false;  // No solutions found
    }

    // Filter by configuration with multiple epsilon passes
    std::vector<const IkSolutionData*> config_matches;

    for (IkReal eps : {1e-6, 1e-5, 1e-3}) {
        config_matches.clear();

        for (const auto& sol : all_solutions) {
            if (matches_configuration(
                sol.joints.data(),
                joint_min.data(),
                joint_max.data(),
                shoulder_config,
                elbow_config,
                wrist_config,
                tcp_pose,
                robot_lower,
                eps))
            {
                config_matches.push_back(&sol);
            }
        }

        if (!config_matches.empty()) { break;  // Found matches at this threshold
        }
    }

    if (config_matches.empty()) {
        return false;  // No configuration match
    }

    // If current_joints provided, select nearest match
    if (current_joints != nullptr) {
        // Validate current_joints are within limits
        if (!plugin->joint_limits.empty()) {
            if (!check_joint_limits(current_joints, dof, plugin->joint_limits)) {
                std::cerr << "[solveIKWithConfig] current_joints violate limits\n";
                return false;
            }
        }

        IkReal min_distance = std::numeric_limits<IkReal>::infinity();
        const IkSolutionData* best_sol = nullptr;

        for (const auto* sol : config_matches) {
            IkReal distance = compute_joint_distance(
                sol->joints.data(),
                current_joints,
                dof,
                plugin->joint_limits
            );

            if (distance < min_distance) {
                min_distance = distance;
                best_sol = sol;
            }
        }

        if (best_sol != nullptr) {
            std::copy(best_sol->joints.begin(), best_sol->joints.end(), out_joints);
            *out_is_solvable = true;
            return true;
        }
    } else {
        // FALLBACK : not nearest, just take the first match
        const auto* first_match = config_matches[0];
        std::copy(first_match->joints.begin(), first_match->joints.end(), out_joints);
        *out_is_solvable = true;
        return true;
    }

    return false;
}


/**
 * Solve IK with nearest joint.
 * @param[in] robot_name Name of the robot.
 * @param[in] tcp_pose Pointer to 12-element TCP pose array (4x4 matrix format).
 * @param[in] current_joints Pointer to current joint angles array.
 * @param[out] out_joints Output array for joint angles.
 * @param[out] out_is_solvable Output flag indicating if a solution was found.
 * @return True if a solution was found, false otherwise.
 */
bool solveIKWithJoint(
    const std::string& robot_name,
    const IkReal* tcp_pose, //NOLINT
    const IkReal* current_joints,
    IkReal* out_joints,
    bool* out_is_solvable
) {
    *out_is_solvable = false;

    // Convert case-insensitive robot name
    std::string robot_lower = robot_name;
    std::transform(robot_lower.begin(), robot_lower.end(), robot_lower.begin(), ::tolower);

    // PATCH: 플러그인 직접 가져와서 DOF와 limits를 같이 사용
    RobotIkPlugin* plugin = find_robot(robot_lower);
    if (plugin == nullptr) {
        std::cerr << "Robot IK plugin not loaded: " << robot_lower << '\n';
        return false;
    }
    int dof = plugin->num_joints;
    if (dof <= 0) {
        return false;  // Robot not found or invalid
    }

    // PATCH: 현재 조인트가 limit을 넘으면 바로 false
    if (!plugin->joint_limits.empty()) {
        if (!check_joint_limits(current_joints, dof, plugin->joint_limits)) {
            std::cerr << "[Joint Limit] current_joints violate limits for robot '"
                      << robot_lower << "'\n";
            return false;
        }
    }

    // Solve IK to get all solutions (이미 내부에서 joint limit 필터링)
    std::vector<IkSolutionData> all_solutions;
    bool ok = solveIK(robot_lower, tcp_pose, all_solutions);
    if (!ok || all_solutions.empty()) {
        return false;  // No solutions found
    }

    // Find nearest solution to current_joints
    IkReal min_distance = std::numeric_limits<IkReal>::infinity();
    int best_idx = -1;

    for (size_t i = 0; i < all_solutions.size(); ++i) {
        IkReal distance = compute_joint_distance(
            all_solutions[i].joints.data(),
            current_joints,
            dof,
            plugin->joint_limits
        );
        if (distance < min_distance) {
            min_distance = distance;
            best_idx = static_cast<int>(i);
        }
    }

    if (best_idx >= 0) {
        // Found nearest solution (이미 solveIK에서 limit 통과한 해만 있음)
        const auto& best_sol = all_solutions[best_idx];
        std::copy(best_sol.joints.begin(), best_sol.joints.end(), out_joints);
        *out_is_solvable = true;
        return true;
    }

    // Should not reach here
    return false;
}

} // namespace ikcore
