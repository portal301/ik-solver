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
#include <locale>
#include <codecvt>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace fs = std::filesystem;

namespace ikcore {

using IkReal = double;

// UTF-8 to UTF-16 conversion helper for Korean/Unicode paths
static std::wstring utf8_to_wstring(const std::string& utf8_str) {
    if (utf8_str.empty()) return std::wstring();
    
    int size_needed = MultiByteToWideChar(CP_UTF8, 0, utf8_str.c_str(), -1, nullptr, 0);
    if (size_needed <= 0) return std::wstring();
    
    std::wstring wstr(size_needed - 1, 0);  // -1 to exclude null terminator
    MultiByteToWideChar(CP_UTF8, 0, utf8_str.c_str(), -1, &wstr[0], size_needed);
    return wstr;
}



// Forward declarations
static void extractTransformFromMatrix(const IkReal* matrix_12, IkReal* eetrans, IkReal* eerot);

struct RobotIkPlugin {
    HMODULE dll = nullptr;
    ikfast::IkFastFunctions<IkReal> fns{};
    int num_joints = 0;
    int num_free = 0;
    int ikreal_size = sizeof(IkReal); // size reported by plugin
    std::string name;
    std::vector<JointLimit> joint_limits;  // Joint limits loaded from JSON
};

static std::unordered_map<std::string, RobotIkPlugin> g_plugins;

template <typename T>
T load_symbol(HMODULE dll, const char* name) {
    FARPROC fp = GetProcAddress(dll, name);
    if (!fp) {
        std::cerr << "GetProcAddress failed for symbol: " << name << std::endl;
        return nullptr;
    }
    return reinterpret_cast<T>(fp);
}

static std::string robot_name_from_path(const fs::path& p) {
    std::string stem = p.stem().string(); // "gp25_ikfast"
    const std::string suffix = "_ikfast";
    if (stem.size() > suffix.size() &&
        stem.compare(stem.size() - suffix.size(), suffix.size(), suffix) == 0) {
        return stem.substr(0, stem.size() - suffix.size());
    }
    return stem;
}

// Skip known dependency DLLs that are not IKFast plugins
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

// Helper: normalize robot name to lowercase for case-insensitive lookup
static std::string normalize_robot_name(const std::string& name) {
    std::string normalized = name;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(), ::tolower);
    return normalized;
}

// Helper: Check if tcp_pose values are finite (not NaN/Inf)
static bool is_tcp_pose_valid(const IkReal* tcp_pose, int size = 12) {
    if (!tcp_pose) return false;
    for (int i = 0; i < size; ++i) {
        if (!std::isfinite(tcp_pose[i])) {
            return false;  // NaN or Inf detected
        }
    }
    return true;
}

// Load joint limits from compiled data
// Returns true if robot_name found, fills out_limits
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

// Check if a joint is fixed (non-moving)
// Fixed joints have limits [0, 0] or very small range (< 1e-6 radians ≈ 0.00006°)
static bool is_fixed_joint(const JointLimit& limit) {
    const IkReal epsilon = 1e-6;  // ~0.00006 degrees
    return std::abs(limit.upper - limit.lower) < epsilon;
}

// Check if joint angles satisfy joint limits
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
        std::cerr << "[Joint Limit] Invalid num_joints: " << num_joints << std::endl;
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

// Add DLL search directories (robots dir + sibling dependencies dir)
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

    const char* vcpkg_root = std::getenv("VCPKG_ROOT");
    if (!vcpkg_root) vcpkg_root = "C:\\dev\\vcpkg";
    fs::path vcpkg_bin = fs::path(vcpkg_root) / "installed" / "x64-windows" / "bin";
    if (fs::exists(vcpkg_bin) && fs::is_directory(vcpkg_bin)) {
        dll_dirs.push_back(vcpkg_bin);
    }

    HMODULE kernel32 = GetModuleHandleW(L"kernel32.dll");
    auto setDefaultDllDirs = reinterpret_cast<SetDefaultDllDirectoriesFn>(
        GetProcAddress(kernel32, "SetDefaultDllDirectories"));
    auto addDllDirectory = reinterpret_cast<AddDllDirectoryFn>(
        GetProcAddress(kernel32, "AddDllDirectory"));

    // Prefer modern API if available (Win8+)
    if (setDefaultDllDirs && addDllDirectory) {
        setDefaultDllDirs(LOAD_LIBRARY_SEARCH_DEFAULT_DIRS | LOAD_LIBRARY_SEARCH_USER_DIRS);
        for (const auto& dir : dll_dirs) {
            addDllDirectory(dir.wstring().c_str());
        }
    } else {
        // Fallback for older systems: set robots dir as DLL dir
        SetDllDirectoryW(robots_dir.wstring().c_str());

        // Best-effort: prepend additional dirs to PATH so dependency resolution still works
        std::string current_path = std::getenv("PATH") ? std::getenv("PATH") : "";
        for (const auto& dir : dll_dirs) {
            std::string dir_str = dir.string();
            if (current_path.find(dir_str) == std::string::npos) {
                current_path = dir_str + ";" + current_path;
            }
        }
        SetEnvironmentVariableA("PATH", current_path.c_str());
    }
}

bool load_ik_plugins(const std::string& robots_dir) {
    g_plugins.clear();

    // Convert UTF-8 path to wide string for proper Korean/Unicode support
    std::cerr << "[INIT] Converting UTF-8 path to wide string: " << robots_dir << std::endl;
    std::wstring wide_path = utf8_to_wstring(robots_dir);
    std::wcerr << L"[INIT] Wide path: " << wide_path << std::endl;

    fs::path dir(wide_path);
    std::wcerr << L"[INIT] Filesystem path: " << dir.wstring() << std::endl;

    if (!fs::exists(dir)) {
        std::cerr << "[ERROR] robots directory does not exist: " << robots_dir << std::endl;
        std::wcerr << L"[ERROR] Wide path does not exist: " << wide_path << std::endl;
        return false;
    }

    if (!fs::is_directory(dir)) {
        std::cerr << "[ERROR] robots path is not a directory: " << robots_dir << std::endl;
        return false;
    }

    std::cerr << "[INIT] robots directory found and validated" << std::endl;

    // Configure DLL search paths (robots dir + dependencies)
    configure_dll_search_paths(dir);

    // CRITICAL: Load liblapack.dll and openblas.dll from robots directory FIRST
    // This prevents conda's LAPACK from being loaded when robot DLLs call dgeev_
    fs::path local_openblas = dir / "openblas.dll";
    fs::path local_lapack = dir / "liblapack.dll";

    if (fs::exists(local_openblas)) {
        HMODULE h = LoadLibraryW(local_openblas.wstring().c_str());
        if (h) {
            char path[MAX_PATH];
            GetModuleFileNameA(h, path, MAX_PATH);
            std::cerr << "Preloaded local OpenBLAS: " << path << std::endl;
        }
    }

    if (fs::exists(local_lapack)) {
        // Load with full path to ensure this specific DLL is loaded
        HMODULE h = LoadLibraryW(local_lapack.wstring().c_str());
        if (h) {
            char path[MAX_PATH];
            GetModuleFileNameA(h, path, MAX_PATH);
            std::cerr << "Preloaded local LAPACK: " << path << std::endl;
        } else {
            DWORD err = GetLastError();
            std::cerr << "Failed to load local LAPACK (error " << err << ")" << std::endl;
        }
    }

    // Also try vcpkg as fallback
    const char* vcpkg_root = std::getenv("VCPKG_ROOT");
    if (!vcpkg_root) vcpkg_root = "C:\\dev\\vcpkg";
    fs::path vcpkg_bin = fs::path(vcpkg_root) / "installed" / "x64-windows" / "bin";

    fs::path vcpkg_openblas = vcpkg_bin / "openblas.dll";
    if (fs::exists(vcpkg_openblas) && !fs::exists(local_openblas)) {
        HMODULE h = LoadLibraryW(vcpkg_openblas.wstring().c_str());
        if (h) {
            char path[MAX_PATH];
            GetModuleFileNameA(h, path, MAX_PATH);
            std::cerr << "Preloaded vcpkg OpenBLAS: " << path << std::endl;
        }
    }

    // Recursively search for IKFast plugin DLLs under robots_dir (manufacturer/model layout)
    std::cerr << "[INIT] Starting recursive directory search..." << std::endl;
    try {
        int dll_count = 0;
        for (auto& entry : fs::recursive_directory_iterator(dir)) {
            if (!entry.is_regular_file()) continue;
            fs::path p = entry.path();
            if (p.extension() != ".dll") continue;
            dll_count++;
            std::wcerr << L"[INIT] Found DLL #" << dll_count << L": " << p.wstring() << std::endl;

        if (is_dependency_dll(p)) {
            // Avoid treating dependency DLLs as IK plugins
            continue;
        }

        std::string robot = robot_name_from_path(p);

        // Use LOAD_WITH_ALTERED_SEARCH_PATH to search in the DLL's directory
        HMODULE dll = LoadLibraryExW(p.wstring().c_str(), nullptr,
                                      LOAD_WITH_ALTERED_SEARCH_PATH);
        if (!dll) {
            DWORD err = GetLastError();
            std::wcerr << L"[ERROR] Failed to LoadLibrary: " << p.wstring()
                       << L" (error code: " << err << L")" << std::endl;
            continue;
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

        // Validate IkReal size compatibility (must match host sizeof(IkReal))
        if (plugin.fns._GetIkRealSize) {
            plugin.ikreal_size = plugin.fns._GetIkRealSize();
        }
        if (plugin.ikreal_size != static_cast<int>(sizeof(IkReal))) {
            std::cerr << "IkReal size mismatch for plugin '" << robot << "': plugin="
                      << plugin.ikreal_size << ", host=" << sizeof(IkReal)
                      << ". Plugin will be skipped. Rebuild plugin with matching precision." << std::endl;
            FreeLibrary(dll);
            continue;
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

        // Store with lowercase name for case-insensitive lookup
        std::string robot_lower = normalize_robot_name(robot);
        g_plugins.emplace(robot_lower, std::move(plugin));
        }
        std::cerr << "[INIT] Finished directory search. Found " << g_plugins.size() << " plugins." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception during directory iteration: " << e.what() << std::endl;
        return false;
    } catch (...) {
        std::cerr << "[ERROR] Unknown exception during directory iteration" << std::endl;
        return false;
    }

    return !g_plugins.empty();
}

static RobotIkPlugin* find_robot(const std::string& name) {
    std::string normalized = normalize_robot_name(name);
    auto it = g_plugins.find(normalized);
    if (it == g_plugins.end()) return nullptr;
    return &it->second;
}

bool solveIK(
    const std::string& robot_name,
    const IkReal* tcp_pose,
    std::vector<IkSolutionData>& out_solutions
) {
    RobotIkPlugin* plugin = find_robot(robot_name);
    if (!plugin) {
        std::cerr << "Robot IK plugin not loaded: " << robot_name << std::endl;
        return false;
    }
    if (plugin->ikreal_size != static_cast<int>(sizeof(IkReal))) {
        std::cerr << "IkReal size mismatch for robot '" << robot_name << "': plugin="
                  << plugin->ikreal_size << ", host=" << sizeof(IkReal)
                  << ". Aborting solveIK." << std::endl;
        return false;
    }
    if (!plugin->fns._ComputeIk) {
        std::cerr << "ComputeIk not available\n";
        return false;
    }

    // PATCH: Input validation - check tcp_pose for NaN/Inf
    if (!is_tcp_pose_valid(tcp_pose, 12)) {
        std::cerr << "[IK Safety] tcp_pose contains NaN or Inf values. Aborting solveIK.\n";
        return false;
    }

    // Apply manufacturer-specific coordinate transformation if available
    IkReal transformed_pose[12];
    const IkReal* pose_to_use = tcp_pose;

    std::string manufacturer = detect_manufacturer(robot_name);
    if (!manufacturer.empty()) {
        auto it = coordinate_transform_data::MANUFACTURER_TRANSFORMS.find(manufacturer);
        if (it != coordinate_transform_data::MANUFACTURER_TRANSFORMS.end()) {
            // Apply transformation: transformed_pose = tcp_pose * coord_transform
            std::cout << "[Coordinate Transform] original pose" << std::endl;
            std::cout << " R: [" 
                      << tcp_pose[0] << ", " << tcp_pose[1] << ", " << tcp_pose[2] << "; "
                      << tcp_pose[4] << ", " << tcp_pose[5] << ", " << tcp_pose[6] << "; "
                      << tcp_pose[8] << ", " << tcp_pose[9] << ", " << tcp_pose[10] << "]"
                      << std::endl;
            
            multiply_transforms(tcp_pose, it->second.data(), transformed_pose);
            std::cout << "[Coordinate Transform] transformed_pose" << std::endl;
            std::cout << " R: [" 
                      << transformed_pose[0] << ", " << transformed_pose[1] << ", " << transformed_pose[2] << "; "
                      << transformed_pose[4] << ", " << transformed_pose[5] << ", " << transformed_pose[6] << "; "
                      << transformed_pose[8] << ", " << transformed_pose[9] << ", " << transformed_pose[10] << "]"
                      << std::endl;
            pose_to_use = transformed_pose;

            // Debug log (can be disabled in production)
            static bool debug_transform = (std::getenv("IK_DEBUG_TRANSFORM") != nullptr);
            if (debug_transform) {
                std::cout << "[Coordinate Transform] Applied '" << manufacturer
                          << "' transform for robot '" << robot_name << "'\n";
            }
        }
    }

    // Extract position and rotation from 4x4 matrix (12 elements)
    IkReal eetrans[3];
    IkReal eerot[9];
    extractTransformFromMatrix(pose_to_use, eetrans, eerot);

    ikfast::IkSolutionList<IkReal> solutions;
    bool ok = plugin->fns._ComputeIk(eetrans, eerot, nullptr, solutions);

    if (!ok) return false;

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
            // Debug which joint(s) violated limits when enabled via IK_DEBUG_LIMITS
            int dof = plugin->num_joints;
            const auto& limits = plugin->joint_limits;
            if (limits.size() < static_cast<size_t>(dof)) {
                std::cerr << "[Joint Limit DEBUG] Limit count mismatch: limits="
                            << limits.size() << ", expected_joints=" << dof << "\n";
            } else {
                for (int ji = 0; ji < dof; ++ji) {
                    const JointLimit& lim = limits[ji];
                    if (is_fixed_joint(lim)) continue;  // skip fixed joints

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

int get_num_joints(const std::string& robot_name) {
    RobotIkPlugin* p = find_robot(robot_name);
    return p ? p->num_joints : -1;
}

int get_num_free_parameters(const std::string& robot_name) {
    RobotIkPlugin* p = find_robot(robot_name);
    return p ? p->num_free : -1;
}

bool computeFK(
    const std::string& robot_name,
    const IkReal* joints,
    IkReal* eetrans,
    IkReal* eerot
) {
    RobotIkPlugin* plugin = find_robot(robot_name);
    if (!plugin) {
        std::cerr << "Robot IK plugin not loaded: " << robot_name << std::endl;
        return false;
    }
    if (!plugin->fns._ComputeFk) {
        std::cerr << "ComputeFk not available for robot: " << robot_name << std::endl;
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
    IkReal fk_trans_tool[3];
    IkReal fk_rot_tool[9];
    plugin->fns._ComputeFk(joints, fk_trans_tool, fk_rot_tool);

    // Apply inverse coordinate transformation to convert back to base frame
    std::string manufacturer = detect_manufacturer(robot_name);
    if (!manufacturer.empty()) {
        auto it = coordinate_transform_data::MANUFACTURER_TRANSFORMS.find(manufacturer);
        if (it != coordinate_transform_data::MANUFACTURER_TRANSFORMS.end()) {
            // Build FK result as 12-element matrix (tool frame)
            IkReal fk_pose_tool[12] = {
                fk_rot_tool[0], fk_rot_tool[1], fk_rot_tool[2], fk_trans_tool[0],
                fk_rot_tool[3], fk_rot_tool[4], fk_rot_tool[5], fk_trans_tool[1],
                fk_rot_tool[6], fk_rot_tool[7], fk_rot_tool[8], fk_trans_tool[2]
            };

            // Compute inverse transform: T_base = T_tool * T_inv
            // For rotation-only transform: T_inv = T^T (transpose)
            const auto& T_forward = it->second;
            IkReal T_inverse[12] = {
                T_forward[0], T_forward[4], T_forward[8],  0.0,  // R^T row 1
                T_forward[1], T_forward[5], T_forward[9],  0.0,  // R^T row 2
                T_forward[2], T_forward[6], T_forward[10], 0.0   // R^T row 3
            };

            IkReal fk_pose_base[12];
            multiply_transforms(fk_pose_tool, T_inverse, fk_pose_base);

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
            static bool debug_transform = (std::getenv("IK_DEBUG_TRANSFORM") != nullptr);
            if (debug_transform) {
                std::cout << "[Coordinate Transform] Applied inverse '" << manufacturer
                          << "' transform for FK of robot '" << robot_name << "'\n";
            }
        } else {
            // No transform - direct copy
            eetrans[0] = fk_trans_tool[0];
            eetrans[1] = fk_trans_tool[1];
            eetrans[2] = fk_trans_tool[2];
            std::copy(fk_rot_tool, fk_rot_tool + 9, eerot);
        }
    } else {
        // No manufacturer - direct copy
        eetrans[0] = fk_trans_tool[0];
        eetrans[1] = fk_trans_tool[1];
        eetrans[2] = fk_trans_tool[2];
        std::copy(fk_rot_tool, fk_rot_tool + 9, eerot);
    }

    return true;
}

// Helper: Detect manufacturer from robot name based on prefix patterns
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

// Helper: Multiply two 4x4 transformation matrices (12-element format)
// Input format: [R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz]
// Result = T_AB * T_BC (T_AB transforms from B to A, T_BC from C to B)
void multiply_transforms(const IkReal* T_AB, const IkReal* T_BC, IkReal* T_AC) {
    // Extract rotation matrices (3x3)
    IkReal R_AB[9] = {T_AB[0], T_AB[1], T_AB[2],
                      T_AB[4], T_AB[5], T_AB[6],
                      T_AB[8], T_AB[9], T_AB[10]};

    IkReal R_BC[9] = {T_BC[0], T_BC[1], T_BC[2],
                      T_BC[4], T_BC[5], T_BC[6],
                      T_BC[8], T_BC[9], T_BC[10]};

    // Extract translation vectors
    IkReal t_AB[3] = {T_AB[3], T_AB[7], T_AB[11]};
    IkReal t_BC[3] = {T_BC[3], T_BC[7], T_BC[11]};

    // Compute R_AC = R_AB * R_BC (3x3 matrix multiplication)
    IkReal R_AC[9];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_AC[i * 3 + j] = 0.0;
            for (int k = 0; k < 3; ++k) {
                R_AC[i * 3 + j] += R_AB[i * 3 + k] * R_BC[k * 3 + j];
            }
        }
    }

    // Compute t_AC = R_AB * t_BC + t_AB
    IkReal t_AC[3];
    for (int i = 0; i < 3; ++i) {
        t_AC[i] = t_AB[i];
        for (int j = 0; j < 3; ++j) {
            t_AC[i] += R_AB[i * 3 + j] * t_BC[j];
        }
    }

    // Pack result into 12-element format
    T_AC[0]  = R_AC[0];  T_AC[1]  = R_AC[1];  T_AC[2]  = R_AC[2];  T_AC[3]  = t_AC[0];
    T_AC[4]  = R_AC[3];  T_AC[5]  = R_AC[4];  T_AC[6]  = R_AC[5];  T_AC[7]  = t_AC[1];
    T_AC[8]  = R_AC[6];  T_AC[9]  = R_AC[7];  T_AC[10] = R_AC[8];  T_AC[11] = t_AC[2];
}

// Helper: Extract translation and rotation from 4x4 matrix (12 elements)
// Input format: [R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz]
static void extractTransformFromMatrix(const IkReal* matrix_12, IkReal* eetrans, IkReal* eerot) {
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

// Helper: Normalize angle to [-pi, pi]
static IkReal normalizeAngle(IkReal angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// Helper: Normalize angle difference to [-pi, pi]
// This is critical for computing distances between angles correctly
// Example: -170° to +170° = 20° difference, not 340°
static IkReal normalizeAngleDiff(IkReal diff) {
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return diff;
}

static void wrap_joints_to_nearest(
    IkReal* joints,
    const IkReal* target_joints,
    int dof,
    const std::vector<JointLimit>& limits
) {
    for (int i = 0; i < dof; ++i) {
        if (!limits.empty() && i < static_cast<int>(limits.size())) {
            IkReal range = limits[i].upper - limits[i].lower;
            if (range >= 2 * M_PI) {
                IkReal diff = joints[i] - target_joints[i];
                IkReal k = std::round(diff / (2 * M_PI));
                joints[i] -= k * 2 * M_PI;
            }
        }
    }
}

// Helper: Compute Euclidean distance between two joint configurations with angle wrapping
static IkReal computeJointDistance(
    const IkReal* joints1,
    const IkReal* joints2,
    int dof,
    const std::vector<JointLimit>& limits
) {
    IkReal sum = 0.0;
    for (int i = 0; i < dof; ++i) {
        IkReal diff = joints1[i] - joints2[i];

        if (!limits.empty() && i < static_cast<int>(limits.size())) {
            IkReal range = limits[i].upper - limits[i].lower;
            if (range >= 2 * M_PI) {
                IkReal abs_diff = std::abs(diff);
                IkReal wrapped_diff = abs_diff - 2 * M_PI;
                
                if (std::abs(wrapped_diff) < abs_diff) {
                    diff = (diff > 0) ? wrapped_diff : -wrapped_diff;
                }
            } else {
                diff = normalize_angle(diff);
            }
        } else {
            diff = normalize_angle(diff);
        }

        sum += diff * diff;
    }
    return std::sqrt(sum);
}

// Helper: Check if solution matches configuration
static IkReal safeMidpoint(const IkReal* joint_min, const IkReal* joint_max, int idx) {
    IkReal mn = joint_min[idx];
    IkReal mx = joint_max[idx];
    if (std::isfinite(mn) && std::isfinite(mx)) {
        return 0.5 * (mn + mx);
    }
    return 0.0;
}

static IkReal getJ3Reference(const std::string& robot_lower,
                             const IkReal* joint_min,
                             const IkReal* joint_max,
                             int idx_elbow) {
    // Robot-specific reference (hard-coded for now)
    if (robot_lower == "kj125") {
        return M_PI / 2.0;  // 90 degrees
    }

    // Fallback: midpoint
    return safeMidpoint(joint_min, joint_max, idx_elbow);
}

static bool matchesConfiguration(
    const IkReal* joints,      // [0..5]
    const IkReal* joint_min,   // [0..5]
    const IkReal* joint_max,   // [0..5]
    int frontback_cfg,         // 0 = FRONT, 1 = REAR
    int elbow_cfg,             // 0 = UP,    1 = DOWN
    int wrist_cfg,             // 0 = N_FLIP,1 = FLIP
    const IkReal* tcp_pose,    // 12 elements [R|T]
    const std::string& robot_lower,
    IkReal eps
) {
    // Default epsilon if zero/negative provided
    if (eps <= 0) eps = 1e-5;

    constexpr int IDX_FRONTBACK = 0;  // joint1 (J1) - index 0
    constexpr int IDX_ELBOW     = 2;  // joint3 (J3) - index 2
    constexpr int IDX_WRIST     = 4;  // joint5 (J5) - index 4

    IkReal j1 = normalizeAngle(joints[IDX_FRONTBACK]);
    IkReal j3 = normalizeAngle(joints[IDX_ELBOW]);
    IkReal j5 = joints[IDX_WRIST];

    // Translation from tcp_pose
    IkReal tx = tcp_pose ? tcp_pose[3]  : 0.0;
    IkReal ty = tcp_pose ? tcp_pose[7]  : 0.0;
    // IkReal tz = tcp_pose ? tcp_pose[11] : 0.0; // not needed for yaw

    // 1) FRONT/BACK using j1 against target direction
    // j1 = 0 aligns TCP toward -Y (reference) for KJ125
    // FRONT: j1와 TCP가 같은 방향 (각도 차이 < 90도)
    // REAR: j1와 TCP가 반대 방향 (각도 차이 >= 90도)
    IkReal yaw_target = std::atan2(tx, -ty);     // Note: (x,-y) -> yaw; j1=0 => -Y (KJ125)
    IkReal diff = normalizeAngle(yaw_target - j1);

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
        std::cout << "  FRONT/BACK: |diff| <= pi/2 = " << (std::abs(diff) <= (M_PI / 2.0 + eps))
                  << ", requested=" << frontback_cfg << ", match=" << frontback_match << "\n";
    }

    // 2) UP/DOWN using J3 reference (robot-specific)
    // J3 < ref => UP (2), J3 >= ref => DOWN (3)
    IkReal j3_ref = getJ3Reference(robot_lower, joint_min, joint_max, IDX_ELBOW);
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
    IkReal j5n = normalizeAngle(j5);
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
    if (!plugin) {
        std::cerr << "Robot IK plugin not loaded: " << robot_lower << std::endl;
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
            if (matchesConfiguration(
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

        if (!config_matches.empty()) break;  // Found matches at this threshold
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
            IkReal distance = computeJointDistance(
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
        // OLD BEHAVIOR: Return first match (backward compatible)
        const auto* first_match = config_matches[0];
        std::copy(first_match->joints.begin(), first_match->joints.end(), out_joints);
        *out_is_solvable = true;
        return true;
    }

    return false;
}

bool solveIKWithJoint(
    const std::string& robot_name,
    const IkReal* tcp_pose,
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
    if (!plugin) {
        std::cerr << "Robot IK plugin not loaded: " << robot_lower << std::endl;
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
        IkReal distance = computeJointDistance(
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
        auto best_sol = all_solutions[best_idx].joints;
        wrap_joints_to_nearest(best_sol.data(), current_joints, dof, plugin->joint_limits);
        std::copy(best_sol.begin(), best_sol.end(), out_joints);
        *out_is_solvable = true;
        return true;
    }

    // Should not reach here
    return false;
}

bool get_joint_limits(const std::string& robot_name, std::vector<JointLimit>& out_limits) {
    return load_joint_limits_from_data(robot_name, out_limits);
}

} // namespace ikcore