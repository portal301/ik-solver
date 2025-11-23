/**
 * @file ikfast_solver.cpp
 * @brief Implementation of unified IKFast solver
 */

#include "ikfast_solver.h"
#include "ikfast.h"
#include <cmath>
#include <limits>
#include <algorithm>
#include <map>
#include <stdexcept>

// Forward declarations for robot-specific IKFast solvers
// These will be linked from individual robot cpp files
namespace kj125_ikfast {
    using namespace ikfast;
    extern int GetNumFreeParameters();
    extern const int* GetFreeIndices();
    extern int GetNumJoints();
    extern int GetIkRealSize();
    extern const char* GetKinematicsHash();
    extern bool ComputeIk(const IkReal* eetrans, const IkReal* eerot,
                          const IkReal* pfree, IkSolutionListBase<IkReal>& solutions);
    extern void ComputeFk(const IkReal* joints, IkReal* eetrans, IkReal* eerot);
}

// Add more robot namespaces here as they are generated
// namespace gp4_ikfast { ... }
// namespace gp7_ikfast { ... }

namespace ikfast_robotics {

// Robot-specific solver interface
struct RobotSolverInterface {
    virtual ~RobotSolverInterface() = default;
    virtual bool computeIK(const double* eetrans, const double* eerot,
                           std::vector<std::vector<double>>& solutions) = 0;
    virtual bool computeFK(const std::vector<double>& joints,
                           double* eetrans, double* eerot) = 0;
    virtual int getDOF() const = 0;
    virtual std::string getName() const = 0;
};

// KJ125 solver implementation
class KJ125Solver : public RobotSolverInterface {
public:
    bool computeIK(const double* eetrans, const double* eerot,
                   std::vector<std::vector<double>>& solutions) override {
        solutions.clear();

        // Create IKFast solution list
        ikfast::IkSolutionList<ikfast::IkReal> ik_solutions;

        // Call IKFast solver
        bool success = kj125_ikfast::ComputeIk(eetrans, eerot, nullptr, ik_solutions);

        if (!success) {
            return false;
        }

        // Extract all solutions
        size_t num_solutions = ik_solutions.GetNumSolutions();
        if (num_solutions == 0) {
            return false;
        }

        int num_joints = getDOF();
        solutions.reserve(num_solutions);

        for (size_t i = 0; i < num_solutions; ++i) {
            const ikfast::IkSolutionBase<ikfast::IkReal>& sol = ik_solutions.GetSolution(i);
            std::vector<double> joint_solution(num_joints);
            sol.GetSolution(joint_solution.data(), nullptr);
            solutions.push_back(joint_solution);
        }

        return true;
    }

    bool computeFK(const std::vector<double>& joints,
                   double* eetrans, double* eerot) override {
        if (joints.size() != static_cast<size_t>(getDOF())) {
            return false;
        }
        kj125_ikfast::ComputeFk(joints.data(), eetrans, eerot);
        return true;
    }

    int getDOF() const override {
        return kj125_ikfast::GetNumJoints();
    }

    std::string getName() const override {
        return "kawasaki_kj125";
    }
};

// Robot factory - maps robot names to solver implementations
static std::map<std::string, std::function<std::unique_ptr<RobotSolverInterface>()>> robot_factory = {
    {"kawasaki_kj125", []() { return std::make_unique<KJ125Solver>(); }},
    {"kj125", []() { return std::make_unique<KJ125Solver>(); }},
    // Add more robots here as they are implemented
    // {"yaskawa_gp4", []() { return std::make_unique<GP4Solver>(); }},
};

// Helper: Convert Pose6D to transformation matrix
static void pose6DToTransform(const Pose6D& pose, double* trans, double* rot) {
    // Translation
    trans[0] = pose.x;
    trans[1] = pose.y;
    trans[2] = pose.z;

    // Rotation matrix from roll-pitch-yaw
    double cr = std::cos(pose.rx), sr = std::sin(pose.rx);
    double cp = std::cos(pose.ry), sp = std::sin(pose.ry);
    double cy = std::cos(pose.rz), sy = std::sin(pose.rz);

    // ZYX Euler angles (roll-pitch-yaw) to rotation matrix
    rot[0] = cy * cp;
    rot[1] = cy * sp * sr - sy * cr;
    rot[2] = cy * sp * cr + sy * sr;
    rot[3] = sy * cp;
    rot[4] = sy * sp * sr + cy * cr;
    rot[5] = sy * sp * cr - cy * sr;
    rot[6] = -sp;
    rot[7] = cp * sr;
    rot[8] = cp * cr;
}

// Helper: Convert transformation matrix to Pose6D
static void transformToPose6D(const double* trans, const double* rot, Pose6D& pose) {
    pose.x = trans[0];
    pose.y = trans[1];
    pose.z = trans[2];

    // Rotation matrix to roll-pitch-yaw
    pose.ry = std::atan2(-rot[6], std::sqrt(rot[0]*rot[0] + rot[3]*rot[3]));

    if (std::abs(std::cos(pose.ry)) > 1e-6) {
        pose.rz = std::atan2(rot[3], rot[0]);
        pose.rx = std::atan2(rot[7], rot[8]);
    } else {
        pose.rz = std::atan2(-rot[1], rot[4]);
        pose.rx = 0.0;
    }
}

// Helper: Calculate joint distance
static double jointDistance(const std::vector<double>& j1, const std::vector<double>& j2) {
    double dist = 0.0;
    for (size_t i = 0; i < j1.size() && i < j2.size(); ++i) {
        double diff = j1[i] - j2[i];
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

// ============================================================================
// IKFastSolver::Impl
// ============================================================================

class IKFastSolver::Impl {
public:
    std::unique_ptr<RobotSolverInterface> solver_;
    std::string robot_name_;
    bool is_valid_;

    Impl(const std::string& robot_name) : robot_name_(robot_name), is_valid_(false) {
        auto it = robot_factory.find(robot_name);
        if (it != robot_factory.end()) {
            solver_ = it->second();
            is_valid_ = (solver_ != nullptr);
        }
    }
};

// ============================================================================
// IKFastSolver Public API
// ============================================================================

IKFastSolver::IKFastSolver(const std::string& robot_name)
    : pimpl_(std::make_unique<Impl>(robot_name)) {
}

IKFastSolver::~IKFastSolver() = default;

bool IKFastSolver::solveIK(
    const Pose6D& tcp_pose,
    const std::vector<double>& current_joints,
    WristConfig wrist_config,
    IKSolution& solution)
{
    if (!pimpl_->is_valid_ || !pimpl_->solver_) {
        return false;
    }

    // Convert pose to transformation matrix
    double trans[3];
    double rot[9];
    pose6DToTransform(tcp_pose, trans, rot);

    // Compute all IK solutions
    std::vector<std::vector<double>> all_solutions;
    if (!pimpl_->solver_->computeIK(trans, rot, all_solutions)) {
        return false;
    }

    if (all_solutions.empty()) {
        return false;
    }

    // Find closest solution to current joints
    double min_dist = std::numeric_limits<double>::max();
    int best_idx = 0;

    for (size_t i = 0; i < all_solutions.size(); ++i) {
        double dist = jointDistance(current_joints, all_solutions[i]);
        if (dist < min_dist) {
            min_dist = dist;
            best_idx = i;
        }
    }

    // Verify solution with FK
    double fk_trans[3];
    double fk_rot[9];
    pimpl_->solver_->computeFK(all_solutions[best_idx], fk_trans, fk_rot);

    // Calculate error
    double pos_error = std::sqrt(
        std::pow(trans[0] - fk_trans[0], 2) +
        std::pow(trans[1] - fk_trans[1], 2) +
        std::pow(trans[2] - fk_trans[2], 2)
    );

    solution.joints = all_solutions[best_idx];
    solution.is_valid = true;
    solution.error = pos_error;

    return true;
}

int IKFastSolver::solveIKAll(
    const Pose6D& tcp_pose,
    std::vector<IKSolution>& solutions)
{
    if (!pimpl_->is_valid_ || !pimpl_->solver_) {
        return 0;
    }

    double trans[3];
    double rot[9];
    pose6DToTransform(tcp_pose, trans, rot);

    std::vector<std::vector<double>> all_solutions;
    if (!pimpl_->solver_->computeIK(trans, rot, all_solutions)) {
        return 0;
    }

    solutions.clear();
    for (const auto& sol : all_solutions) {
        IKSolution ik_sol;
        ik_sol.joints = sol;
        ik_sol.is_valid = true;
        ik_sol.error = 0.0; // Could compute FK error here
        solutions.push_back(ik_sol);
    }

    return static_cast<int>(solutions.size());
}

bool IKFastSolver::computeFK(
    const std::vector<double>& joints,
    Pose6D& tcp_pose)
{
    if (!pimpl_->is_valid_ || !pimpl_->solver_) {
        return false;
    }

    double trans[3];
    double rot[9];

    if (!pimpl_->solver_->computeFK(joints, trans, rot)) {
        return false;
    }

    transformToPose6D(trans, rot, tcp_pose);
    return true;
}

std::string IKFastSolver::getRobotName() const {
    return pimpl_->robot_name_;
}

int IKFastSolver::getDOF() const {
    if (!pimpl_->is_valid_ || !pimpl_->solver_) {
        return 0;
    }
    return pimpl_->solver_->getDOF();
}

bool IKFastSolver::isValid() const {
    return pimpl_->is_valid_;
}

std::vector<std::string> IKFastSolver::getSupportedRobots() {
    std::vector<std::string> robots;
    for (const auto& pair : robot_factory) {
        robots.push_back(pair.first);
    }
    return robots;
}

} // namespace ikfast_robotics
