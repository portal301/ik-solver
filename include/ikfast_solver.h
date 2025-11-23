/**
 * @file ikfast_solver.h
 * @brief Unified IKFast solver interface for multiple robot models
 *
 * This header provides a simple, unified API for computing inverse kinematics
 * across multiple robot models using pre-generated IKFast solvers.
 *
 * @author IKFast Generator Team
 * @date 2025
 */

#ifndef IKFAST_SOLVER_H
#define IKFAST_SOLVER_H

#include <vector>
#include <string>
#include <memory>

namespace ikfast_robotics {

/**
 * @brief 6D pose structure (position + orientation)
 */
struct Pose6D {
    double x, y, z;        // Position in meters
    double rx, ry, rz;     // Orientation in radians (roll-pitch-yaw)

    Pose6D() : x(0), y(0), z(0), rx(0), ry(0), rz(0) {}
    Pose6D(double x_, double y_, double z_, double rx_, double ry_, double rz_)
        : x(x_), y(y_), z(z_), rx(rx_), ry(ry_), rz(rz_) {}
};

/**
 * @brief Wrist configuration enum
 */
enum class WristConfig {
    NONE = 0,      // No preference
    FLIP = 1,      // Wrist flip configuration
    NO_FLIP = 2    // Wrist no-flip configuration
};

/**
 * @brief IK solution result structure
 */
struct IKSolution {
    std::vector<double> joints;  // Joint angles in radians
    bool is_valid;                // Whether this solution is valid
    double error;                 // FK verification error (meters)

    IKSolution() : is_valid(false), error(0.0) {}
};

/**
 * @brief Main IKFast solver class - unified interface for all robots
 *
 * Usage example:
 * @code
 *   IKFastSolver solver("yaskawa_gp4");
 *   Pose6D target(0.5, 0.0, 0.4, 0.0, 0.0, 0.0);
 *   std::vector<double> current_joints = {0, 0, 0, 0, 0, 0};
 *   IKSolution solution;
 *
 *   if (solver.solveIK(target, current_joints, WristConfig::NONE, solution)) {
 *       // Use solution.joints
 *   }
 * @endcode
 */
class IKFastSolver {
public:
    /**
     * @brief Constructor - creates solver for specified robot model
     * @param robot_name Robot model identifier (e.g., "yaskawa_gp4", "kawasaki_kj125")
     */
    explicit IKFastSolver(const std::string& robot_name);

    /**
     * @brief Destructor
     */
    ~IKFastSolver();

    /**
     * @brief Solve inverse kinematics for target TCP pose
     * @param tcp_pose Target TCP 6D pose (position + orientation)
     * @param current_joints Current joint angles (used for finding closest solution)
     * @param wrist_config Preferred wrist configuration
     * @param[out] solution Output IK solution (closest to current_joints)
     * @return true if solution found, false otherwise
     */
    bool solveIK(
        const Pose6D& tcp_pose,
        const std::vector<double>& current_joints,
        WristConfig wrist_config,
        IKSolution& solution
    );

    /**
     * @brief Solve IK and return all solutions
     * @param tcp_pose Target TCP 6D pose
     * @param[out] solutions All valid IK solutions
     * @return Number of solutions found
     */
    int solveIKAll(
        const Pose6D& tcp_pose,
        std::vector<IKSolution>& solutions
    );

    /**
     * @brief Compute forward kinematics
     * @param joints Joint angles in radians
     * @param[out] tcp_pose Resulting TCP pose
     * @return true if FK computation successful
     */
    bool computeFK(
        const std::vector<double>& joints,
        Pose6D& tcp_pose
    );

    /**
     * @brief Get robot model name
     */
    std::string getRobotName() const;

    /**
     * @brief Get number of degrees of freedom
     */
    int getDOF() const;

    /**
     * @brief Check if solver is initialized
     */
    bool isValid() const;

    /**
     * @brief Get list of all supported robot models
     */
    static std::vector<std::string> getSupportedRobots();

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace ikfast_robotics

#endif // IKFAST_SOLVER_H
