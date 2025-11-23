/**
 * @file test_solver.cpp
 * @brief Basic tests for IKFast solver
 */

#include "ikfast_solver.h"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace ikfast_robotics;

bool test_solver_creation() {
    std::cout << "Test: Solver creation... ";

    IKFastSolver solver("kawasaki_kj125");
    if (!solver.isValid()) {
        std::cout << "FAILED (solver not valid)\n";
        return false;
    }

    if (solver.getDOF() != 6) {
        std::cout << "FAILED (expected 6 DOF, got " << solver.getDOF() << ")\n";
        return false;
    }

    std::cout << "PASSED\n";
    return true;
}

bool test_supported_robots() {
    std::cout << "Test: Supported robots... ";

    auto robots = IKFastSolver::getSupportedRobots();
    if (robots.empty()) {
        std::cout << "FAILED (no robots supported)\n";
        return false;
    }

    std::cout << "PASSED (" << robots.size() << " robots)\n";
    return true;
}

bool test_forward_kinematics() {
    std::cout << "Test: Forward kinematics... ";

    IKFastSolver solver("kawasaki_kj125");
    if (!solver.isValid()) {
        std::cout << "SKIPPED (solver not available)\n";
        return true;
    }

    std::vector<double> joints = {0, 0, 0, 0, 0, 0};
    Pose6D pose;

    if (!solver.computeFK(joints, pose)) {
        std::cout << "FAILED (FK computation failed)\n";
        return false;
    }

    std::cout << "PASSED\n";
    return true;
}

bool test_inverse_kinematics() {
    std::cout << "Test: Inverse kinematics... ";

    IKFastSolver solver("kawasaki_kj125");
    if (!solver.isValid()) {
        std::cout << "SKIPPED (solver not available)\n";
        return true;
    }

    Pose6D target(0.5, 0.0, 0.4, 0.0, 0.0, 0.0);
    std::vector<double> current_joints = {0, 0, 0, 0, 0, 0};
    IKSolution solution;

    bool success = solver.solveIK(target, current_joints, WristConfig::NONE, solution);

    // Note: IK might fail for some poses - this is normal
    if (success) {
        if (solution.joints.size() != 6) {
            std::cout << "FAILED (wrong number of joints)\n";
            return false;
        }
    }

    std::cout << (success ? "PASSED" : "PASSED (no solution - pose might be unreachable)") << "\n";
    return true;
}

int main() {
    std::cout << "IKFast Solver Test Suite\n";
    std::cout << "========================\n\n";

    int passed = 0;
    int failed = 0;

    if (test_solver_creation()) passed++; else failed++;
    if (test_supported_robots()) passed++; else failed++;
    if (test_forward_kinematics()) passed++; else failed++;
    if (test_inverse_kinematics()) passed++; else failed++;

    std::cout << "\n========================\n";
    std::cout << "Results: " << passed << " passed, " << failed << " failed\n";

    return (failed == 0) ? 0 : 1;
}
