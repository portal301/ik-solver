/**
 * ikfast_pybind.cpp
 * Wrapper for IKFast Python integration
 *
 * Provides Python bindings for IKFast core functions using pybind11.
 * Functions include initialization, IK/FK solving, and joint limit retrieval.
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>       // std::vector, std::string 변환
#include <pybind11/numpy.h>
#include "ikfast_core.hpp"

namespace py = pybind11;
using IkReal = ikcore::IkReal;

PYBIND11_MODULE(ikfast_solver, m) { //// m stands for module
    ///Module for IKFast multi-robot solver using dynamically loaded DLLs

    /**
     * Documentation string
     */
    m.doc() = "IKFast multi-robot solver using dynamically loaded DLLs";

    /**
     * Binding of load_ik_plugins
     * @param[in] robots_dir Directory containing robot IK DLLs
     * @return True if at least one plugin loaded, False otherwise
     */ 
    m.def("load_ik_plugins",
          [](const std::string& robots_dir) {
              return ikcore::load_ik_plugins(robots_dir);
          },
          py::arg("robots_dir"),
          R"pbdoc(
              Load IK plugins from robots directory.

              Returns:
                  bool: True if at least one plugin loaded, False otherwise
          )pbdoc");

    /**
     * Binding of get_num_joints
     * @param[in] robot_name Name of the robot
     * @return Number of joints
     */ 
    m.def("get_num_joints",
          &ikcore::get_num_joints,
          py::arg("robot_name"));


    /**
     * Binding of get_num_free_parameters
     * @param[in] robot_name Name of the robot
     * @return Number of free parameters
     */
    m.def("get_num_free_parameters",
          &ikcore::get_num_free_parameters,
          py::arg("robot_name"));


    /**
     * Binding of get_joint_limits
     * @param[in] robot_name Name of the robot
     * @return List of joint limits
     */
    m.def("get_joint_limits",
          [](const std::string& robot_name) {
              std::vector<ikcore::JointLimit> limits;
              if (!ikcore::get_joint_limits(robot_name, limits) || limits.empty()) {
                  return py::list();
              }
              py::list out;
              for (const auto& lim : limits) {
                  py::dict item;
                  item["lower"] = lim.lower;
                  item["upper"] = lim.upper;
                  out.append(std::move(item));
              }
              return out;
          },
          py::arg("robot_name"),
          R"pbdoc(
              Get joint limits for the robot.

              Returns:
                  list of dicts with keys: lower, upper (radians)
          )pbdoc");

    /**
     * Binding of solve_ik
     * @param[in] robot_name Name of the robot
     * @param[in] tcp_pose TCP pose as a 12-element array representing a 4x4 matrix
     * @return Tuple of (solutions, is_solvable)
     */
    m.def("solve_ik",
          [](const std::string& robot_name,
             const py::array_t<IkReal>& tcp_pose) {

              if (tcp_pose.size() != 12) {
                  // Return empty result instead of throwing
                  return py::make_tuple(std::vector<std::vector<IkReal>>{}, false);
              }

              std::vector<ikcore::IkSolutionData> sols;
              bool ok = ikcore::solveIK(robot_name, tcp_pose.data(), sols);
              if (!ok) {
                  return py::make_tuple(std::vector<std::vector<IkReal>>{}, false);
              }

              std::vector<std::vector<IkReal>> out;
              out.reserve(sols.size());
              for (auto& s : sols) { out.push_back(s.joints);
              }
              bool is_solvable = !out.empty();
              return py::make_tuple(out, is_solvable);
          },
          py::arg("robot_name"),
          py::arg("tcp_pose"),
          R"pbdoc(
              Solve Inverse Kinematics - Returns ALL solutions

              Args:
                  robot_name: Name of the robot
                  tcp_pose: TCP pose [12] = 4x4 matrix (R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz)

              Returns:
                  tuple: (solutions, is_solvable) where solutions is list of joint angle arrays, is_solvable is bool
          )pbdoc");

    /**
     * Binding of solve_ik_with_config
     * @param[in] robot_name Name of the robot
     * @param[in] tcp_pose TCP pose as a 12-element array representing a 4x4 matrix
     * @param[in] shoulder_config Shoulder configuration (0=FRONT, 1=BACK)
     * @param[in] elbow_config Elbow configuration (2=UP, 3=DOWN)
     * @param[in] wrist_config Wrist configuration (4=N_FLIP, 5=FLIP)
     * @return Tuple of (solutions, is_solvable)
     */
    m.def("solve_ik_with_config",
          [](const std::string& robot_name,
             const py::array_t<IkReal>& tcp_pose,
             int shoulder_config,
             int elbow_config,
             int wrist_config) {

              if (tcp_pose.size() != 12) {
                  // Return empty result instead of throwing
                  return py::make_tuple(py::array_t<IkReal>(0), false);
              }

              int dof = ikcore::get_num_joints(robot_name);
              if (dof <= 0) {
                  // Robot not found - return empty result instead of throwing
                  return py::make_tuple(py::array_t<IkReal>(0), false);
              }

              std::vector<IkReal> joints(dof);
              bool is_solvable = false;

              bool ok = ikcore::solveIKWithConfig(
                  robot_name,
                  tcp_pose.data(),
                  shoulder_config,  // Now using int directly
                  elbow_config,     // Now using int directly
                  wrist_config,     // Now using int directly
                  joints.data(),
                  &is_solvable
              );

              if (!ok || !is_solvable) {
                  // Return empty array and False
                  return py::make_tuple(py::array_t<IkReal>(0), false);
              }

              // Return (joints, True)
              py::array_t<IkReal> joints_out(dof);
              std::memcpy(joints_out.mutable_data(), joints.data(), dof * sizeof(IkReal));
              return py::make_tuple(joints_out, true);
          },
          py::arg("robot_name"),
          py::arg("tcp_pose"),
          py::arg("shoulder_config"),
          py::arg("elbow_config"),
          py::arg("wrist_config"),
          R"pbdoc(
              Solve IK with Configuration - Returns single solution matching configuration

              Args:
                  robot_name: Name of the robot
                  tcp_pose: TCP pose [12] = 4x4 matrix (R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz)
                  shoulder_config: 0=FRONT, 1=BACK
                  elbow_config: 2=UP, 3=DOWN
                  wrist_config: 4=N_FLIP, 5=FLIP

              Returns:
                  tuple: (joints, is_solvable)
                      joints: Joint angles array [dof] if solvable, empty array otherwise
                      is_solvable: True if solution found, False otherwise
          )pbdoc");

    /**
     * Binding of solve_ik_with_config
     * @param[in] robot_name Name of the robot
     * @param[in] tcp_pose TCP pose as a 12-element array representing a 4x4 matrix
     * @param[in] shoulder_config Shoulder configuration (0=FRONT, 1=BACK)
     * @param[in] elbow_config Elbow configuration (2=UP, 3=DOWN)
     * @param[in] wrist_config Wrist configuration (4=N_FLIP, 5=FLIP)
     * @param[in] current_joints Current joint angles for nearest solutionb (nullptr if not used)
     * @return Tuple of (solutions, is_solvable)
     */
    m.def("solve_ik_with_config",
          [](const std::string& robot_name,
             const py::array_t<IkReal>& tcp_pose,
             int shoulder_config,
             int elbow_config,
             int wrist_config,
             const py::array_t<IkReal>& current_joints) {

              if (tcp_pose.size() != 12) {
                  return py::make_tuple(py::array_t<IkReal>(0), false);
              }

              int dof = ikcore::get_num_joints(robot_name);
              if (dof <= 0) {
                  return py::make_tuple(py::array_t<IkReal>(0), false);
              }

              if (current_joints.size() != static_cast<size_t>(dof)) {
                  return py::make_tuple(py::array_t<IkReal>(0), false);
              }

              std::vector<IkReal> joints(dof);
              bool is_solvable = false;

              bool ok = ikcore::solveIKWithConfig(
                  robot_name,
                  tcp_pose.data(),
                  shoulder_config,
                  elbow_config,
                  wrist_config,
                  joints.data(),
                  &is_solvable,
                  current_joints.data()  // Pass current_joints for continuity
              );

              if (!ok || !is_solvable) {
                  return py::make_tuple(py::array_t<IkReal>(0), false);
              }

              py::array_t<IkReal> joints_out(dof);
              std::memcpy(joints_out.mutable_data(), joints.data(), dof * sizeof(IkReal));
              return py::make_tuple(joints_out, true);
          },
          py::arg("robot_name"),
          py::arg("tcp_pose"),
          py::arg("shoulder_config"),
          py::arg("elbow_config"),
          py::arg("wrist_config"),
          py::arg("current_joints"),
          R"pbdoc(
              Solve IK with Configuration and Continuity

              Args:
                  robot_name: Name of the robot
                  tcp_pose: TCP pose [12]
                  shoulder_config: 0=FRONT, 1=BACK
                  elbow_config: 2=UP, 3=DOWN
                  wrist_config: 4=N_FLIP, 5=FLIP
                  current_joints: Current joint angles [dof] for continuity

              Returns:
                  tuple: (joints, is_solvable) - Nearest solution within config
          )pbdoc");

    /**
     * Binding of solve_ik_with_joint
     * @param[in] robot_name Name of the robot
     * @param[in] tcp_pose TCP pose as a 12-element array representing a 4x4 matrix
     * @param[in] current_joints Current joint angles [dof]
     * @return Tuple of (joints, is_solvable)
     */
    m.def("solve_ik_with_joint",
          [](const std::string& robot_name,
             const py::array_t<IkReal>& tcp_pose,
             const py::array_t<IkReal>& current_joints) {

              if (tcp_pose.size() != 12) {
                  // Return empty result instead of throwing
                  return py::make_tuple(py::array_t<IkReal>(0), false);
              }

              int dof = ikcore::get_num_joints(robot_name);
              if (dof <= 0) {
                  // Robot not found - return empty result instead of throwing
                  return py::make_tuple(py::array_t<IkReal>(0), false);
              }

              if (current_joints.size() != static_cast<size_t>(dof)) {
                  // Size mismatch - return empty result instead of throwing
                  return py::make_tuple(py::array_t<IkReal>(0), false);
              }

              std::vector<IkReal> joints(dof);
              bool is_solvable = false;

              bool ok = ikcore::solveIKWithJoint(
                  robot_name,
                  tcp_pose.data(),
                  current_joints.data(),
                  joints.data(),
                  &is_solvable
              );

              if (!ok || !is_solvable) {
                  // Return empty array and False
                  return py::make_tuple(py::array_t<IkReal>(0), false);
              }

              // Return (joints, True)
              py::array_t<IkReal> joints_out(dof);
              std::memcpy(joints_out.mutable_data(), joints.data(), dof * sizeof(IkReal));
              return py::make_tuple(joints_out, true);
          },
          py::arg("robot_name"),
          py::arg("tcp_pose"),
          py::arg("current_joints"),
          R"pbdoc(
              Solve IK with Joint - Returns single solution nearest to current_joints

              Args:
                  robot_name: Name of the robot
                  tcp_pose: TCP pose [12] = 4x4 matrix (R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz)
                  current_joints: Current joint angles [dof]

              Returns:
                  tuple: (joints, is_solvable)
                      joints: Joint angles array [dof] if solvable, empty array otherwise
                      is_solvable: True if solution found, False otherwise
          )pbdoc");

    /**
     * Binding of compute_fk
     * @param[in] robot_name Name of the robot
     * @param[in] joints Joint angles array [dof]
     * @return Tuple of (eetrans, eerot, success)
     */
    m.def("compute_fk",
          [](const std::string& robot_name,
             const py::array_t<IkReal>& joints) {
              int dof = ikcore::get_num_joints(robot_name);
              if (dof <= 0) {
                  // Robot not found - return empty result instead of throwing
                  return py::make_tuple(py::array_t<IkReal>(0), py::array_t<IkReal>(0), false);
              }

              if (joints.size() != static_cast<size_t>(dof)) {
                  // Size mismatch - return empty result instead of throwing
                  return py::make_tuple(py::array_t<IkReal>(0), py::array_t<IkReal>(0), false);
              }

              IkReal eetrans[3];
              IkReal eerot[9];

              bool ok = ikcore::computeFK(robot_name, joints.data(), eetrans, eerot);
              if (!ok) {
                  // FK computation failed - return empty result instead of throwing
                  return py::make_tuple(py::array_t<IkReal>(0), py::array_t<IkReal>(0), false);
              }

              // Return tuple: (eetrans, eerot, True)
              py::array_t<IkReal> trans_out(3);
              py::array_t<IkReal> rot_out(9);
              std::memcpy(trans_out.mutable_data(), eetrans, 3 * sizeof(IkReal));
              std::memcpy(rot_out.mutable_data(), eerot, 9 * sizeof(IkReal));

              return py::make_tuple(trans_out, rot_out, true);
          },
          py::arg("robot_name"),
          py::arg("joints"),
          R"pbdoc(
              Compute Forward Kinematics

              Args:
                  robot_name: Name of the robot
                  joints: Joint angles array [dof]

              Returns:
                  tuple: (eetrans, eerot, success)
                      eetrans: Position [3] (empty if failed)
                      eerot: Rotation matrix [9] row-major (empty if failed)
                      success: True if successful, False if failed (invalid inputs or joint limit violation)
          )pbdoc");
}
