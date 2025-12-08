using System;
using System.Runtime.InteropServices;
using System.IO;

namespace TestIKFast
{
    /// <summary>
    /// IKFastUnity DLL P/Invoke wrapper
    /// </summary>
    public static class IKFastUnity
    {
#if X86
        private const string DLL_NAME = "IKFastUnity_x86.dll";
#else
        private const string DLL_NAME = "IKFastUnity_x64.dll";
#endif

        // UTF-8 인코딩을 사용하여 한글 경로 지원
        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IKU_Init([MarshalAs(UnmanagedType.LPUTF8Str)] string robots_dir);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IKU_GetNumJoints([MarshalAs(UnmanagedType.LPUTF8Str)] string robot_name);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IKU_GetJointLimits(
            [MarshalAs(UnmanagedType.LPUTF8Str)] string robot_name,
            [Out] double[] out_lower,
            [Out] double[] out_upper,
            int max_joints);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IKU_SolveIK(
            [MarshalAs(UnmanagedType.LPUTF8Str)] string robot_name,
            [In] double[] tcp_pose,  // [12]: R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz
            [Out] double[] out_solutions,
            int max_solutions
        );

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IKU_SolveIKWithConfig(
            [MarshalAs(UnmanagedType.LPUTF8Str)] string robot_name,
            [In] double[] tcp_pose,  // [12]: R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz
            int shoulder_config,
            int elbow_config,
            int wrist_config,
            [Out] double[] out_joints,
            out int out_is_solvable
        );

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IKU_SolveIKWithJoint(
            [MarshalAs(UnmanagedType.LPUTF8Str)] string robot_name,
            [In] double[] tcp_pose,  // [12]: R11, R12, R13, Tx, R21, R22, R23, Ty, R31, R32, R33, Tz
            [In] double[] current_joints,
            [Out] double[] out_joints,
            out int out_is_solvable
        );

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl)]
        public static extern int IKU_ComputeFK(
            [MarshalAs(UnmanagedType.LPUTF8Str)] string robot_name,
            [In] double[] joints,
            [Out] double[] out_eetrans,
            [Out] double[] out_eerot
        );
    }

    public enum PoseConfig
    {
        NULL = -1,
        RIGHT = 0,
        LEFT = 1,
        UP = 2,
        DOWN = 3,
        N_FLIP = 4,
        FLIP = 5
    }

    // Removed MatrixHelper: examples use direct transformation array for simplicity

    /// <summary>
    /// Python-style wrapper for IKFast solver
    /// Provides the same input/output interface as Python ikfast_solver module
    /// </summary>
    public static class ikfast_solver
    {
        /// <summary>
        /// Solve IK and return all solutions (Python: solve_ik)
        /// </summary>
        /// <param name="robot_name">Robot name (case insensitive)</param>
        /// <param name="tcp_pose">TCP 4x4 transformation matrix (12 elements)</param>
        /// <returns>(solutions, is_solvable) where solutions is array of joint arrays</returns>
        public static (double[][] solutions, bool is_solvable) solve_ik(
            string robot_name,
            double[] tcp_pose)
        {
            int dof = IKFastUnity.IKU_GetNumJoints(robot_name);
            if (dof <= 0)
            {
                return (Array.Empty<double[]>(), false);
            }

            const int max_solutions = 32;  // Internal buffer size for all possible solutions
            double[] out_solutions = new double[max_solutions * dof];
            int num_solutions = IKFastUnity.IKU_SolveIK(robot_name, tcp_pose, out_solutions, max_solutions);

            if (num_solutions <= 0)
            {
                return (Array.Empty<double[]>(), false);
            }

            double[][] solutions = new double[num_solutions][];
            for (int i = 0; i < num_solutions; i++)
            {
                solutions[i] = new double[dof];
                Array.Copy(out_solutions, i * dof, solutions[i], 0, dof);
            }

            return (solutions, true);
        }

        /// <summary>
        /// Solve IK with specific configuration (Python: solve_ik_with_config)
        /// </summary>
        /// <param name="robot_name">Robot name (case insensitive)</param>
        /// <param name="tcp_pose">TCP 4x4 transformation matrix (12 elements)</param>
        /// <param name="shoulder_config">Shoulder configuration (0=RIGHT, 1=LEFT)</param>
        /// <param name="elbow_config">Elbow configuration (2=UP, 3=DOWN)</param>
        /// <param name="wrist_config">Wrist configuration (4=N_FLIP, 5=FLIP)</param>
        /// <returns>(joints, is_solvable)</returns>
        public static (double[] joints, bool is_solvable) solve_ik_with_config(
            string robot_name,
            double[] tcp_pose,
            int shoulder_config,
            int elbow_config,
            int wrist_config)
        {
            int dof = IKFastUnity.IKU_GetNumJoints(robot_name);
            if (dof <= 0)
            {
                return (Array.Empty<double>(), false);
            }

            double[] joints = new double[dof];
            int is_solvable_int;
            int result = IKFastUnity.IKU_SolveIKWithConfig(
                robot_name, tcp_pose,
                shoulder_config, elbow_config, wrist_config,
                joints, out is_solvable_int);

            if (result != 1 || is_solvable_int != 1)
            {
                return (Array.Empty<double>(), false);
            }

            return (joints, true);
        }

        /// <summary>
        /// Solve IK and return solution nearest to current joints (Python: solve_ik_with_joint)
        /// </summary>
        /// <param name="robot_name">Robot name (case insensitive)</param>
        /// <param name="tcp_pose">TCP 4x4 transformation matrix (12 elements)</param>
        /// <param name="current_joints">Current joint angles (radians)</param>
        /// <returns>(joints, is_solvable)</returns>
        public static (double[] joints, bool is_solvable) solve_ik_with_joint(
            string robot_name,
            double[] tcp_pose,
            double[] current_joints)
        {
            int dof = IKFastUnity.IKU_GetNumJoints(robot_name);
            if (dof <= 0)
            {
                return (Array.Empty<double>(), false);
            }

            double[] joints = new double[dof];
            int is_solvable_int;
            int result = IKFastUnity.IKU_SolveIKWithJoint(
                robot_name, tcp_pose, current_joints,
                joints, out is_solvable_int);

            if (result != 1 || is_solvable_int != 1)
            {
                return (Array.Empty<double>(), false);
            }

            return (joints, true);
        }

        /// <summary>
        /// Compute forward kinematics (Python: compute_fk)
        /// </summary>
        /// <param name="robot_name">Robot name (case insensitive)</param>
        /// <param name="joints">Joint angles (radians)</param>
        /// <returns>(translation, rotation) where translation is [x,y,z] and rotation is 3x3 matrix (row-major, 9 elements)</returns>
        public static (double[] translation, double[] rotation) compute_fk(
            string robot_name,
            double[] joints)
        {
            double[] trans = new double[3];
            double[] rot = new double[9];
            int result = IKFastUnity.IKU_ComputeFK(robot_name, joints, trans, rot);

            if (result != 1)
            {
                return (Array.Empty<double>(), Array.Empty<double>());
            }

            return (trans, rot);
        }
    }

    class Program
    {
        [DllImport("kernel32", SetLastError = true, CharSet = CharSet.Unicode)]
        private static extern IntPtr LoadLibrary(string lpFileName);

        private static void PreloadDependencies(string depsDir)
        {
            if (!Directory.Exists(depsDir)) return;
            foreach (var dll in Directory.EnumerateFiles(depsDir, "*.dll"))
            {
                LoadLibrary(dll);
            }
        }

        static void Main(string[] args)
        {
            string robotName = "mpx3500_c00x";

            Console.WriteLine("=".PadRight(60, '='));
            Console.WriteLine("IKFast C# Test - All IK Modes + FK");
            Console.WriteLine("=".PadRight(60, '='));
            Console.WriteLine();

            try
            {
                string exeDir = AppContext.BaseDirectory;
                string testsDir = Path.GetFullPath(Path.Combine(exeDir, "..", "..", "..", "..")); // -> tests
                string ikSolverDir = Path.GetFullPath(Path.Combine(testsDir, "..")); // -> ik-solver
                string robotsDir = Path.Combine(ikSolverDir, "src", "robots");
                string libDir = Path.Combine(ikSolverDir, "lib");
                string vcpkgRoot = Environment.GetEnvironmentVariable("VCPKG_ROOT") ?? @"C:\dev\vcpkg";
                string vcpkgBin = Path.Combine(vcpkgRoot, @"installed\x64-windows\bin");

                // Ensure bundled BLAS/LAPACK are loaded before conda/MKL
                bool isolatePath = Environment.GetEnvironmentVariable("IKFAST_ISOLATE_PATH") == "1";
                string currentPath = Environment.GetEnvironmentVariable("PATH") ?? string.Empty;
                string newPath = "";
                if (Directory.Exists(vcpkgBin)) newPath += vcpkgBin + ";";
                if (Directory.Exists(libDir)) newPath += libDir + ";";
                if (Directory.Exists(robotsDir)) newPath += robotsDir + ";";

                if (isolatePath)
                {
                    // Minimal system paths to avoid loading MKL from conda
                    string sysRoot = Environment.GetFolderPath(Environment.SpecialFolder.Windows);
                    string system32 = Path.Combine(sysRoot, "System32");
                    newPath += system32 + ";" + sysRoot + ";";
                }
                else
                {
                    newPath += currentPath;
                }

                Environment.SetEnvironmentVariable("PATH", newPath);

                // Preload dependency DLLs explicitly
                PreloadDependencies(vcpkgBin);
                PreloadDependencies(libDir);

                Console.WriteLine($"Loading IK plugins from: {robotsDir}");

                if (!Directory.Exists(robotsDir))
                {
                    Console.WriteLine($"ERROR: Robots directory not found: {robotsDir}");
                    return;
                }

                if (IKFastUnity.IKU_Init(robotsDir) == 0)
                {
                    Console.WriteLine("ERROR: Failed to initialize IKFast (no plugins loaded)");
                    Console.WriteLine("Ensure dependencies are available (openblas, lapack, libgfortran, etc.)");
                    return;
                }
                Console.WriteLine("✓ IK plugins loaded successfully");
                Console.WriteLine();

                RunAllTests(robotName);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"ERROR: {ex.Message}");
                Console.WriteLine($"Stack trace: {ex.StackTrace}");
            }
        }

        static void RunAllTests(string robotName)
        {
            Console.WriteLine("=".PadRight(60, '='));
            Console.WriteLine($"Testing Robot: {robotName.ToUpper()}");
            Console.WriteLine("=".PadRight(60, '='));

            int numJoints = IKFastUnity.IKU_GetNumJoints(robotName);
            if (numJoints <= 0)
            {
                Console.WriteLine($"✗ Robot '{robotName}' not loaded or not available");
                return;
            }

            Console.WriteLine($"DOF: {numJoints}");
            Console.WriteLine();

            // Target pose in Euler angles (for display)
            double x = 0.5, y = 0.0, z = 0.3;
            double rx = 0.0, ry = 0.0, rz = 0.0;

            Console.WriteLine("Target TCP Pose:");
            Console.WriteLine($"  Position: ({x:F3}, {y:F3}, {z:F3}) m");
            Console.WriteLine($"  Orientation: ({rx:F3}, {ry:F3}, {rz:F3}) rad (Euler ZYX)");
            Console.WriteLine();

            // Build 4x4 transformation (12 elements) with identity rotation
            double[] tcpPose = new double[]
            {
                var limits = LoadRobotJointLimits(robot, robotsDir);
                if (limits == null)
                {
                    Console.WriteLine($"{robot,-20} | SKIP (no limits or DOF)");
                    continue;
                }

                var result = TestRobot(robot, limits);
                if (result == null) continue;

                bool allOk = result.Values.All(v => v.Contains("OK"));
                
                string statusLine = $"{robot,-20} | ";
                foreach (var kvp in result.OrderBy(k => k.Key))
                {
                    statusLine += $"{kvp.Key,-25}={kvp.Value,-30}";
                }
                Console.WriteLine(statusLine);
                if (allOk) passed++;
            }

            Console.WriteLine("--------------------------------------------------------------------------------");
            Console.WriteLine($"Result: {passed}/{robots.Count} robots passed all tests");
            Console.WriteLine("================================================================================");
        }

        static List<string> DiscoverRobots(string robotsDir)
        {
            var robots = new HashSet<string>();
            foreach (var file in Directory.GetFiles(robotsDir, "*_ikfast.dll", SearchOption.AllDirectories))
            {
                string robotName = Path.GetFileNameWithoutExtension(file).Replace("_ikfast", "");
                robots.Add(robotName);
            }
            return robots.OrderBy(r => r).ToList();
        }

        static Dictionary<string, double[]> LoadRobotJointLimits(string robotName, string robotsDir)
        {
            int dof = IKFastUnity.IKU_GetNumJoints(robotName);
            if (dof > 0)
            {
                double[] lowers = new double[dof];
                double[] uppers = new double[dof];
                int count = IKFastUnity.IKU_GetJointLimits(robotName, lowers, uppers, dof);

                if (count == dof)
                {
                    // Use compiled limits to sample random joints within bounds
                    double[] sampled = new double[dof];
                    Random rand = new Random();
                    for (int i = 0; i < dof; i++)
                    {
                        double lo = lowers[i];
                        double hi = uppers[i];
                        if (hi < lo) (lo, hi) = (hi, lo);
                        sampled[i] = lo + ((hi - lo) * rand.NextDouble());
                    }
                    return new Dictionary<string, double[]> { { "joints", sampled } };
                }
            }

            // Find joint_limits.json in the robot's directory
            // Path format: robots_dir/<manufacturer>/<robot_name>/joint_limits.json
            // or: robots_dir/<robot_name>/joint_limits.json
            
            foreach (var file in Directory.GetFiles(robotsDir, "joint_limits.json", SearchOption.AllDirectories))
            {
                try
                {
                    // Check if this file is for the target robot by checking the directory structure
                    string parentDir = Path.GetFileName(Path.GetDirectoryName(file));
                    
                    // Normalize robot name for comparison (gp25_12, gp50, etc.)
                    string normalizedRobotName = robotName.ToLower().Replace("-", "_");
                    string normalizedParentDir = parentDir.ToLower().Replace("-", "_");
                    
                    if (normalizedParentDir.Contains(normalizedRobotName))
                    {
                        // Found the correct file, now load joint limits
                        string json = File.ReadAllText(file);
                        
                        // Parse JSON array format: [{"min": ..., "max": ...}, ...]
                        // Remove brackets and parse individually
                        var matches = System.Text.RegularExpressions.Regex.Matches(json, @"\{[^}]+\}");
                        
                        if (matches.Count > 0)
                        {
                            int jsonDof = matches.Count;
                            double[] result = new double[jsonDof];
                            Random rand = new Random();
                            
                            for (int i = 0; i < jsonDof; i++)
                            {
                                try
                                {
                                    var limitObj = matches[i].Value;
                                    
                                    // Extract min/max values
                                    double minVal = -Math.PI;
                                    double maxVal = Math.PI;
                                    
                                    // Try to parse "min" and "max" fields
                                    var minMatch = System.Text.RegularExpressions.Regex.Match(limitObj, @"""(?:min|lower)""\s*:\s*([-\d.]+)");
                                    var maxMatch = System.Text.RegularExpressions.Regex.Match(limitObj, @"""(?:max|upper)""\s*:\s*([-\d.]+)");
                                    
                                    if (minMatch.Success && double.TryParse(minMatch.Groups[1].Value, out var min))
                                        minVal = min;
                                    if (maxMatch.Success && double.TryParse(maxMatch.Groups[1].Value, out var max))
                                        maxVal = max;
                                    
                                    result[i] = minVal + ((maxVal - minVal) * rand.NextDouble());
                                }
                                catch
                                {
                                    result[i] = -Math.PI + (Math.PI * 2 * rand.NextDouble());
                                }
                            }
                            
                            return new Dictionary<string, double[]> { { "joints", result } };
                        }
                    }
                }
                catch { }
            }
            
            return null;
        }

        static Dictionary<string, string> TestRobot(string robotName, Dictionary<string, double[]> limitsDict)
        {
            int dof = IKFastUnity.IKU_GetNumJoints(robotName);
            if (dof <= 0) return null;

            double[] origJoints = limitsDict["joints"];
            if (origJoints.Length != dof) return null;

            var results = new Dictionary<string, string>();

            // FK: Get original TCP pose from original joints
            double[] tcpPoseOrig = new double[12];
            double[] trans = new double[3];
            double[] rot = new double[9];
            IKFastUnity.IKU_ComputeFK(robotName, origJoints, trans, rot);
            Array.Copy(new double[] {
                rot[0], rot[1], rot[2], trans[0],
                rot[3], rot[4], rot[5], trans[1],
                rot[6], rot[7], rot[8], trans[2]
            }, tcpPoseOrig, 12);

            // Helper function: Compare TCP poses by doing FK on IK result
            Func<double[], double?> CompareTCPViaFK = (ikJoints) =>
            {
                try
                {
                    double[] trans2 = new double[3];
                    double[] rot2 = new double[9];
                    IKFastUnity.IKU_ComputeFK(robotName, ikJoints, trans2, rot2);
                    
                    double[] tcpPoseIK = new double[12];
                    Array.Copy(new double[] {
                        rot2[0], rot2[1], rot2[2], trans2[0],
                        rot2[3], rot2[4], rot2[5], trans2[1],
                        rot2[6], rot2[7], rot2[8], trans2[2]
                    }, tcpPoseIK, 12);
                    
                    // Compare TCP poses (all 12 elements)
                    double tcpErr = 0;
                    for (int i = 0; i < 12; i++)
                        tcpErr += Math.Pow(tcpPoseIK[i] - tcpPoseOrig[i], 2);
                    return Math.Sqrt(tcpErr);
                }
                catch
                {
                    return null;
                }
            };

            // 1) ikfast_solver.solve_ik (all solutions) - Python style
            Console.WriteLine("--- ikfast_solver.solve_ik (all solutions) ---");
            var (solutions, is_solvable) = ikfast_solver.solve_ik(robotName, tcpPose);
            Console.WriteLine($"Found {solutions.Length} solution(s), is_solvable: {is_solvable}");
            for (int i = 0; i < solutions.Length; i++)
            {
                Console.Write($"  Solution {i + 1}: ");
                for (int j = 0; j < numJoints; j++)
                {
                    double rad = solutions[i][j];
                    Console.Write($"{rad * 180.0 / Math.PI,8:F2}° ");
                }
                Console.WriteLine();
            }
            if (is_solvable && solutions.Length > 0)
            {
                var (fkTrans, fkRot) = ikfast_solver.compute_fk(robotName, solutions[0]);
                if (fkTrans.Length > 0)
                {
                    // Extract target position from matrix: tcpPose[3], tcpPose[7], tcpPose[11]
                    double posErr = Math.Sqrt(
                        Math.Pow(tcpPose[3] - fkTrans[0], 2) +
                        Math.Pow(tcpPose[7] - fkTrans[1], 2) +
                        Math.Pow(tcpPose[11] - fkTrans[2], 2)
                    );
                    Console.WriteLine($"  FK (sol1): pos=({fkTrans[0]:F6}, {fkTrans[1]:F6}, {fkTrans[2]:F6}), err={posErr:E3} m");
                }
            }
            Console.WriteLine();

            // 2) ikfast_solver.solve_ik_with_config - Python style
            var configs = new[]
            {
                (PoseConfig.RIGHT, PoseConfig.DOWN, PoseConfig.N_FLIP, "Right-Down-NoFlip"),
                (PoseConfig.LEFT,  PoseConfig.DOWN, PoseConfig.N_FLIP, "Left-Down-NoFlip"),
                (PoseConfig.RIGHT, PoseConfig.UP,   PoseConfig.N_FLIP, "Right-Up-NoFlip"),
                (PoseConfig.LEFT,  PoseConfig.UP,   PoseConfig.FLIP,   "Left-Up-Flip")
            };

            foreach (var (shoulder, elbow, wrist, configName) in configs)
            {
                Console.WriteLine($"Configuration: {configName}");
                var (joints, config_is_solvable) = ikfast_solver.solve_ik_with_config(
                    robotName, tcpPose,
                    (int)shoulder, (int)elbow, (int)wrist
                );

                if (config_is_solvable)
                {
                    Console.WriteLine("  ✓ Solution found:");
                    for (int j = 0; j < numJoints; j++)
                    {
                        double deg = joints[j] * 180.0 / Math.PI;
                        Console.WriteLine($"    J{j + 1}: {deg,8:F2}° ({joints[j],8:F4} rad)");
                    }

                    var (fkTrans, fkRot) = ikfast_solver.compute_fk(robotName, joints);
                    if (fkTrans.Length > 0)
                    {
                        // Extract target position from matrix: tcpPose[3], tcpPose[7], tcpPose[11]
                        double posErr = Math.Sqrt(
                            Math.Pow(tcpPose[3] - fkTrans[0], 2) +
                            Math.Pow(tcpPose[7] - fkTrans[1], 2) +
                            Math.Pow(tcpPose[11] - fkTrans[2], 2)
                        );

                        Console.WriteLine($"    FK: pos=({fkTrans[0]:F6}, {fkTrans[1]:F6}, {fkTrans[2]:F6}), err={posErr:E3} m");
                    }
                }
                else
                {
                    Console.WriteLine("  ✗ No solution found for this configuration");
                }

                Console.WriteLine();
            }

            // 3) ikfast_solver.solve_ik_with_joint (nearest to current) - Python style
            Console.WriteLine("--- ikfast_solver.solve_ik_with_joint (nearest to current) ---");
            double[] currentJoints = new double[numJoints];
            var (nearest, nearest_is_solvable) = ikfast_solver.solve_ik_with_joint(
                robotName, tcpPose, currentJoints
            );
            if (nearest_is_solvable)
            {
                Console.WriteLine("  ✓ Solution (nearest):");
                for (int i = 0; i < numJoints; i++)
                {
                    double deg = nearest[i] * 180.0 / Math.PI;
                    Console.WriteLine($"    J{i + 1}: {deg,8:F2}° ({nearest[i],8:F4} rad)");
                }

                var (fkTrans, fkRot) = ikfast_solver.compute_fk(robotName, nearest);
                if (fkTrans.Length > 0)
                {
                    // Extract target position from matrix: tcpPose[3], tcpPose[7], tcpPose[11]
                    double posErr = Math.Sqrt(
                        Math.Pow(tcpPose[3] - fkTrans[0], 2) +
                        Math.Pow(tcpPose[7] - fkTrans[1], 2) +
                        Math.Pow(tcpPose[11] - fkTrans[2], 2)
                    );
                    Console.WriteLine($"    FK: pos=({fkTrans[0]:F6}, {fkTrans[1]:F6}, {fkTrans[2]:F6}), err={posErr:E3} m");
                }
            }
            else
            {
                Console.WriteLine("  ✗ No solution found (nearest)");
            }
            Console.WriteLine();

            Console.WriteLine("=".PadRight(60, '='));
            Console.WriteLine("All IK tests completed!");
            Console.WriteLine("=".PadRight(60, '='));
        }
    }
}
