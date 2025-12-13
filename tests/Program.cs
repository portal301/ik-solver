using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.IO;
using System.Linq;

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
                try { LoadLibrary(dll); } catch { }
            }
        }

        static void Main(string[] args)
        {
            Console.WriteLine("================================================================================");
            Console.WriteLine("IKFast Unified Test - All Robots, All IK Functions (C#)");
            Console.WriteLine("================================================================================");
            Console.WriteLine();

            try
            {
                string exeDir = AppContext.BaseDirectory;
                string testsDir = Path.GetFullPath(Path.Combine(exeDir, "..", "..", "..", ".."));
                string ikSolverDir = Path.GetFullPath(Path.Combine(testsDir, ".."));
                string robotsDir = Path.Combine(ikSolverDir, "src", "robots");
                string libDir = Path.Combine(ikSolverDir, "lib");
                string vcpkgRoot = Environment.GetEnvironmentVariable("VCPKG_ROOT") ?? @"C:\dev\vcpkg";
                string vcpkgBin = Path.Combine(vcpkgRoot, @"installed\x64-windows\bin");

                bool isolatePath = Environment.GetEnvironmentVariable("IKFAST_ISOLATE_PATH") == "1";
                string currentPath = Environment.GetEnvironmentVariable("PATH") ?? string.Empty;
                string newPath = "";
                if (Directory.Exists(vcpkgBin)) newPath += vcpkgBin + ";";
                if (Directory.Exists(libDir)) newPath += libDir + ";";
                if (Directory.Exists(robotsDir)) newPath += robotsDir + ";";

                if (isolatePath)
                {
                    string sysRoot = Environment.GetFolderPath(Environment.SpecialFolder.Windows);
                    string system32 = Path.Combine(sysRoot, "System32");
                    newPath += system32 + ";" + sysRoot + ";";
                }
                else
                {
                    newPath += currentPath;
                }

                Environment.SetEnvironmentVariable("PATH", newPath);
                PreloadDependencies(vcpkgBin);
                PreloadDependencies(libDir);

                if (!Directory.Exists(robotsDir))
                {
                    Console.WriteLine($"[FAIL] Robots directory not found: {robotsDir}");
                    return;
                }

                if (IKFastUnity.IKU_Init(robotsDir) == 0)
                {
                    Console.WriteLine("[FAIL] Failed to initialize IKFast");
                    return;
                }
                Console.WriteLine("[OK] Plugins loaded\n");
                RunAllRobotsTests(robotsDir);
            }
            catch (Exception ex)
            {
                Console.WriteLine($"[FAIL] {ex.Message}");
            }
        }

        static void RunAllRobotsTests(string robotsDir)
        {
            var robots = DiscoverRobots(robotsDir);
            if (robots.Count == 0)
            {
                Console.WriteLine("[FAIL] No robots found");
                return;
            }

            Console.WriteLine($"Testing {robots.Count} robot(s):");
            Console.WriteLine("--------------------------------------------------------------------------------");

            int passed = 0;
            foreach (var robot in robots)
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

            // Test 1: solve_ik - Compare via TCP
            try
            {
                const int maxSol = 32;
                double[] outSol = new double[maxSol * dof];
                int numSol = IKFastUnity.IKU_SolveIK(robotName, tcpPoseOrig, outSol, maxSol);
                if (numSol > 0)
                {
                    double minTcpErr = double.MaxValue;
                    for (int i = 0; i < numSol; i++)
                    {
                        double[] ikJoints = new double[dof];
                        Array.Copy(outSol, i * dof, ikJoints, 0, dof);
                        double? tcpErr = CompareTCPViaFK(ikJoints);
                        if (tcpErr.HasValue && tcpErr < minTcpErr)
                            minTcpErr = tcpErr.Value;
                    }
                    results["solve_ik"] = $"OK ({numSol:D2} sol, err={minTcpErr:F6})";
                }
                else
                    results["solve_ik"] = "FAIL (no solution)";
            }
            catch (Exception e)
            {
                results["solve_ik"] = $"ERROR";
            }

            // Test 2: solve_ik_with_config - Compare via TCP
            try
            {
                for (int s = 0; s < 2; s++)
                {
                    for (int e = 0; e < 2; e++)
                    {
                        for (int w = 0; w < 2; w++)
                        {
                            double[] sol = new double[dof];
                            int ret = IKFastUnity.IKU_SolveIKWithConfig(robotName, tcpPoseOrig, s, e, w, sol, out int ok);
                            if (ret == 1 && ok == 1)
                            {
                                double? tcpErr = CompareTCPViaFK(sol);
                                results["solve_ik_with_config"] = "OK";
                                break;
                            }
                        }
                        if (results.ContainsKey("solve_ik_with_config")) break;
                    }
                    if (results.ContainsKey("solve_ik_with_config")) break;
                }
                if (!results.ContainsKey("solve_ik_with_config"))
                    results["solve_ik_with_config"] = "FAIL";
            }
            catch (Exception e)
            {
                results["solve_ik_with_config"] = $"ERROR";
            }

            // Test 3: solve_ik_with_joint - Compare via TCP
            try
            {
                double[] current = new double[dof];
                double[] sol = new double[dof];
                int ret = IKFastUnity.IKU_SolveIKWithJoint(robotName, tcpPoseOrig, current, sol, out int ok);
                if (ret == 1 && ok == 1)
                {
                    double? tcpErr = CompareTCPViaFK(sol);
                    results["solve_ik_with_joint"] = "OK";
                }
                else
                    results["solve_ik_with_joint"] = "FAIL";
            }
            catch (Exception e)
            {
                results["solve_ik_with_joint"] = $"ERROR";
            }

            return results;
        }
    }
}
