/*
 * IKFast Solver Unity Wrapper
 *
 * Unity 프로젝트에서 IKFast를 쉽게 사용할 수 있도록 P/Invoke 호출을 래핑한 클래스입니다.
 * 
 * 사용 방법:
 *   1. IKFastUnity_x64.dll과 robots/ 폴더를 Assets/Plugins/x86_64/에 복사
 *   2. 이 스크립트를 Assets/Scripts/에 복사
 *   3. IKFastSolver.Initialize() 호출하여 초기화
 *   4. IKFastSolver.SolveIK() 등의 메서드 사용
 *
 * 예제:
 *   if (!IKFastSolver.IsInitialized)
 *   {
 *       IKFastSolver.Initialize("Assets/Plugins/x86_64/robots");
 *   }
 *   
 *   Matrix4x4 targetPose = Matrix4x4.TRS(
 *       new Vector3(0.5f, 0f, 0.3f),
 *       Quaternion.identity,
 *       Vector3.one
 *   );
 *   
 *   var solutions = IKFastSolver.SolveIK("gp25", targetPose);
 *   if (solutions.Length > 0)
 *   {
 *       Debug.Log($"Found {solutions.Length} solutions");
 *   }
 */

using System;
using System.IO;
using System.Runtime.InteropServices;
using UnityEngine;

namespace IKFast
{
    /// <summary>
    /// IKFast 로봇 구성 (Shoulder, Elbow, Wrist)
    /// </summary>
    public enum RobotConfig
    {
        // Shoulder
        RIGHT = 0,
        LEFT = 1,

        // Elbow
        UP = 2,
        DOWN = 3,

        // Wrist
        N_FLIP = 4,
        FLIP = 5
    }

    /// <summary>
    /// IKFast DLL P/Invoke 네이티브 호출
    /// </summary>
    internal static class IKFastNative
    {
        private const string DLL_NAME = "IKFastUnity_x64";

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int IKU_Init([MarshalAs(UnmanagedType.LPStr)] string robots_dir);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int IKU_GetNumJoints([MarshalAs(UnmanagedType.LPStr)] string robot_name);

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int IKU_GetJointLimits(
            [MarshalAs(UnmanagedType.LPStr)] string robot_name,
            double[] out_lower,
            double[] out_upper,
            int max_joints
        );

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int IKU_SolveIK(
            [MarshalAs(UnmanagedType.LPStr)] string robot_name,
            double[] tcp_pose,
            double[] out_solutions,
            int max_solutions
        );

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int IKU_SolveIKWithConfig(
            [MarshalAs(UnmanagedType.LPStr)] string robot_name,
            double[] tcp_pose,
            int shoulder_config,
            int elbow_config,
            int wrist_config,
            double[] out_joints,
            out int is_solvable
        );

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int IKU_SolveIKWithJoint(
            [MarshalAs(UnmanagedType.LPStr)] string robot_name,
            double[] tcp_pose,
            double[] current_joints,
            double[] out_joints,
            out int is_solvable
        );

        [DllImport(DLL_NAME, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern int IKU_ComputeFK(
            [MarshalAs(UnmanagedType.LPStr)] string robot_name,
            double[] joints,
            double[] out_eetrans,
            double[] out_eerot
        );
    }

    /// <summary>
    /// 관절 제한 정보
    /// </summary>
    public struct JointLimits
    {
        public double[] Lower;
        public double[] Upper;

        public JointLimits(double[] lower, double[] upper)
        {
            Lower = lower;
            Upper = upper;
        }
    }

    /// <summary>
    /// IKFast Solver Unity 래퍼
    /// </summary>
    public static class IKFastSolver
    {
        private static bool _isInitialized = false;
        private const int MAX_SOLUTIONS = 48;

        /// <summary>
        /// 초기화 여부
        /// </summary>
        public static bool IsInitialized => _isInitialized;

        /// <summary>
        /// IKFast 플러그인 초기화
        /// </summary>
        /// <param name="robotsDir">로봇 플러그인 DLL 디렉토리 경로 (비우면 프로젝트에서 재귀 검색)</param>
        /// <returns>초기화 성공 여부</returns>
        public static bool Initialize(string robotsDir = null)
        {
            if (_isInitialized)
            {
                Debug.LogWarning("IKFast already initialized");
                return true;
            }

            // robotsDir 미지정 시 프로젝트 내에서 재귀적으로 검색
            string resolvedDir = ResolveRobotsDirectory(robotsDir);
            if (string.IsNullOrEmpty(resolvedDir))
            {
                Debug.LogError("Failed to locate robots directory. Specify robotsDir or place a 'robots' folder under Assets.");
                return false;
            }

            try
            {
                int result = IKFastNative.IKU_Init(resolvedDir);
                _isInitialized = (result != 0);

                if (_isInitialized)
                {
                    Debug.Log($"IKFast initialized successfully from: {resolvedDir}");
                }
                else
                {
                    Debug.LogError($"Failed to initialize IKFast from: {resolvedDir}");
                }

                return _isInitialized;
            }
            catch (Exception e)
            {
                Debug.LogError($"IKFast initialization error: {e.Message}");
                return false;
            }
        }

        /// <summary>
        /// robots 디렉토리를 재귀적으로 찾습니다.
        /// 우선순위: 사용자 지정 경로 > Assets/Plugins/x86_64 > Assets/Plugins > Assets > 프로젝트 루트
        /// </summary>
        private static string ResolveRobotsDirectory(string robotsDir)
        {
            // 1) 명시적 경로가 유효하면 그대로 사용
            if (!string.IsNullOrEmpty(robotsDir) && Directory.Exists(robotsDir))
            {
                return Path.GetFullPath(robotsDir);
            }

            // 2) 검색 루트 목록 구성
            string assetsPath = Application.dataPath; // .../Project/Assets
            string pluginsPath = Path.Combine(assetsPath, "Plugins");
            string x86Path = Path.Combine(pluginsPath, "x86_64");
            string projectRoot = Path.GetFullPath(Path.Combine(assetsPath, ".."));

            string[] searchRoots = new[]
            {
                x86Path,
                pluginsPath,
                assetsPath,
                projectRoot
            };

            foreach (string root in searchRoots)
            {
                if (string.IsNullOrEmpty(root) || !Directory.Exists(root))
                    continue;

                try
                {
                    foreach (string dir in Directory.EnumerateDirectories(root, "robots", SearchOption.AllDirectories))
                    {
                        // robots 폴더 안에 *_ikfast.dll 이나 liblapack.dll 등이 존재하면 유효하다고 간주
                        bool hasIkfastDll = Directory.EnumerateFiles(dir, "*_ikfast.dll", SearchOption.AllDirectories).GetEnumerator().MoveNext();
                        bool hasLapack = File.Exists(Path.Combine(dir, "liblapack.dll"));
                        if (hasIkfastDll || hasLapack)
                        {
                            return Path.GetFullPath(dir);
                        }
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogWarning($"Robots directory search skipped at {root}: {ex.Message}");
                }
            }

            // 찾지 못한 경우
            return null;
        }

        /// <summary>
        /// 로봇의 자유도(DOF) 조회
        /// </summary>
        /// <param name="robotName">로봇 이름 (예: "gp25")</param>
        /// <returns>자유도 (실패 시 -1)</returns>
        public static int GetNumJoints(string robotName)
        {
            if (!_isInitialized)
            {
                Debug.LogError("IKFast not initialized. Call Initialize() first.");
                return -1;
            }

            return IKFastNative.IKU_GetNumJoints(robotName);
        }

        /// <summary>
        /// 로봇의 관절 제한 조회
        /// </summary>
        /// <param name="robotName">로봇 이름 (예: "gp25")</param>
        /// <returns>관절 제한 정보 (실패 시 null)</returns>
        public static JointLimits? GetJointLimits(string robotName)
        {
            if (!_isInitialized)
            {
                Debug.LogError("IKFast not initialized. Call Initialize() first.");
                return null;
            }

            int dof = GetNumJoints(robotName);
            if (dof <= 0) return null;

            double[] lower = new double[dof];
            double[] upper = new double[dof];

            int count = IKFastNative.IKU_GetJointLimits(robotName, lower, upper, dof);
            if (count <= 0) return null;

            return new JointLimits(lower, upper);
        }

        /// <summary>
        /// IK 계산 - 모든 솔루션 반환
        /// </summary>
        /// <param name="robotName">로봇 이름 (예: "gp25")</param>
        /// <param name="targetPose">목표 TCP 자세 (Unity Matrix4x4)</param>
        /// <returns>IK 솔루션 배열 (각 솔루션은 라디안 단위 관절 각도 배열)</returns>
        public static double[][] SolveIK(string robotName, Matrix4x4 targetPose)
        {
            if (!_isInitialized)
            {
                Debug.LogError("IKFast not initialized. Call Initialize() first.");
                return new double[0][];
            }

            int dof = GetNumJoints(robotName);
            if (dof <= 0)
            {
                Debug.LogError($"Invalid robot name: {robotName}");
                return new double[0][];
            }

            // Unity Matrix4x4 -> IKFast TCP pose (row-major, 처음 3행)
            double[] tcpPose = MatrixToTcpPose(targetPose);

            // IK 계산
            double[] solutions = new double[dof * MAX_SOLUTIONS];
            int numSolutions = IKFastNative.IKU_SolveIK(robotName, tcpPose, solutions, MAX_SOLUTIONS);

            if (numSolutions <= 0)
            {
                return new double[0][];
            }

            // 결과 파싱
            double[][] result = new double[numSolutions][];
            for (int i = 0; i < numSolutions; i++)
            {
                result[i] = new double[dof];
                Array.Copy(solutions, i * dof, result[i], 0, dof);
            }

            return result;
        }

        /// <summary>
        /// IK 계산 - 특정 구성(Configuration)의 솔루션 반환
        /// </summary>
        /// <param name="robotName">로봇 이름 (예: "gp25")</param>
        /// <param name="targetPose">목표 TCP 자세 (Unity Matrix4x4)</param>
        /// <param name="shoulderConfig">어깨 구성 (RIGHT 또는 LEFT)</param>
        /// <param name="elbowConfig">팔꿈치 구성 (UP 또는 DOWN)</param>
        /// <param name="wristConfig">손목 구성 (N_FLIP 또는 FLIP)</param>
        /// <param name="joints">출력 관절 각도 (라디안)</param>
        /// <returns>솔루션 발견 여부</returns>
        public static bool SolveIKWithConfig(
            string robotName,
            Matrix4x4 targetPose,
            RobotConfig shoulderConfig,
            RobotConfig elbowConfig,
            RobotConfig wristConfig,
            out double[] joints)
        {
            joints = null;

            if (!_isInitialized)
            {
                Debug.LogError("IKFast not initialized. Call Initialize() first.");
                return false;
            }

            int dof = GetNumJoints(robotName);
            if (dof <= 0)
            {
                Debug.LogError($"Invalid robot name: {robotName}");
                return false;
            }

            double[] tcpPose = MatrixToTcpPose(targetPose);
            joints = new double[dof];
            int isSolvable;

            IKFastNative.IKU_SolveIKWithConfig(
                robotName, tcpPose,
                (int)shoulderConfig,
                (int)elbowConfig,
                (int)wristConfig,
                joints,
                out isSolvable
            );

            return isSolvable != 0;
        }

        /// <summary>
        /// IK 계산 - 현재 관절 각도에서 가장 가까운 솔루션 반환
        /// </summary>
        /// <param name="robotName">로봇 이름 (예: "gp25")</param>
        /// <param name="targetPose">목표 TCP 자세 (Unity Matrix4x4)</param>
        /// <param name="currentJoints">현재 관절 각도 (라디안)</param>
        /// <param name="nearestJoints">출력 관절 각도 (라디안)</param>
        /// <returns>솔루션 발견 여부</returns>
        public static bool SolveIKNearest(
            string robotName,
            Matrix4x4 targetPose,
            double[] currentJoints,
            out double[] nearestJoints)
        {
            nearestJoints = null;

            if (!_isInitialized)
            {
                Debug.LogError("IKFast not initialized. Call Initialize() first.");
                return false;
            }

            int dof = GetNumJoints(robotName);
            if (dof <= 0 || currentJoints == null || currentJoints.Length != dof)
            {
                Debug.LogError($"Invalid robot name or joint count: {robotName}");
                return false;
            }

            double[] tcpPose = MatrixToTcpPose(targetPose);
            nearestJoints = new double[dof];
            int isSolvable;

            IKFastNative.IKU_SolveIKWithJoint(
                robotName, tcpPose, currentJoints,
                nearestJoints,
                out isSolvable
            );

            return isSolvable != 0;
        }

        /// <summary>
        /// FK 계산 - 관절 각도에서 TCP 자세 계산
        /// </summary>
        /// <param name="robotName">로봇 이름 (예: "gp25")</param>
        /// <param name="joints">관절 각도 (라디안)</param>
        /// <returns>TCP 자세 (Unity Matrix4x4), 실패 시 null</returns>
        public static Matrix4x4? ComputeFK(string robotName, double[] joints)
        {
            if (!_isInitialized)
            {
                Debug.LogError("IKFast not initialized. Call Initialize() first.");
                return null;
            }

            int dof = GetNumJoints(robotName);
            if (dof <= 0 || joints == null || joints.Length != dof)
            {
                Debug.LogError($"Invalid robot name or joint count: {robotName}");
                return null;
            }

            double[] eePos = new double[3];
            double[] eeRot = new double[9];

            int result = IKFastNative.IKU_ComputeFK(robotName, joints, eePos, eeRot);
            if (result == 0)
            {
                return null;
            }

            return TcpPoseToMatrix(eePos, eeRot);
        }

        /// <summary>
        /// Unity Matrix4x4를 IKFast TCP pose 배열로 변환
        /// </summary>
        private static double[] MatrixToTcpPose(Matrix4x4 matrix)
        {
            // Unity는 column-major, IKFast는 row-major
            // Unity Matrix4x4: m00=R11, m01=R12, m02=R13, m03=Tx
            //                  m10=R21, m11=R22, m12=R23, m13=Ty
            //                  m20=R31, m21=R32, m22=R33, m23=Tz
            return new double[]
            {
                matrix.m00, matrix.m01, matrix.m02, matrix.m03,  // R11, R12, R13, Tx
                matrix.m10, matrix.m11, matrix.m12, matrix.m13,  // R21, R22, R23, Ty
                matrix.m20, matrix.m21, matrix.m22, matrix.m23   // R31, R32, R33, Tz
            };
        }

        /// <summary>
        /// IKFast TCP pose를 Unity Matrix4x4로 변환
        /// </summary>
        private static Matrix4x4 TcpPoseToMatrix(double[] position, double[] rotation)
        {
            Matrix4x4 matrix = new Matrix4x4();

            // Rotation (row-major)
            matrix.m00 = (float)rotation[0]; matrix.m01 = (float)rotation[1]; matrix.m02 = (float)rotation[2];
            matrix.m10 = (float)rotation[3]; matrix.m11 = (float)rotation[4]; matrix.m12 = (float)rotation[5];
            matrix.m20 = (float)rotation[6]; matrix.m21 = (float)rotation[7]; matrix.m22 = (float)rotation[8];

            // Translation
            matrix.m03 = (float)position[0];
            matrix.m13 = (float)position[1];
            matrix.m23 = (float)position[2];

            // Homogeneous coordinate
            matrix.m30 = 0; matrix.m31 = 0; matrix.m32 = 0; matrix.m33 = 1;

            return matrix;
        }

        /// <summary>
        /// 라디안을 도(Degree)로 변환
        /// </summary>
        public static double RadToDeg(double rad) => rad * 180.0 / Math.PI;

        /// <summary>
        /// 도(Degree)를 라디안으로 변환
        /// </summary>
        public static double DegToRad(double deg) => deg * Math.PI / 180.0;

        /// <summary>
        /// 관절 각도 배열을 도(Degree)로 변환
        /// </summary>
        public static double[] RadToDeg(double[] joints)
        {
            double[] result = new double[joints.Length];
            for (int i = 0; i < joints.Length; i++)
            {
                result[i] = RadToDeg(joints[i]);
            }
            return result;
        }

        /// <summary>
        /// 관절 각도 배열을 라디안으로 변환
        /// </summary>
        public static double[] DegToRad(double[] joints)
        {
            double[] result = new double[joints.Length];
            for (int i = 0; i < joints.Length; i++)
            {
                result[i] = DegToRad(joints[i]);
            }
            return result;
        }
    }
}