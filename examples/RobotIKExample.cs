/*
 * Robot IK Example for Unity
 *
 * Unity 씬에서 로봇의 IK/FK를 테스트하는 MonoBehaviour 스크립트입니다.
 * GameObject에 부착하여 실행하면 콘솔에 IK/FK 계산 결과를 출력합니다.
 *
 * 사용 방법:
 *   1. 빈 GameObject를 생성 (예: "RobotIKTester")
 *   2. 이 스크립트를 GameObject에 부착
 *   3. Inspector에서 Robot Name 설정 (예: "gp25")
 *   4. Play 모드에서 실행
 *
 * Inspector 설정:
 *   - Robot Name: 사용할 로봇 이름 (gp25, gp12, kj125 등)
 *   - Target Position: 목표 TCP 위치 (미터)
 *   - Target Rotation: 목표 TCP 회전 (Euler 각도)
 *   - Auto Initialize: 시작 시 자동 초기화 여부
 */

using UnityEngine;
using IKFast;

public class RobotIKExample : MonoBehaviour
{
    [Header("Robot Settings")]
    [Tooltip("로봇 이름 (예: gp25, gp12, kj125)")]
    public string robotName = "gp25";

    [Header("Target Pose")]
    [Tooltip("목표 TCP 위치 (미터)")]
    public Vector3 targetPosition = new Vector3(0.5f, 0f, 0.3f);

    [Tooltip("목표 TCP 회전 (Euler 각도)")]
    public Vector3 targetRotation = Vector3.zero;

    [Header("IK Settings")]
    [Tooltip("시작 시 자동으로 IKFast 초기화")]
    public bool autoInitialize = true;

    [Tooltip("IK 솔루션 중 사용할 인덱스 (0부터 시작)")]
    public int solutionIndex = 0;

    [Header("Configuration")]
    [Tooltip("어깨 구성 (RIGHT 또는 LEFT)")]
    public RobotConfig shoulderConfig = RobotConfig.RIGHT;

    [Tooltip("팔꿈치 구성 (UP 또는 DOWN)")]
    public RobotConfig elbowConfig = RobotConfig.DOWN;

    [Tooltip("손목 구성 (N_FLIP 또는 FLIP)")]
    public RobotConfig wristConfig = RobotConfig.N_FLIP;

    [Header("Debug")]
    [Tooltip("IK 솔루션을 콘솔에 출력")]
    public bool debugLogSolutions = true;

    [Tooltip("FK 검증을 콘솔에 출력")]
    public bool debugLogFK = true;

    // 현재 로봇의 관절 각도 (라디안)
    private double[] currentJoints = null;

    // 로봇 자유도
    private int dof = 0;

    void Start()
    {
        if (autoInitialize)
        {
            InitializeIKFast();
        }
    }

    /// <summary>
    /// IKFast 초기화
    /// </summary>
    public void InitializeIKFast()
    {
        if (IKFastSolver.IsInitialized)
        {
            Debug.Log("IKFast is already initialized");
            return;
        }

        // Unity 프로젝트의 Plugins 폴더 경로
        // 빌드 시에는 Data 폴더로 변경될 수 있음
        string robotsPath = Application.dataPath + "/Plugins/x86_64/robots";

        // Editor에서만 동작하는 경로 (Plugins 직접 참조)
#if UNITY_EDITOR
        robotsPath = Application.dataPath + "/Plugins/x86_64/robots";
#else
        // 빌드된 애플리케이션에서는 실행 파일 기준 상대 경로
        robotsPath = Application.dataPath + "/Plugins/robots";
#endif

        bool success = IKFastSolver.Initialize(robotsPath);
        if (!success)
        {
            Debug.LogError($"Failed to initialize IKFast from: {robotsPath}");
            Debug.LogError("Make sure IKFastUnity_x64.dll and robots/ folder are in Assets/Plugins/x86_64/");
            return;
        }

        // 로봇 DOF 조회
        dof = IKFastSolver.GetNumJoints(robotName);
        if (dof <= 0)
        {
            Debug.LogError($"Invalid robot name: {robotName}");
            return;
        }

        Debug.Log($"IKFast initialized. Robot: {robotName}, DOF: {dof}");

        // 초기 관절 각도 설정 (0도)
        currentJoints = new double[dof];

        // 관절 제한 조회
        var limits = IKFastSolver.GetJointLimits(robotName);
        if (limits.HasValue)
        {
            Debug.Log($"Joint Limits for {robotName}:");
            for (int i = 0; i < dof; i++)
            {
                Debug.Log($"  Joint {i + 1}: [{IKFastSolver.RadToDeg(limits.Value.Lower[i]):F2}°, " +
                          $"{IKFastSolver.RadToDeg(limits.Value.Upper[i]):F2}°]");
            }
        }
    }

    /// <summary>
    /// IK 계산 - 모든 솔루션
    /// </summary>
    public void SolveIKAllSolutions()
    {
        if (!CheckInitialized()) return;

        Matrix4x4 targetPose = CreateTargetPose();
        var solutions = IKFastSolver.SolveIK(robotName, targetPose);

        if (solutions.Length == 0)
        {
            Debug.LogWarning("No IK solution found");
            return;
        }

        Debug.Log($"Found {solutions.Length} IK solutions for target: " +
                  $"pos={targetPosition}, rot={targetRotation}");

        if (debugLogSolutions)
        {
            for (int i = 0; i < solutions.Length; i++)
            {
                double[] jointsDeg = IKFastSolver.RadToDeg(solutions[i]);
                string jointsStr = string.Join(", ", System.Array.ConvertAll(jointsDeg, j => $"{j:F2}°"));
                Debug.Log($"  Solution {i + 1}: [{jointsStr}]");
            }
        }

        // 선택한 솔루션을 현재 관절 각도로 설정
        if (solutionIndex >= 0 && solutionIndex < solutions.Length)
        {
            currentJoints = solutions[solutionIndex];
            Debug.Log($"Selected solution {solutionIndex + 1}");

            // FK 검증
            if (debugLogFK)
            {
                VerifyFK(targetPose);
            }
        }
    }

    /// <summary>
    /// IK 계산 - 특정 구성
    /// </summary>
    public void SolveIKWithConfiguration()
    {
        if (!CheckInitialized()) return;

        Matrix4x4 targetPose = CreateTargetPose();
        double[] joints;

        bool success = IKFastSolver.SolveIKWithConfig(
            robotName, targetPose,
            shoulderConfig, elbowConfig, wristConfig,
            out joints
        );

        if (!success)
        {
            Debug.LogWarning($"No IK solution found for configuration: " +
                            $"{shoulderConfig}-{elbowConfig}-{wristConfig}");
            return;
        }

        currentJoints = joints;
        double[] jointsDeg = IKFastSolver.RadToDeg(joints);
        string jointsStr = string.Join(", ", System.Array.ConvertAll(jointsDeg, j => $"{j:F2}°"));

        Debug.Log($"IK solution for {shoulderConfig}-{elbowConfig}-{wristConfig}: [{jointsStr}]");

        // FK 검증
        if (debugLogFK)
        {
            VerifyFK(targetPose);
        }
    }

    /// <summary>
    /// IK 계산 - 현재 위치에서 가장 가까운 솔루션
    /// </summary>
    public void SolveIKNearest()
    {
        if (!CheckInitialized()) return;
        if (currentJoints == null || currentJoints.Length != dof)
        {
            Debug.LogWarning("Current joints not set. Using zero position.");
            currentJoints = new double[dof];
        }

        Matrix4x4 targetPose = CreateTargetPose();
        double[] nearestJoints;

        bool success = IKFastSolver.SolveIKNearest(
            robotName, targetPose, currentJoints,
            out nearestJoints
        );

        if (!success)
        {
            Debug.LogWarning("No IK solution found near current position");
            return;
        }

        currentJoints = nearestJoints;
        double[] jointsDeg = IKFastSolver.RadToDeg(nearestJoints);
        string jointsStr = string.Join(", ", System.Array.ConvertAll(jointsDeg, j => $"{j:F2}°"));

        Debug.Log($"Nearest IK solution: [{jointsStr}]");

        // FK 검증
        if (debugLogFK)
        {
            VerifyFK(targetPose);
        }
    }

    /// <summary>
    /// FK 계산
    /// </summary>
    public void ComputeForwardKinematics()
    {
        if (!CheckInitialized()) return;
        if (currentJoints == null || currentJoints.Length != dof)
        {
            Debug.LogWarning("Current joints not set");
            return;
        }

        var result = IKFastSolver.ComputeFK(robotName, currentJoints);
        if (!result.HasValue)
        {
            Debug.LogError("FK computation failed");
            return;
        }

        Matrix4x4 tcpPose = result.Value;
        Vector3 position = tcpPose.GetPosition();
        Quaternion rotation = tcpPose.rotation;

        Debug.Log($"FK Result:");
        Debug.Log($"  Position: {position}");
        Debug.Log($"  Rotation: {rotation.eulerAngles}");
    }

    /// <summary>
    /// 목표 자세 생성
    /// </summary>
    private Matrix4x4 CreateTargetPose()
    {
        return Matrix4x4.TRS(
            targetPosition,
            Quaternion.Euler(targetRotation),
            Vector3.one
        );
    }

    /// <summary>
    /// FK 검증
    /// </summary>
    private void VerifyFK(Matrix4x4 targetPose)
    {
        var fkResult = IKFastSolver.ComputeFK(robotName, currentJoints);
        if (!fkResult.HasValue)
        {
            Debug.LogError("FK verification failed");
            return;
        }

        Vector3 targetPos = targetPose.GetPosition();
        Vector3 fkPos = fkResult.Value.GetPosition();
        float error = Vector3.Distance(targetPos, fkPos);

        Debug.Log($"FK Verification:");
        Debug.Log($"  Target Position: {targetPos}");
        Debug.Log($"  FK Position: {fkPos}");
        Debug.Log($"  Position Error: {error:E3} m");
    }

    /// <summary>
    /// 초기화 확인
    /// </summary>
    private bool CheckInitialized()
    {
        if (!IKFastSolver.IsInitialized)
        {
            Debug.LogError("IKFast not initialized. Call InitializeIKFast() first or enable Auto Initialize.");
            return false;
        }
        return true;
    }

    /// <summary>
    /// 에디터 전용: Inspector에서 버튼으로 테스트
    /// </summary>
#if UNITY_EDITOR
    [ContextMenu("Initialize IKFast")]
    private void MenuInitialize()
    {
        InitializeIKFast();
    }

    [ContextMenu("Solve IK - All Solutions")]
    private void MenuSolveIKAll()
    {
        SolveIKAllSolutions();
    }

    [ContextMenu("Solve IK - With Configuration")]
    private void MenuSolveIKConfig()
    {
        SolveIKWithConfiguration();
    }

    [ContextMenu("Solve IK - Nearest")]
    private void MenuSolveIKNearest()
    {
        SolveIKNearest();
    }

    [ContextMenu("Compute FK")]
    private void MenuComputeFK()
    {
        ComputeForwardKinematics();
    }
#endif
}