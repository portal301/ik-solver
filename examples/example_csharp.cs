/*
 * IKFast Solver C# 사용 예제
 *
 * 이 예제는 IKFastUnity_x64.dll을 사용하여 로봇의 IK/FK를 계산하는 방법을 보여줍니다.
 *
 * 빌드 방법:
 *   csc /unsafe example_csharp.cs
 *
 * 실행 전 준비:
 *   1. IKFastUnity_x64.dll을 실행 파일과 같은 디렉토리에 복사
 *   2. robots/ 디렉토리 (모든 DLL 포함)를 실행 파일과 같은 디렉토리에 복사
 */

using System;
using System.Runtime.InteropServices;

class IKFastExample
{
    // ========================================================================
    // P/Invoke 선언
    // UTF-8 인코딩을 사용하여 한글 경로 지원
    // ========================================================================

    [DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl)]
    private static extern int IKU_Init([MarshalAs(UnmanagedType.LPUTF8Str)] string robots_dir);

    [DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl)]
    private static extern int IKU_GetNumJoints([MarshalAs(UnmanagedType.LPUTF8Str)] string robot_name);

    [DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl)]
    private static extern int IKU_SolveIK(
        [MarshalAs(UnmanagedType.LPUTF8Str)] string robot_name,
        double[] tcp_pose,
        double[] out_solutions,
        int max_solutions
    );

    [DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl)]
    private static extern int IKU_SolveIKWithConfig(
        [MarshalAs(UnmanagedType.LPUTF8Str)] string robot_name,
        double[] tcp_pose,
        int shoulder_config,
        int elbow_config,
        int wrist_config,
        double[] out_joints,
        out int is_solvable
    );

    [DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl)]
    private static extern int IKU_SolveIKWithJoint(
        [MarshalAs(UnmanagedType.LPUTF8Str)] string robot_name,
        double[] tcp_pose,
        double[] current_joints,
        double[] out_joints,
        out int is_solvable
    );

    [DllImport("IKFastUnity_x64", CallingConvention = CallingConvention.Cdecl)]
    private static extern int IKU_ComputeFK(
        [MarshalAs(UnmanagedType.LPUTF8Str)] string robot_name,
        double[] joints,
        double[] out_eetrans,
        double[] out_eerot
    );

    // Configuration enum
    enum PoseConfig
    {
        NULL = -1,
        RIGHT = 0, LEFT = 1,
        UP = 0, DOWN = 1,
        N_FLIP = 0, FLIP = 1
    }

    // ========================================================================
    // Main
    // ========================================================================

    static void Main()
    {
        Console.WriteLine("IKFast Solver C# Example");
        Console.WriteLine(new string('=', 60));
        Console.WriteLine();

        // 1. 초기화
        string robotsDir = @"robots";
        if (IKU_Init(robotsDir) == 0)
        {
            Console.WriteLine("Failed to load robot plugins");
            return;
        }
        Console.WriteLine("✓ IK plugins loaded successfully\n");

        // 2. 로봇 선택
        string robotName = "gp25";
        int dof = IKU_GetNumJoints(robotName);
        Console.WriteLine($"Robot: {robotName}, DOF: {dof}");

        // 3. 목표 TCP 자세 설정 (4x4 동차 변환 행렬의 처음 3행)
        // 위치: (0.5, 0.0, 0.3) m, 회전: identity (0도)
        double[] tcpPose = new double[] {
            1, 0, 0, 0.5,   // R11, R12, R13, Tx
            0, 1, 0, 0.0,   // R21, R22, R23, Ty
            0, 0, 1, 0.3    // R31, R32, R33, Tz
        };

        Console.WriteLine("목표 TCP 자세:");
        Console.WriteLine($"  위치: ({tcpPose[3]:F3}, {tcpPose[7]:F3}, {tcpPose[11]:F3}) m");
        Console.WriteLine("  회전: Identity (0°)\n");

        // 4. IK 계산 - 모든 솔루션
        Console.WriteLine(new string('=', 60));
        Console.WriteLine("IK 계산 - 모든 솔루션");
        Console.WriteLine(new string('=', 60));

        const int maxSolutions = 48;
        double[] solutions = new double[dof * maxSolutions];
        int numSolutions = IKU_SolveIK(robotName, tcpPose, solutions, maxSolutions);

        if (numSolutions > 0)
        {
            Console.WriteLine($"찾은 솔루션 개수: {numSolutions}\n");

            // 모든 솔루션 출력
            for (int i = 0; i < numSolutions; i++)
            {
                Console.Write($"  솔루션 {i + 1}: ");
                for (int j = 0; j < dof; j++)
                {
                    double deg = solutions[i * dof + j] * 180.0 / Math.PI;
                    Console.Write($"{deg,7:F2}° ");
                }
                Console.WriteLine();
            }

            // 첫 번째 솔루션으로 FK 검증
            double[] joints = new double[dof];
            Array.Copy(solutions, 0, joints, 0, dof);

            double[] fkTrans = new double[3];
            double[] fkRot = new double[9];
            IKU_ComputeFK(robotName, joints, fkTrans, fkRot);

            double error = Math.Sqrt(
                Math.Pow(tcpPose[3] - fkTrans[0], 2) +
                Math.Pow(tcpPose[7] - fkTrans[1], 2) +
                Math.Pow(tcpPose[11] - fkTrans[2], 2)
            );

            Console.WriteLine($"\nFK 검증 (솔루션 1):");
            Console.WriteLine($"  위치: ({fkTrans[0]:F6}, {fkTrans[1]:F6}, {fkTrans[2]:F6}) m");
            Console.WriteLine($"  오차: {error:E3} m");
        }
        else
        {
            Console.WriteLine("솔루션을 찾을 수 없습니다.");
        }
        Console.WriteLine();

        // 5. 특정 Configuration으로 IK 계산
        Console.WriteLine(new string('=', 60));
        Console.WriteLine("IK 계산 - 특정 Configuration");
        Console.WriteLine(new string('=', 60));

        var configs = new[] {
            (PoseConfig.RIGHT, PoseConfig.DOWN, PoseConfig.N_FLIP, "Right-Down-NoFlip"),
            (PoseConfig.LEFT, PoseConfig.DOWN, PoseConfig.N_FLIP, "Left-Down-NoFlip"),
            (PoseConfig.RIGHT, PoseConfig.UP, PoseConfig.N_FLIP, "Right-Up-NoFlip"),
            (PoseConfig.LEFT, PoseConfig.UP, PoseConfig.FLIP, "Left-Up-Flip"),
        };

        foreach (var (shoulder, elbow, wrist, name) in configs)
        {
            double[] configJoints = new double[dof];
            int isSolvable;

            IKU_SolveIKWithConfig(
                robotName, tcpPose,
                (int)shoulder, (int)elbow, (int)wrist,
                configJoints,
                out isSolvable
            );

            if (isSolvable == 1)
            {
                Console.Write($"{name}: ");
                for (int j = 0; j < dof; j++)
                {
                    double deg = configJoints[j] * 180.0 / Math.PI;
                    Console.Write($"{deg,7:F2}° ");
                }
                Console.WriteLine();
            }
            else
            {
                Console.WriteLine($"{name}: 솔루션 없음");
            }
        }
        Console.WriteLine();

        // 6. 현재 관절 각도에서 가장 가까운 IK 솔루션
        Console.WriteLine(new string('=', 60));
        Console.WriteLine("IK 계산 - 현재 위치에서 가장 가까운 솔루션");
        Console.WriteLine(new string('=', 60));

        double[] currentJoints = new double[dof];
        Console.Write("현재 관절 각도: ");
        for (int j = 0; j < dof; j++)
        {
            Console.Write($"{currentJoints[j] * 180.0 / Math.PI,7:F2}° ");
        }
        Console.WriteLine();

        double[] nearestJoints = new double[dof];
        int isNearestSolvable;

        IKU_SolveIKWithJoint(
            robotName, tcpPose, currentJoints,
            nearestJoints,
            out isNearestSolvable
        );

        if (isNearestSolvable == 1)
        {
            Console.Write("가장 가까운 솔루션: ");
            for (int j = 0; j < dof; j++)
            {
                double deg = nearestJoints[j] * 180.0 / Math.PI;
                Console.Write($"{deg,7:F2}° ");
            }
            Console.WriteLine();

            // FK 검증
            double[] fkTrans2 = new double[3];
            double[] fkRot2 = new double[9];
            IKU_ComputeFK(robotName, nearestJoints, fkTrans2, fkRot2);

            double error2 = Math.Sqrt(
                Math.Pow(tcpPose[3] - fkTrans2[0], 2) +
                Math.Pow(tcpPose[7] - fkTrans2[1], 2) +
                Math.Pow(tcpPose[11] - fkTrans2[2], 2)
            );
            Console.WriteLine($"FK 검증 오차: {error2:E3} m");
        }
        else
        {
            Console.WriteLine("솔루션을 찾을 수 없습니다.");
        }
        Console.WriteLine();

        // 7. FK 단독 사용 예제
        Console.WriteLine(new string('=', 60));
        Console.WriteLine("FK 계산 - 관절 각도에서 TCP 자세 구하기");
        Console.WriteLine(new string('=', 60));

        double[] testJoints = new double[] { 0, 0, Math.PI / 2, 0, Math.PI / 4, 0 };
        Console.Write("입력 관절 각도: ");
        for (int j = 0; j < dof; j++)
        {
            double deg = testJoints[j] * 180.0 / Math.PI;
            Console.Write($"{deg,7:F2}° ");
        }
        Console.WriteLine();

        double[] fkTrans3 = new double[3];
        double[] fkRot3 = new double[9];
        IKU_ComputeFK(robotName, testJoints, fkTrans3, fkRot3);

        Console.WriteLine($"\nTCP 위치: ({fkTrans3[0]:F6}, {fkTrans3[1]:F6}, {fkTrans3[2]:F6}) m");
        Console.WriteLine("TCP 회전 행렬 (row-major):");
        for (int i = 0; i < 3; i++)
        {
            Console.WriteLine($"  [{fkRot3[i * 3 + 0],8:F5}, {fkRot3[i * 3 + 1],8:F5}, {fkRot3[i * 3 + 2],8:F5}]");
        }

        Console.WriteLine();
        Console.WriteLine(new string('=', 60));
        Console.WriteLine("예제 완료!");
        Console.WriteLine(new string('=', 60));
    }
}
