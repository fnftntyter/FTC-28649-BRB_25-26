package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

@Autonomous(name="JAYOOL_Final_Complete", group="ImReady")
public class JAYOOL extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // 1. 초기화 (이동 및 슈터 시스템)
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ShooterBase shooter = new ShooterBase(hardwareMap);

        telemetry.addData("Status", "JAYOOL 준비 완료 - 공 발사 로직 포함");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 2. 이동: (61, 10) 좌표 이동 후 90도 회전
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        // 1. 먼저 X축으로 이동 (바라보는 방향 유지)
                        .lineToX(9)
                        // 2. 그 다음 Y축으로 이동하면서 헤딩(방향) 변경
                        .setTangent(Math.toRadians(90)) // Y축 방향으로 경로 탄젠트 설정
                        .lineToYLinearHeading(9, Math.toRadians(200))
                        // 3. 마지막 제자리 회전
                        .turn(Math.toRadians(-26))
                        .build()
        );
        // 3. 공 발사 및 서보 복귀 시퀀스 (3회 반복)
        telemetry.addData("Action", "공 3회 발사 시작...");
        telemetry.update();

        // (1) 슈터 모터 가속 (루프 밖에서 한 번만 실행하여 속도 유지)
        shooter.setVelocity(ShooterBase.DEFAULT_TPS);
        sleep(1200); // 휠이 충분히 돌 때까지 최초 1.2초 대기

        // --- 공 3번 발사 루프 시작 ---
        for (int i = 1; i <= 3; i++) {
            telemetry.addData("Action", i + "번째 발사 중...");
            telemetry.update();

            // (2) 공 밀어넣기 실행 (서보 전진)
            shooter.pushBall();

            // (3) 서보가 제자리로 돌아올 때까지 대기
            ElapsedTime shootTimer = new ElapsedTime();
            while (opModeIsActive() && shooter.updateCycle()) {
                // 서보 복귀를 위해 계속 업데이트
                if (shootTimer.seconds() > 3.0) break; // 3초 타임아웃
            }

            // (4) 다음 공이 장전되거나 휠 속도가 회복될 시간을 위해 잠시 대기
            // 로봇의 메커니즘에 따라 이 sleep 시간은 조절이 필요할 수 있습니다.
            sleep(1000);
        }
        // --- 루프 종료 ---

        // (5) 모든 동작 완료 후 시스템 정지
        shooter.setPower(0);
    }
}