package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

/**
 * 고정 경로 자율 — 카메라 조준 없이 정해진 위치로 가서 정해진 속도로 쏜다.
 * <p>
 * 카메라가 말썽이거나 태그가 가려지는 자리에서 쓰는 <b>백업 루틴</b>이다.
 * 태그 기반 자동 조준은 {@link AutoBase}(AutoDrivingBLUE/RED)를 쓸 것.
 */
@Autonomous(name = "JAYOOL_Final_Complete", group = "ImReady")
public class JAYOOL extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ShooterBase shooter = new ShooterBase(hardwareMap);
        CameraBase camera = CameraBase.getInstance(hardwareMap);

        telemetry.addData("Status", "JAYOOL 준비 완료 (고정 경로 / 고정 TPS)");
        telemetry.addData("발사 TPS", ShooterBase.DEFAULT_TPS);
        telemetry.addData("발사 수", GameConfig.AUTO_SHOT_COUNT);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        try {
            // ── 이동 ──
            Actions.runBlocking(
                    drive.actionBuilder(startPose)
                            // 1. X축으로 먼저 이동 (바라보는 방향 유지)
                            .lineToX(9)
                            // 2. Y축으로 이동하면서 헤딩 변경
                            .setTangent(Math.toRadians(90))
                            .lineToYLinearHeading(9, Math.toRadians(200))
                            // 3. 마지막 제자리 회전
                            .turn(Math.toRadians(-26))
                            .build());

            // ── 발사 ──
            // 예전 코드는 sleep()으로 가속 시간을 눈대중했는데,
            // 이제 실제 인코더 속도가 목표에 닿을 때까지 기다린다.
            telemetry.addData("Action", "발사 시작");
            telemetry.update();

            shooter.setVelocity(ShooterBase.DEFAULT_TPS);
            shooter.shootBlocking(this, GameConfig.AUTO_SHOT_COUNT, GameConfig.AUTO_SPINUP_TIMEOUT_S);

        } finally {
            shooter.setPower(0);
            camera.closeCamera();
        }
    }
}
