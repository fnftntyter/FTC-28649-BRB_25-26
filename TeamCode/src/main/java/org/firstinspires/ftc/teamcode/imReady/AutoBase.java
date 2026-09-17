package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 자율주행 공통 루틴.
 * <p>
 * BLUE / RED는 {@link #alliance()}만 다르고 나머지는 전부 같다.
 * 좌표·회전량·발사 수는 {@link GameConfig}에서 읽으므로,
 * 룰이 바뀌면 이 파일이 아니라 GameConfig를 고친다.
 * <p>
 * 순서: 모티프 판독(INIT) → 조준 회전 → 카메라 미세 조준 → 스핀업 → 발사 → 파킹
 */
public abstract class AutoBase extends LinearOpMode {

    /** 이 OpMode의 진영 */
    protected abstract GameConfig.Alliance alliance();

    /** 이번 판에 읽은 패턴/모티프 태그 ID. 못 읽었으면 -1. */
    protected int motifTagId = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        GameConfig.Alliance alliance = alliance();
        Pose2d startPose = GameConfig.startPose(alliance);
        int goalTag = GameConfig.goalTag(alliance);

        MecanumBase drive = new MecanumBase(hardwareMap, startPose);
        ShooterBase shooter = new ShooterBase(hardwareMap);
        CameraBase camera = CameraBase.getInstance(hardwareMap);

        try {
            // ── INIT: 패턴 태그를 읽어둔다 ──
            // START를 누르기 전까지 계속 보므로, 한 번이라도 보이면 값이 남는다.
            while (opModeInInit()) {
                camera.update();
                int seen = camera.getMotifTagId();
                if (seen != -1) {
                    motifTagId = seen;
                }

                telemetry.addData("진영", alliance);
                telemetry.addData("시작 좌표", "(%.1f, %.1f, %.0f°)",
                        startPose.position.x, startPose.position.y,
                        Math.toDegrees(startPose.heading.log()));
                telemetry.addData("패턴 태그", motifTagId == -1 ? "아직 못 읽음" : motifTagId);
                telemetry.addData("골 태그(" + goalTag + ")",
                        camera.isTagVisible(goalTag) ? "보임" : "안 보임");
                telemetry.update();
            }

            if (isStopRequested()) return;

            // ── 1. 골 쪽으로 대략 조준 ──
            Actions.runBlocking(
                    drive.getRoadRunnerDrive()
                            .actionBuilder(startPose)
                            // Road Runner의 turn()은 라디안을 받는다.
                            // GameConfig가 도 → 라디안 변환과 RED 대칭을 함께 처리한다.
                            .turn(GameConfig.aimTurnRad(alliance))
                            .build());

            // ── 2. 카메라로 미세 조준 ──
            aimWithCamera(drive, camera, goalTag, GameConfig.AUTO_AIM_TIMEOUT_S);

            // ── 3. 거리에 맞춘 속도로 스핀업 후 발사 ──
            shooter.aimAtGoal(alliance);
            telemetry.addData("발사 거리(in)", shooter.getLastDistance());
            telemetry.addData("목표 TPS", ShooterBase.TPS);
            telemetry.update();

            shooter.shootBlocking(this, GameConfig.AUTO_SHOT_COUNT, GameConfig.AUTO_SPINUP_TIMEOUT_S);
            shooter.setPower(0);

            // ── 4. 파킹 ──
            if (GameConfig.AUTO_DO_PARK && opModeIsActive()) {
                drive.getRoadRunnerDrive().updatePoseEstimate();
                Pose2d here = drive.getRoadRunnerDrive().localizer.getPose();

                // splineTo 대신 strafeToLinearHeading을 쓴다.
                // 발사 후 헤딩이 어디를 보고 있든 경로가 깨지지 않는다.
                Actions.runBlocking(
                        drive.getRoadRunnerDrive()
                                .actionBuilder(here)
                                .strafeToLinearHeading(GameConfig.parkPosition(alliance),
                                        GameConfig.parkHeadingRad(alliance))
                                .build());
            }

        } finally {
            // STOP을 눌러 중간에 끊겨도 슈터는 반드시 멈추고 카메라는 반드시 닫는다.
            // (카메라를 안 닫으면 다음 OpMode에서 초기화가 깨진다.)
            shooter.setPower(0);
            drive.stopMotors();
            camera.closeCamera();
        }
    }

    /**
     * 골 태그가 보이는 동안 헤딩을 맞춘다. 제한시간 안에 정렬되면 조기 종료한다.
     * 태그가 끝내 안 보이면 1번 단계의 회전각을 그대로 믿고 진행한다.
     */
    private void aimWithCamera(MecanumBase drive, CameraBase camera, int goalTag, double timeoutSeconds) {
        if (timeoutSeconds <= 0) return;

        ElapsedTime timer = new ElapsedTime();
        boolean everSaw = false;

        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            camera.update();

            if (drive.aimAt(goalTag)) {
                everSaw = true;
                if (drive.isAimed()) break;
            }
            drive.holdHeading();
        }

        drive.freezeHeading();
        drive.stopMotors();

        telemetry.addData("카메라 조준", everSaw ? "태그 정렬 완료" : "태그 못 봄 — 고정각으로 진행");
        telemetry.update();
    }
}
