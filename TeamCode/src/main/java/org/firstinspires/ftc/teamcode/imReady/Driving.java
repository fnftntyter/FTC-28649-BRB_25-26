package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * 수동조작 OpMode.
 * <p>
 * <b>시작 전에 반드시 진영을 확인할 것.</b> INIT 상태에서 gamepad1의
 * dpad ←(BLUE) / →(RED)로 고른다. 진영에 따라 자동 조준이 볼 태그가 달라진다.
 *
 * <pre>
 * ── gamepad1 (드라이버) ──
 *  좌스틱      이동
 *  우스틱      회전
 *  A          필드중심 ↔ 로봇중심
 *  B          자동 조준 on/off
 *  dpad       헤딩을 0/90/180/270°로 스냅
 *  범퍼       정밀 저속
 *  back       현재 방향을 필드중심 기준(0°)으로 재설정
 *
 * ── gamepad2 (슈터) ──
 *  X          슈터 on/off (켜면 거리 자동 조준)
 *  Y          1발 발사
 *  RB         연속 발사 (GameConfig.AUTO_SHOT_COUNT 발)
 *  LB         발사 취소 / 서보 원위치
 *  dpad ↑↓    효율계수 ±0.01
 *  dpad ←→    효율계수 ±0.001
 * </pre>
 */
@TeleOp(name = "Driving", group = "Ready")
public class Driving extends OpMode {

    private CameraBase camera;
    private MecanumBase drive;
    private ShooterBase shooter;

    private FtcDashboard dashboard;
    private MultipleTelemetry multipleTelemetry;

    /** 진영. INIT 중에 dpad로 고른다. */
    private GameConfig.Alliance alliance = GameConfig.Alliance.BLUE;

    @Override
    public void init() {
        camera = CameraBase.getInstance(hardwareMap);
        drive = new MecanumBase(hardwareMap, new Pose2d(0, 0, 0));
        shooter = new ShooterBase(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        camera.enableCameraStreaming(30);
    }

    /** INIT 상태에서 진영을 고르는 화면 */
    @Override
    public void init_loop() {
        if (gamepad1.dpad_left) {
            alliance = GameConfig.Alliance.BLUE;
        } else if (gamepad1.dpad_right) {
            alliance = GameConfig.Alliance.RED;
        }

        telemetry.addLine(">> dpad ←: BLUE   dpad →: RED");
        telemetry.addData("진영", alliance);
        telemetry.addData("조준할 골 태그", GameConfig.goalTag(alliance));
        telemetry.update();
    }

    @Override
    public void loop() {
        camera.update();

        drive.update(gamepad1, alliance);
        shooter.update(gamepad2, alliance);

        // 슈터가 도는 동안 거리에 맞춰 목표 속도를 계속 따라간다
        shooter.trackGoal(alliance);

        // 연속 발사 / 취소
        if (gamepad2.rightBumperWasPressed()) {
            shooter.requestShots(GameConfig.AUTO_SHOT_COUNT);
        }
        if (gamepad2.leftBumperWasPressed()) {
            shooter.cancelShots();
        }

        showData();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        camera.closeCamera();
        super.stop();
    }

    private void showData() {
        int goalTag = GameConfig.goalTag(alliance);
        boolean tagVisible = camera.isTagVisible(goalTag);

        multipleTelemetry.addData("진영", alliance);

        // ── 발사 준비 상태 (드라이버가 제일 자주 보는 줄) ──
        String ready;
        if (!shooter.isShooterSpin) {
            ready = "슈터 꺼짐";
        } else if (!shooter.isAtSpeed()) {
            ready = "가속 중...";
        } else if (drive.isAutoAiming() && !drive.isAimed()) {
            ready = "조준 중...";
        } else {
            ready = "발사 준비 완료";
        }
        multipleTelemetry.addData(">>> 상태", ready);

        // ── 조준 ──
        multipleTelemetry.addData("골 태그(" + goalTag + ")", tagVisible ? "보임" : "안 보임");
        multipleTelemetry.addData("거리(in)", tagVisible ? camera.getDistanceById(goalTag) : Double.NaN);
        multipleTelemetry.addData("조준 오차(deg)", Math.toDegrees(drive.getHeadingErrorRad()));
        multipleTelemetry.addData("자동 조준", drive.isAutoAiming() ? "ON" : "OFF");

        // ── 슈터 ──
        multipleTelemetry.addData("목표 TPS", ShooterBase.TPS);
        multipleTelemetry.addData("실제 TPS", shooter.getCurrentTps());
        multipleTelemetry.addData("좌/우 TPS",
                "%.0f / %.0f", shooter.shooterLeft.getVelocity(), shooter.shooterRight.getVelocity());
        multipleTelemetry.addData("조준 모드", ShooterBase.AIM_MODE);
        multipleTelemetry.addData("효율계수", ShooterBase.SHOOTER_EFFICENCY);
        if (shooter.didAimFail()) {
            multipleTelemetry.addLine("! 거리 계산 실패 — 기본 TPS 사용 중");
        }
        multipleTelemetry.addData("발사 수", shooter.getShotsFired());

        // ── 주행 ──
        multipleTelemetry.addData("주행 모드", drive.isFieldCentric() ? "필드중심" : "로봇중심");
        multipleTelemetry.addData("보이는 태그", camera.getVisibleTagIds());

        multipleTelemetry.update();
    }
}
