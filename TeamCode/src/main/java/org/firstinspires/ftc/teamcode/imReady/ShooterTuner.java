package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * 거리별 TPS 실측표를 만드는 튜닝 OpMode.
 *
 * <h3>쓰는 법</h3>
 * <ol>
 *   <li>로봇을 골에서 적당한 거리에 놓는다 (태그가 보여야 한다).</li>
 *   <li>gamepad1 X로 슈터를 켠다.</li>
 *   <li>gamepad1 Y로 한 발씩 쏴 보면서, dpad ↑/↓(±25 TPS), ←/→(±5 TPS)로
 *       공이 골에 정확히 들어가는 TPS를 찾는다.</li>
 *   <li>맞으면 <b>A</b>를 눌러 (거리, TPS)를 기록한다.</li>
 *   <li>로봇을 다른 거리로 옮겨 3~6개 지점을 반복한다.</li>
 *   <li>다 되면 <b>START(▶)</b>를 눌러 자바 코드를 출력시킨다.
 *       화면에 나온 줄을 {@link ShooterBase#DIST_TPS_TABLE}에 그대로 붙여넣는다.</li>
 *   <li>마지막으로 {@link ShooterBase#AIM_MODE}를 {@code TABLE}로 바꾼다.</li>
 * </ol>
 *
 * <p>B를 누르면 마지막 기록을 취소한다.
 *
 * <p>실측표를 쓰는 이유: 포물선 공식은 튜닝 손잡이가 효율계수 하나뿐이라
 * 공기저항·압축량이 거리에 따라 달라지는 실제 슈터를 잘 못 따라간다.
 * 몇 지점만 실측해 보간하는 쪽이 훨씬 잘 맞는다.
 */
@Config
@TeleOp(name = "ShooterTuner", group = "Test")
public class ShooterTuner extends OpMode {

    /** 어느 진영 골로 튜닝할지 */
    public static GameConfig.Alliance ALLIANCE = GameConfig.Alliance.BLUE;

    public static double TPS_STEP_COARSE = 25;
    public static double TPS_STEP_FINE = 5;
    public static double START_TPS = 900;

    private CameraBase camera;
    private ShooterBase shooter;
    private MecanumBase drive;
    private MultipleTelemetry multipleTelemetry;

    private final List<double[]> samples = new ArrayList<>();
    private double targetTps = START_TPS;
    private boolean printCode = false;

    @Override
    public void init() {
        camera = CameraBase.getInstance(hardwareMap);
        shooter = new ShooterBase(hardwareMap);
        drive = new MecanumBase(hardwareMap, new Pose2d(0, 0, 0));

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        camera.enableCameraStreaming(30);

        targetTps = START_TPS;
    }

    @Override
    public void loop() {
        camera.update();

        int goalTag = GameConfig.goalTag(ALLIANCE);
        boolean tagVisible = camera.isTagVisible(goalTag);
        double distance = camera.getDistanceById(goalTag);

        // ── 로봇 이동 (거리를 바꿔가며 재려면 움직여야 한다) ──
        drive.update(gamepad1, ALLIANCE);

        // ── 슈터 on/off ──
        if (gamepad1.xWasPressed()) {
            shooter.isShooterSpin = !shooter.isShooterSpin;
            if (shooter.isShooterSpin) {
                shooter.setVelocity(targetTps);
            } else {
                shooter.setPower(0);
            }
        }

        // ── TPS 조절 ──
        double before = targetTps;
        if (gamepad1.dpadUpWasPressed()) targetTps += TPS_STEP_COARSE;
        else if (gamepad1.dpadDownWasPressed()) targetTps -= TPS_STEP_COARSE;
        else if (gamepad1.dpadRightWasPressed()) targetTps += TPS_STEP_FINE;
        else if (gamepad1.dpadLeftWasPressed()) targetTps -= TPS_STEP_FINE;

        if (targetTps < 0) targetTps = 0;
        if (targetTps != before && shooter.isShooterSpin) {
            shooter.setVelocity(targetTps);
        }

        // ── 발사 ──
        if (gamepad1.yWasPressed()) {
            shooter.requestShot();
        }
        shooter.updateShotCycle();

        // ── 샘플 기록 / 취소 ──
        if (gamepad1.aWasPressed() && tagVisible) {
            samples.add(new double[] { distance, targetTps });
        }
        if (gamepad1.bWasPressed() && !samples.isEmpty()) {
            samples.remove(samples.size() - 1);
        }
        if (gamepad1.startWasPressed()) {
            printCode = !printCode;
        }

        // ── 화면 ──
        multipleTelemetry.addData("진영 / 골 태그", "%s / %d", ALLIANCE, goalTag);
        multipleTelemetry.addData("태그", tagVisible ? "보임" : "안 보임 !!");
        multipleTelemetry.addData("거리(in)", tagVisible ? distance : Double.NaN);
        multipleTelemetry.addLine();
        multipleTelemetry.addData("목표 TPS", targetTps);
        multipleTelemetry.addData("실제 TPS", shooter.getCurrentTps());
        multipleTelemetry.addData("속도 도달", shooter.isAtSpeed() ? "OK" : "가속 중");
        multipleTelemetry.addLine();
        multipleTelemetry.addData("기록된 샘플", samples.size());
        multipleTelemetry.addLine("A: 기록  B: 마지막 취소  START: 코드 출력");

        if (printCode) {
            multipleTelemetry.addLine();
            multipleTelemetry.addLine("── ShooterBase.DIST_TPS_TABLE 에 붙여넣기 ──");
            for (String line : buildTableCode()) {
                multipleTelemetry.addLine(line);
            }
        }

        multipleTelemetry.update();
    }

    @Override
    public void stop() {
        shooter.setPower(0);
        camera.closeCamera();
        super.stop();
    }

    /** 기록한 샘플을 거리 오름차순으로 정렬해 자바 배열 리터럴로 만든다. */
    private List<String> buildTableCode() {
        List<String> lines = new ArrayList<>();

        if (samples.isEmpty()) {
            lines.add("(기록된 샘플이 없습니다)");
            return lines;
        }

        List<double[]> sorted = new ArrayList<>(samples);
        Collections.sort(sorted, new Comparator<double[]>() {
            @Override
            public int compare(double[] a, double[] b) {
                return Double.compare(a[0], b[0]);
            }
        });

        lines.add("public static double[][] DIST_TPS_TABLE = {");
        for (double[] s : sorted) {
            lines.add(String.format(Locale.US, "        { %.1f, %.0f },", s[0], s[1]));
        }
        lines.add("};");
        return lines;
    }
}
