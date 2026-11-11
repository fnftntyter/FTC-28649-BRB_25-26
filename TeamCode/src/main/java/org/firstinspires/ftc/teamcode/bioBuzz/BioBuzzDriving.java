package org.firstinspires.ftc.teamcode.bioBuzz;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bioBuzz.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.bioBuzz.pedro.PoseStorage;

/**
 * BIOBUZZ 수동조작 — Pedro Pathing 내장 드라이브 사용.
 * <p>
 * 바퀴를 직접 돌리지 않고 Pedro의 {@link Follower}에게 "이만큼 움직여"라고 시킨다.
 * 그래서 자율과 수동이 <b>같은 모터 설정, 같은 위치 추정(Pinpoint)</b>을 쓴다.
 * 모터 이름/방향을 바꾸려면 {@link PedroConstants#drivetrainConfig} 한 곳만 고치면 된다.
 *
 * <pre>
 * ── gamepad1 ──
 *  좌스틱        이동 (앞뒤좌우)
 *  우스틱 ←→     제자리 회전
 *
 *  X            인테이크 (공 먹기)      ─┐ 누르고 있는 동안만 돈다
 *  Y            파이프 (공 올리기)       ─┤ 손을 떼면 바로 멈춤
 *  B            슈터 (공 발사)          ─┘
 *
 *  LT(왼 트리거)  당기고 있는 동안 정밀 저속
 *  LB           필드중심 ↔ 로봇중심 전환
 *  RB           스틱을 놓으면 그 자리에 버티기(HOLD) on/off
 *               ※ {@link #HOLD_ENABLED}가 false면 눌러도 동작하지 않는다 (튜닝 전 기본값)
 *
 *  dpad ↑←↓→    누르고 있는 동안 그 방향으로 로봇 방향 고정 (필드 기준 0/90/180/270°)
 *  back         지금 로봇이 보는 방향을 "정면"으로 재설정 (필드중심 기준)
 *
 *  A, RT        안 씀 (비어 있음)
 * </pre>
 * 기구 출력 세기는 {@link BioBuzzMechanism#INTAKE_POWER} /
 * {@link BioBuzzMechanism#PIPE_POWER} / {@link BioBuzzMechanism#SHOOTER_POWER}
 * 이고 FtcDashboard에서 실시간으로 조절할 수 있다.
 *
 * <h3>필드중심 주행에서 "정면"이란</h3>
 * TeleOp을 시작한 순간 로봇이 보던 방향이 정면이다.
 * 드라이버가 서 있는 쪽 반대편을 로봇이 보게 놓고 시작하거나,
 * 시작 후 로봇을 그렇게 돌려놓고 back을 누르면 된다.
 */
@Config
@TeleOp(name = "BioBuzz Driving", group = "BioBuzz")
public class BioBuzzDriving extends OpMode {

    // ────────────────────────────────────────────────
    //  조정값 — FtcDashboard에서 실시간으로 바꿀 수 있다
    // ────────────────────────────────────────────────

    /** 일반 주행 속도 배율 (0~1) */
    public static double SPEED_NORMAL = 0.8;
    /** 범퍼를 눌렀을 때 속도 배율 (0~1) */
    public static double SPEED_SLOW = 0.3;

    /** 스틱 중앙 흔들림 무시 범위 */
    public static double STICK_DEADZONE = 0.05;

    /** 트리거를 "당겼다"고 인정하는 깊이 (0~1). LT 정밀 저속에 쓴다. */
    public static double TRIGGER_THRESHOLD = 0.3;

    /**
     * dpad 방향 고정의 회전 세기 (P 게인).
     * 목표 방향을 지나쳐서 흔들리면 줄이고, 너무 느리게 돌면 키운다.
     */
    public static double HEADING_LOCK_P = 1.5;
    /** dpad 방향 고정의 흔들림 억제 (D 게인). 흔들리면 조금씩 키운다. */
    public static double HEADING_LOCK_D = 0.1;

    /**
     * B 버튼의 "놓으면 그 자리 버티기(HOLD)" 기능을 쓸 수 있게 할지.
     * <p>
     * <b>기본 false — 튜닝이 끝나기 전에는 켜면 안 된다.</b>
     * HOLD는 Pedro의 {@code follower.hold()}를 쓰는 폐루프 위치 제어라서
     * 아래 두 가지가 모두 맞아야 동작한다:
     * <ol>
     *   <li>{@code PedroConstants.localizerConfig} — 핀포인트 오프셋/포드 방향.
     *       x나 y 부호가 반대면 위치 보정이 양의 피드백이 되어
     *       로봇이 스스로 가속해 달아난다.</li>
     *   <li>{@code PedroConstants.foresightConfig} — 추종 게인 12개.
     *       임시값 상태면 제자리에서 진동하거나 엉뚱한 방향으로 밀린다.</li>
     * </ol>
     * 둘 다 끝낸 뒤 FtcDashboard에서 이 값을 true로 바꿔 시험해 보고,
     * 괜찮으면 여기 기본값도 true로 적는다.
     */
    public static boolean HOLD_ENABLED = false;

    // ────────────────────────────────────────────────

    private Follower follower;
    private BioBuzzMechanism mechanism;
    private MultipleTelemetry multipleTelemetry;

    /** dpad 방향 고정용 PID. 상태를 가지므로 한 번만 만든다. */
    private final PIDController headingPid = Controller.pid(HEADING_LOCK_P, 0.0, HEADING_LOCK_D);

    private boolean fieldCentric = true;
    /**
     * 스틱을 놓으면 그 자리를 지키는 기능. 튜닝 전에는 흔들릴 수 있어 기본 OFF.
     * {@code PedroConstants.foresightConfig} 튜닝을 끝낸 뒤 켜 볼 것.
     */
    private boolean holdWhenReleased = false;

    /** 필드중심 주행의 "정면" 방향 (rad) */
    private double forwardHeading = 0.0;

    @Override
    public void init() {
        follower = PedroConstants.createFollower(hardwareMap);
        mechanism = new BioBuzzMechanism(hardwareMap);
        multipleTelemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("BioBuzz Driving (Pedro) 준비 완료");
        telemetry.addData("자율 위치 이어받기", PoseStorage.lastPose != null ? "예" : "아니오 — (0,0,0)에서 시작");
        telemetry.update();
    }

    @Override
    public void start() {
        // 자율을 돌렸으면 그 끝 위치에서, 아니면 원점에서 시작한다.
        Pose startPose = (PoseStorage.lastPose != null) ? PoseStorage.lastPose : Pose.zero();
        follower.setPose(startPose);
        forwardHeading = startPose.heading();
    }

    @Override
    public void loop() {
        // ① 위치 추정 + 모터 출력. 매 루프 반드시 한 번 불러야 한다.
        follower.update();

        // ② 버튼 처리
        // LB — 필드중심 ↔ 로봇중심
        if (gamepad1.leftBumperWasPressed()) {
            fieldCentric = !fieldCentric;
        }
        // RB — HOLD on/off. HOLD_ENABLED가 false면 아무 일도 하지 않는다.
        // (튜닝 전에 켜면 로봇이 제자리에서 멋대로 움직인다. 위 HOLD_ENABLED 주석 참고)
        if (gamepad1.rightBumperWasPressed() && HOLD_ENABLED) {
            holdWhenReleased = !holdWhenReleased;
        }
        // 켜 둔 상태에서 Dashboard로 HOLD_ENABLED를 끄면 바로 풀린다.
        if (!HOLD_ENABLED) {
            holdWhenReleased = false;
        }
        if (gamepad1.backWasPressed()) {
            forwardHeading = follower.pose().heading();
        }

        // ③ 스틱 → Pedro 규약으로 변환
        //    Pedro의 DrivePowers(앞, 왼쪽, 반시계) — 셋 다 그 방향이 +다.
        //    게임패드 스틱은 위·오른쪽이 +(y는 위가 −)라서 전부 부호를 뒤집는다.
        // 정밀 저속 — LT. (범퍼는 LB/RB 토글로 옮겼으므로 트리거를 쓴다)
        double speed = (gamepad1.left_trigger > TRIGGER_THRESHOLD) ? SPEED_SLOW : SPEED_NORMAL;
        double forward = -deadzone(gamepad1.left_stick_y) * speed;
        double left    = -deadzone(gamepad1.left_stick_x) * speed;
        double turn    = -deadzone(gamepad1.right_stick_x) * speed;

        DrivePowers powers = fieldCentric
                ? ManualDrive.fieldCentric(forward, left, turn, follower.pose().heading(), forwardHeading)
                : new DrivePowers(forward, left, turn);

        // ④ dpad: 누르고 있는 동안 로봇 방향을 고정 (회전 스틱 대신 자동으로 돌린다)
        Double lockHeading = dpadHeading();
        if (lockHeading != null) {
            // Dashboard에서 바꾼 값이 바로 먹도록 매 루프 게인을 다시 넣는다.
            headingPid.kP = HEADING_LOCK_P;
            headingPid.kD = HEADING_LOCK_D;
            powers = ManualDrive.headingLock(follower, headingPid, powers, forwardHeading + lockHeading);
        } else {
            headingPid.reset();
        }

        // ⑤ 실제로 보내기
        if (holdWhenReleased) {
            // 스틱을 움직이면 주행, 놓고 로봇이 멈추면 그 자리를 지킨다.
            ManualDrive.driveOrHold(follower, powers);
        } else {
            follower.manual(powers);
        }

        // ⑥ 기구 모터 — 누르고 있는 동안만 돈다. else로 0을 주지 않으면
        //    손을 떼도 계속 돌아간다 (setPower는 한 번 주면 유지되므로).
        if (gamepad1.x  ) {
            mechanism.intakeIn();
        } else {
            mechanism.setIntakePower(0);
        }

        if (gamepad1.y) {
            mechanism.pipeUp();
        } else {
            mechanism.setPipePower(0);
        }

        // 슈터 — B를 누르고 있는 동안 돈다.
        // 슈터가 설정에 없으면 mechanism 쪽에서 알아서 무시한다.
        if (gamepad1.b) {
            mechanism.shooterOn();
        } else {
            mechanism.shooterOff();
        }

        showData(speed);
    }

    @Override
    public void stop() {
        mechanism.stop();
        follower.stop();
        super.stop();
    }

    /** dpad가 눌렸으면 정면 기준 목표 각도(rad), 아니면 null */
    private Double dpadHeading() {
        if (gamepad1.dpad_up)    return 0.0;
        if (gamepad1.dpad_left)  return Math.toRadians(90);
        if (gamepad1.dpad_down)  return Math.toRadians(180);
        if (gamepad1.dpad_right) return Math.toRadians(-90);
        return null;
    }

    private static double deadzone(double v) {
        return Math.abs(v) < STICK_DEADZONE ? 0.0 : v;
    }

    private void showData(double speed) {
        Pose pose = follower.pose();

        multipleTelemetry.addData("주행 모드(LB)", fieldCentric ? "필드중심" : "로봇중심");
        multipleTelemetry.addData("놓으면 버티기(RB)",
                !HOLD_ENABLED ? "잠김 — 튜닝 전 (HOLD_ENABLED=false)"
                              : (holdWhenReleased ? "ON" : "OFF"));
        multipleTelemetry.addData("속도 배율", "%.2f", speed);
        multipleTelemetry.addData("Follower 상태", follower.mode());
        multipleTelemetry.addLine();
        multipleTelemetry.addData("인테이크(X)", "%.2f", mechanism.intake.getPower());
        multipleTelemetry.addData("파이프(Y)", "%.2f", mechanism.pipe.getPower());
        multipleTelemetry.addData("슈터(B)", mechanism.shooterAvailable
                ? String.format("%.2f", mechanism.getShooterPower())
                : "모터 없음 — 설정에 " + BioBuzzMechanism.NAME_SHOOTER + " 추가 필요");
        multipleTelemetry.addLine();
        multipleTelemetry.addData("위치 X / Y (in)", "%.1f / %.1f", pose.x(), pose.y());
        multipleTelemetry.addData("방향 (deg)", "%.1f", Math.toDegrees(pose.heading()));
        multipleTelemetry.addData("정면 기준 (deg)", "%.1f", Math.toDegrees(forwardHeading));
        multipleTelemetry.update();
    }
}
