package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class ShooterBase extends ShooterHardware {

    // ────────────────────────────────────────────────
    //  속도 제어
    // ────────────────────────────────────────────────

    public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(35, 0, 10, 13);

    /** 현재 목표 TPS (읽기 전용으로 볼 것. 바꾸려면 setVelocity 사용) */
    public static double TPS = 1000;

    /** 태그가 안 보일 때 쓰는 기본 TPS */
    public static double DEFAULT_TPS = 910;

    /** 모터가 낼 수 있는 상한 */
    public static double MAX_TPS = 2800;

    /** 목표 속도 도달로 인정하는 오차 (TPS) */
    public static double TPS_TOLERANCE = 40;

    // ────────────────────────────────────────────────
    //  서보 (공 밀어넣기)
    // ────────────────────────────────────────────────

    public static double SERVO_MAX = 0.8;
    public static double SERVO_MIN = 0;
    /** 서보가 끝까지 밀고 나갈 때까지 (ms) */
    public static double SERVO_PUSH_MS = 1100;
    /** 서보가 원위치로 돌아올 때까지 (ms) */
    public static double SERVO_RETRACT_MS = 1100;

    // ────────────────────────────────────────────────
    //  슈터 물리 제원
    // ────────────────────────────────────────────────

    public static double FLYWHEEL_RADIUS = 1.889765; // 바퀴 반지름 (in)
    /** 슬립·압축 손실을 뭉뚱그린 효율계수. 1이면 슬립 없음. 실측으로 맞출 것. */
    public static double SHOOTER_EFFICENCY = 0.58;
    public static double TICK_PER_REV = 28; // 모터 인코더 해상도 (PPR)
    public static double GEAR_RATIO = 1;

    private static final double G_IN_PER_S2 = 386.1; // 중력가속도 (in/s^2)

    // ────────────────────────────────────────────────
    //  조준 방식
    // ────────────────────────────────────────────────

    public enum AimMode {
        /** 포물선 운동 공식으로 계산. 튜닝 파라미터가 효율계수 하나뿐이라 거리 편차에 약하다. */
        PHYSICS,
        /** 실측한 (거리, TPS) 표를 선형보간. 표가 2점 미만이면 자동으로 PHYSICS로 떨어진다. */
        TABLE,
        /** 거리 무시하고 항상 DEFAULT_TPS */
        FIXED
    }

    public static AimMode AIM_MODE = AimMode.PHYSICS;

    /**
     * 실측 표 — {거리(inch), TPS} 쌍을 <b>거리 오름차순</b>으로 넣는다.
     * ShooterTuner OpMode로 값을 뽑은 뒤 여기에 붙여넣을 것.
     * <p>
     * 예: {@code { {40, 880}, {60, 960}, {80, 1050} }}
     */
    public static double[][] DIST_TPS_TABLE = {};

    // ────────────────────────────────────────────────

    /** 마지막으로 계산에 쓴 거리 (텔레메트리용) */
    private double lastDistance = 0;
    /** 마지막 조준 계산이 실패했는지 (사각지대 등) */
    private boolean lastAimFailed = false;

    public boolean isShooterSpin = false;

    private final CameraBase camera;

    private enum ShotState { IDLE, PUSHING, RETRACTING }

    private ShotState shotState = ShotState.IDLE;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private int shotsQueued = 0;
    private int shotsFired = 0;

    public ShooterBase(HardwareMap hardwareMap) {
        super(hardwareMap);
        camera = CameraBase.getInstance(hardwareMap);

        // PIDF는 여기서 한 번만 쓴다.
        // 예전 코드는 update() 안에서 매 루프마다 설정해 Lynx 버스를 두 번씩 때렸고,
        // 그 때문에 루프 타임이 튀었다.
        applyPidf();

        setServoPosition(SERVO_MIN);
    }

    /** 대시보드에서 PIDF를 바꿨을 때만 수동으로 부른다. */
    public void applyPidf() {
        shooterLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, SHOOTER_PID);
        shooterRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, SHOOTER_PID);
    }

    // ────────────────────────────────────────────────
    //  TeleOp 조작
    // ────────────────────────────────────────────────

    /**
     * gamepad2 기준 조작:
     * <ul>
     *   <li>X — 슈터 on/off (켤 때 보이는 골 태그로 거리 자동 조준)</li>
     *   <li>Y — 공 1발 발사</li>
     *   <li>dpad ↑/↓ — 효율계수 ±0.01, ←/→ — ±0.001</li>
     * </ul>
     *
     * @param alliance 우리 진영. 어느 골 태그를 볼지 결정한다.
     */
    public void update(Gamepad gamepad, GameConfig.Alliance alliance) {
        if (gamepad.xWasPressed()) {
            isShooterSpin = !isShooterSpin;
            if (isShooterSpin) {
                aimAtGoal(alliance);
            } else {
                setPower(0);
            }
        }

        // 슈터 효율 미세조정
        if (gamepad.dpadUpWasPressed()) {
            SHOOTER_EFFICENCY += 0.01;
        } else if (gamepad.dpadDownWasPressed()) {
            SHOOTER_EFFICENCY -= 0.01;
        } else if (gamepad.dpadLeftWasPressed()) {
            SHOOTER_EFFICENCY += 0.001;
        } else if (gamepad.dpadRightWasPressed()) {
            SHOOTER_EFFICENCY -= 0.001;
        }
        // 0 이하로 내려가면 TPS가 발산하므로 반드시 막는다.
        SHOOTER_EFFICENCY = Range.clip(SHOOTER_EFFICENCY, 0.05, 1.5);

        if (gamepad.yWasPressed()) {
            requestShot();
        }

        updateShotCycle();
    }

    /**
     * 슈터가 돌고 있는 동안 거리에 맞춰 목표 TPS를 계속 갱신한다.
     * 로봇이 움직이면서 쏠 때 쓴다.
     * <p>
     * 태그가 잠깐 안 보여도 직전 속도를 그대로 유지한다.
     * (매번 DEFAULT_TPS로 되돌리면 태그가 깜빡일 때마다 휠 속도가 출렁인다.)
     */
    public void trackGoal(GameConfig.Alliance alliance) {
        if (!isShooterSpin) return;

        AprilTagDetection detection = camera.getDetectionById(GameConfig.goalTag(alliance));
        if (detection != null) {
            activeShooterTps(detection);
        }
    }

    /** 우리 진영 골 태그를 보고 거리에 맞는 TPS를 건다. 태그가 없으면 DEFAULT_TPS. */
    public void aimAtGoal(GameConfig.Alliance alliance) {
        AprilTagDetection detection = camera.getDetectionById(GameConfig.goalTag(alliance));
        if (detection != null) {
            activeShooterTps(detection);
        } else {
            lastAimFailed = true;
            setVelocity(DEFAULT_TPS);
        }
    }

    // ────────────────────────────────────────────────
    //  속도 명령
    // ────────────────────────────────────────────────

    public void setVelocity(double tps) {
        TPS = Range.clip(tps, 0, MAX_TPS);
        shooterLeft.setVelocity(TPS);
        shooterRight.setVelocity(TPS);
    }

    public void addVelocity(double amount) {
        setVelocity(TPS + amount);
    }

    public void setPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    /** @deprecated 이름이 헷갈려서 남겨둔 것. {@link #setVelocity(double)}를 쓸 것. */
    @Deprecated
    public void addTPS(double amount) {
        addVelocity(amount);
    }

    public double getCurrentTps() {
        return (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
    }

    /** 두 휠 모두 목표 속도에 도달했는가. 발사 전에 이걸로 확인한다. */
    public boolean isAtSpeed() {
        if (TPS <= 0) return false;
        return Math.abs(shooterLeft.getVelocity() - TPS) < TPS_TOLERANCE
                && Math.abs(shooterRight.getVelocity() - TPS) < TPS_TOLERANCE;
    }

    // ────────────────────────────────────────────────
    //  거리 → TPS
    // ────────────────────────────────────────────────

    /** 태그로 측정한 거리를 바탕으로 목표 TPS를 계산해 건다. */
    public void activeShooterTps(AprilTagDetection detection) {
        setVelocity(tpsForDistance(camera.getDistanceByTag(detection)));
    }

    /**
     * 수평거리(inch)에 필요한 TPS.
     * {@link #AIM_MODE}에 따라 실측 표 또는 포물선 공식을 쓴다.
     */
    public double tpsForDistance(double distanceIn) {
        lastDistance = distanceIn;
        lastAimFailed = false;

        if (distanceIn <= 0) {
            lastAimFailed = true;
            return DEFAULT_TPS;
        }

        if (AIM_MODE == AimMode.FIXED) {
            return DEFAULT_TPS;
        }

        if (AIM_MODE == AimMode.TABLE && DIST_TPS_TABLE.length >= 2) {
            return interpolateTable(distanceIn);
        }

        return tpsFromPhysics(distanceIn);
    }

    /** 실측 표 선형보간. 표 범위를 벗어나면 양 끝값으로 클램프한다. */
    private double interpolateTable(double distanceIn) {
        double[][] t = DIST_TPS_TABLE;

        if (distanceIn <= t[0][0]) return t[0][1];
        if (distanceIn >= t[t.length - 1][0]) return t[t.length - 1][1];

        for (int i = 0; i < t.length - 1; i++) {
            double d0 = t[i][0], d1 = t[i + 1][0];
            if (distanceIn >= d0 && distanceIn <= d1) {
                double span = d1 - d0;
                if (span <= 0) return t[i][1]; // 거리가 중복된 항목 방어
                double ratio = (distanceIn - d0) / span;
                return t[i][1] + ratio * (t[i + 1][1] - t[i][1]);
            }
        }
        return DEFAULT_TPS; // 도달 불가
    }

    /**
     * 포물선 운동으로 필요한 초기 속도를 구하고 TPS로 변환한다.
     * <p>
     * h = x·tanθ − g·x² / (2·v0²·cos²θ) 를 v0에 대해 푼 것.
     */
    private double tpsFromPhysics(double distanceIn) {
        double theta = Math.toRadians(GameConfig.LAUNCH_ANGLE_DEG);
        double deltaH = GameConfig.GOAL_HEIGHT_DIFF_IN;

        // 이 값이 0 이하면 현재 발사각으로는 그 높이에 도달할 수 없다 (사각지대).
        double discriminant = (distanceIn * Math.tan(theta)) - deltaH;
        if (discriminant <= 0) {
            lastAimFailed = true;
            return DEFAULT_TPS;
        }

        double v0 = Math.sqrt(
                (G_IN_PER_S2 * distanceIn * distanceIn)
                        / (2 * Math.pow(Math.cos(theta), 2) * discriminant));

        return calculateTPSFromVelocity(v0);
    }

    /**
     * 발사 속도(in/s) → 모터 TPS.
     * TPS = (v0 / efficiency / r) × (ticks_per_rev / 2π) × gear_ratio
     */
    private double calculateTPSFromVelocity(double v0) {
        double wheelTangentialVelocity = v0 / SHOOTER_EFFICENCY;   // 슬립 보정
        double angularVelocity = wheelTangentialVelocity / FLYWHEEL_RADIUS; // rad/s
        return angularVelocity * (TICK_PER_REV / (2 * Math.PI)) * GEAR_RATIO;
    }

    public double getLastDistance() {
        return lastDistance;
    }

    /** 마지막 조준 계산이 실패했는지 (태그 없음 / 사각지대) */
    public boolean didAimFail() {
        return lastAimFailed;
    }

    // ────────────────────────────────────────────────
    //  발사 시퀀스 (논블로킹 상태 머신)
    // ────────────────────────────────────────────────

    public void setServoPosition(double position) {
        servo.setPosition(Range.clip(position, 0, 1));
    }

    /** 1발 발사를 예약한다. 이미 발사 중이면 큐에 쌓인다. */
    public void requestShot() {
        shotsQueued++;
    }

    /** n발 연속 발사를 예약한다. */
    public void requestShots(int count) {
        shotsQueued += Math.max(0, count);
    }

    /** 예약된 발사를 모두 취소하고 서보를 원위치로 보낸다. */
    public void cancelShots() {
        shotsQueued = 0;
        shotState = ShotState.IDLE;
        setServoPosition(SERVO_MIN);
    }

    /** @deprecated {@link #requestShot()}을 쓸 것. */
    @Deprecated
    public void pushBall() {
        requestShot();
    }

    /**
     * 발사 시퀀스를 한 스텝 진행시킨다. <b>매 루프 반드시 호출해야 한다.</b>
     *
     * @return 아직 발사 중이면 true, 모두 끝났으면 false.
     *         (예전 {@code updateCycle()}은 이 값이 반대로 나와서
     *          {@code while(updateCycle())}이 무한루프가 될 수 있었다.)
     */
    public boolean updateShotCycle() {
        switch (shotState) {
            case IDLE:
                if (shotsQueued > 0) {
                    shotsQueued--;
                    shotState = ShotState.PUSHING;
                    shotTimer.reset();
                    setServoPosition(SERVO_MAX);
                }
                break;

            case PUSHING:
                if (shotTimer.milliseconds() >= SERVO_PUSH_MS) {
                    shotState = ShotState.RETRACTING;
                    shotTimer.reset();
                    setServoPosition(SERVO_MIN);
                }
                break;

            case RETRACTING:
                if (shotTimer.milliseconds() >= SERVO_RETRACT_MS) {
                    shotState = ShotState.IDLE;
                    shotsFired++;
                }
                break;
        }
        return isShotCycleBusy();
    }

    /** 발사 중이거나 발사 대기가 남아 있으면 true */
    public boolean isShotCycleBusy() {
        return shotState != ShotState.IDLE || shotsQueued > 0;
    }

    public int getShotsFired() {
        return shotsFired;
    }

    /** @deprecated 반환값 의미가 뒤집혀 있던 옛 이름. {@link #updateShotCycle()}을 쓸 것. */
    @Deprecated
    public boolean updateCycle() {
        return updateShotCycle();
    }

    // ────────────────────────────────────────────────
    //  자율주행용 블로킹 헬퍼
    // ────────────────────────────────────────────────

    /**
     * 슈터가 목표 속도에 도달할 때까지 기다린다.
     *
     * @return 제한시간 안에 도달했으면 true
     */
    public boolean waitForSpinUp(LinearOpMode opMode, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < timeoutSeconds) {
            if (isAtSpeed()) return true;
            opMode.idle();
        }
        return isAtSpeed();
    }

    /**
     * n발을 순서대로 쏘고, 다 쏠 때까지 블로킹한다. 자율주행에서 쓴다.
     * 한 발마다 휠 속도가 회복되기를 기다린 뒤 밀어넣는다.
     */
    public void shootBlocking(LinearOpMode opMode, int count, double perShotTimeoutSeconds) {
        for (int i = 0; i < count && opMode.opModeIsActive(); i++) {
            waitForSpinUp(opMode, perShotTimeoutSeconds);

            requestShot();
            ElapsedTime timer = new ElapsedTime();
            // 서보 왕복(밀기+복귀)에 필요한 시간 + 여유
            double limit = (SERVO_PUSH_MS + SERVO_RETRACT_MS) / 1000.0 + 1.0;
            while (opMode.opModeIsActive() && updateShotCycle()) {
                if (timer.seconds() > limit) {
                    cancelShots(); // 서보가 걸린 경우 — 다음 발로 넘어간다
                    break;
                }
                opMode.idle();
            }
        }
    }
}
