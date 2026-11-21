package org.firstinspires.ftc.teamcode.bioBuzz;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * 주행 외 기구 모터 — 인테이크(공 먹기), 파이프(공 올리기), 슈터(공 발사).
 *
 * <h3>Robot Configuration (익스펜션 허브)</h3>
 * <pre>
 *  Expansion Hub · Motors · port 0 — Pipemtr      (공 올려주는 파이프)
 *  Expansion Hub · Motors · port 1 — Intakemtr    (공 먹는 인테이크)
 *  Expansion Hub · Motors · port 2 — Shootermtr   (공 발사하는 슈터)
 * </pre>
 * 설정 화면에서 <b>모터 종류</b>도 실제 모터 모델로 골라야 한다
 * (goBILDA 5203, REV HD Hex 등). 종류가 틀리면 엔코더 눈금과 최대 RPM이 어긋난다.
 *
 * <h3>슈터는 아직 없어도 된다</h3>
 * 슈터만 {@code tryGet}으로 찾는다. 설정에 {@code Shootermtr}가 없으면
 * {@link #shooterAvailable}가 false가 되고 슈터 관련 호출은 조용히 무시된다.
 * 덕분에 슈터를 달기 전에도 주행·인테이크·파이프는 그대로 쓸 수 있다.
 * 슈터가 로봇에 영구히 붙으면 {@code tryGet} → {@code get}으로 바꿔도 된다
 * (그러면 설정이 빠졌을 때 바로 에러가 나서 실수를 빨리 잡는다).
 *
 * <h3>드라이브 모터와 섞이지 않는다</h3>
 * Pedro Pathing({@code PedroConstants})은 컨트롤 허브의 주행 모터 4개만 잡는다.
 * 이 두 모터는 익스펜션 허브에 있고 이름도 겹치지 않으므로,
 * 같은 OpMode에서 {@code createFollower()}와 함께 써도 충돌하지 않는다.
 */
@Config
public class BioBuzzMechanism {

    // ────────────────────────────────────────────────
    //  설정 파일 이름 — 드라이버 허브의 Configure Robot과 똑같이 맞출 것
    // ────────────────────────────────────────────────

    /** 익스펜션 허브 모터 0번 — 공을 위로 올려주는 파이프 */
    public static final String NAME_PIPE = "Pipemtr";
    /** 익스펜션 허브 모터 1번 — 공을 먹는 인테이크 */
    public static final String NAME_INTAKE = "Intakemtr";
    /** 익스펜션 허브 모터 2번 — 공을 발사하는 슈터 */
    public static final String NAME_SHOOTER = "shootermtr";

    // ────────────────────────────────────────────────
    //  출력값 — FtcDashboard에서 실시간으로 바꿀 수 있다
    // ────────────────────────────────────────────────

    /** 인테이크가 공을 빨아들일 때 출력 (0~1). 음수를 주면 토해낸다. */
    public static double INTAKE_POWER = 1;
    /** 파이프가 공을 올릴 때 출력 (0~1) */
    public static double PIPE_POWER = 0.8;

    /** 5203 19.2:1 기어비의 출력축 1회전당 엔코더 카운트 (초기 추정값) */
    public static double PIPE_TICKS_PER_REV = 537.7;

    /** Y 버튼을 누르고 있는 동안의 파이프 출력 (기존 1회전 동작에도 사용) */
    public static double PIPE_TURN_POWER = 0.8;

    /** 1회전 완료 판정 오차 */
    public static int PIPE_TURN_TOLERANCE = 10;

    /** 엔코더가 목표에 도달하지 못할 때 계속 도는 것을 막는 안전 제한 시간 */
    public static double PIPE_TURN_TIMEOUT_SECONDS = 1.5;

    /** RUN_TO_POSITION 명령 직후 isBusy() 상태가 갱신될 때까지 기다리는 시간 */
    private static final double PIPE_TURN_STARTUP_GRACE_SECONDS = 0.15;

    /**
     * 목표 RPM. <b>최대한 빠르게</b> 쏘기 위한 값.
     * <p>
     * 모터 무부하 최대는 1150 RPM이지만 폐루프로 그 값을 유지할 수는 없다.
     * SDK가 속도 명령의 상한을 <b>무부하 최대의 85%</b>로 잡기 때문이다
     * ({@code MotorConfigurationType.getAchieveableMaxTicksPerSecond()}
     *  = {@code ticksPerRev × maxRPM × 0.85 / 60}, fraction 기본값 0.85).
     * <pre>
     *   1150 × 0.85 = 977 RPM  ← 폐루프로 명령할 수 있는 실질 상한
     * </pre>
     * 그래서 975로 잡았다. 이보다 올려도 상한에 걸려 같은 속도가 나오고,
     * PID가 도달 못 하는 목표를 쫓느라 적분이 쌓인다(= 포화).
     * <p>
     * 발사 거리가 모자라면 이 값을 줄이는 게 아니라 슈터 기구(압축량·각도)를 봐야 한다.
     * 진짜 무부하 최대로 돌리고 싶으면 폐루프를 포기하고
     * {@code setShooterPower(1.0)}(개방루프)을 쓰면 된다 — 아래 shooterOn 주석 참고.
     */
    public static double SHOOTER_TARGET_RPM = 975.0;

    /**
     * 모터 무부하 최대 RPM. <b>F(피드포워드) 계산에 쓰이므로 참값이어야 한다.</b>
     * goBILDA 5203 5.2:1 = 1150 RPM.
     */
    public static double SHOOTER_MAX_RPM = 1150.0;

    /**
     * 출력축 1회전당 엔코더 카운트. 0이면 Robot Configuration의 모터 사양을 쓴다.
     * <p>
     * <b>지금은 0으로 두면 안 된다.</b> Robot Configuration에 슈터가
     * "GoBILDA 5202/3/4 series"로 등록돼 있는데, SDK의 그 항목은
     * <b>99.5:1 기어박스(2786 ticks/rev, 최대 60 RPM)</b> 하나로 고정돼 있다
     * (SDK의 {@code GoBILDA5202Series} 애노테이션에서 확인).
     * 실제 모터는 5.2:1이라 사양이 전혀 다르다.
     * <p>
     * 그래서 실제 값 145.1을 직접 넣는다. 이걸 안 하면 RPM 환산이 19배 틀려서
     * 목표를 낮춰도 명령이 상한에 걸려 항상 전속으로 도는 증상이 생긴다.
     * <p>
     * Robot Configuration의 모터 종류를 5.2:1에 맞는 항목으로 고치면 0으로 되돌려도 된다.
     */
    public static double SHOOTER_TICKS_PER_REV = 145.1;
    /** REV Hub 속도 PIDF 초기값. 실제 슈터에서 튜닝 필요; 단위는 RPM이 아닌 ticks/s. */
    public static double SHOOTER_P = 10.0;
    public static double SHOOTER_I = 3.0;
    public static double SHOOTER_D = 0.0;
    /** -1이면 32767 / 최대 ticks/s로 계산. 0 이상이면 이 F 값을 직접 사용. */
    public static double SHOOTER_F = -1.0;
    /** 목표 속도 도달 표시용 허용 오차. 자동 발사 조건은 아니다. */
    public static double SHOOTER_RPM_TOLERANCE = 100.0;

    // ────────────────────────────────────────────────
    //  하드웨어
    // ────────────────────────────────────────────────

    public final DcMotorEx pipe;
    public final DcMotorEx intake;
    /** 슈터. 설정에 없으면 null이다 — 직접 쓰기 전에 {@link #shooterAvailable} 확인. */
    public final DcMotorEx shooter;

    private double shooterTargetRpm = 0.0;
    private double appliedShooterP = Double.NaN;
    private double appliedShooterI = Double.NaN;
    private double appliedShooterD = Double.NaN;
    private double appliedShooterF = Double.NaN;

    // 파이프 360도 회전 진행 상태
    private boolean pipeOneTurnRunning = false;
    private boolean pipeOneTurnFailed = false;
    private final ElapsedTime pipeTurnTimer = new ElapsedTime();

    /** 슈터가 설정에 등록돼 있는가. false면 슈터 관련 호출이 전부 무시된다. */
    public final boolean shooterAvailable;

    public BioBuzzMechanism(HardwareMap hardwareMap) {
        pipe = hardwareMap.get(DcMotorEx.class, NAME_PIPE);
        intake = hardwareMap.get(DcMotorEx.class, NAME_INTAKE);

        // 슈터는 아직 안 달렸을 수 있다. get()은 없으면 예외로 죽지만
        // tryGet()은 null을 돌려주므로 주행·인테이크·파이프가 계속 쓸 수 있다.
        shooter = hardwareMap.tryGet(DcMotorEx.class, NAME_SHOOTER);
        shooterAvailable = shooter != null;

        initMotors();
    }

    /**
     * 방향 / 런모드 / 정지동작 초기화.
     * <p>
     * 파이프와 인테이크 방향은 현재 {@code REVERSE}로 설정했다.
     * 실제 기구에서 양수 출력이 공을 올리고 먹는 방향인지 확인해야 한다.
     * <p>
     * 파이프와 인테이크는 누르는 동안 출력이 바로 반영되도록
     * {@code RUN_WITHOUT_ENCODER}를 사용한다.
     */
    private void initMotors() {
        pipe.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        // 파이프: 엔코더 목표 없이 버튼을 누르는 동안 직접 출력 제어
        pipe.setPower(0);
        pipe.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pipe.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 인테이크: 기존처럼 출력 제어
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (shooterAvailable) {
            shooter.setDirection(DcMotorEx.Direction.FORWARD);
            shooter.setPower(0);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // 슈터만 FLOAT — 플라이휠을 매번 급제동시키면 기어박스에 부담이 크다.
            // 출력을 끊으면 관성으로 천천히 멈춘다.
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    // ────────────────────────────────────────────────
    //  기본 조작 — 아직 어떤 OpMode에도 연결되어 있지 않다
    // ────────────────────────────────────────────────

    /** 인테이크 직접 제어. 양수 = 먹기, 음수 = 토하기 */
    public void setIntakePower(double power) {
        intake.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * 파이프 직접 제어(자유 회전). <b>음수 = 올리기, 양수 = 내리기</b>
     * (BioBuzzDriving의 Y 버튼이 음수로 올리는 것과 같은 규약).
     * <p>
     * 한 바퀴 동작({@link #startPipeOneTurn()}) 중이면 그걸 먼저 취소한다.
     * RUN_TO_POSITION 상태에서 setPower만 부르면 자유 회전이 되는 게 아니라
     * "목표까지 가는 속도"만 바뀌어서, 부르는 쪽에서 보면 전혀 다르게 동작한다.
     */
    public void setPipePower(double power) {
        if (pipeOneTurnRunning) {
            stopPipeOneTurn(false);
        }
        pipe.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void intakeIn() {
        setIntakePower(INTAKE_POWER);
    }

    public void intakeOut() {
        setIntakePower(-INTAKE_POWER);
    }

    /** 공을 올리는 방향으로 자유 회전. 이 로봇은 <b>음수가 올리는 방향</b>이다. */
    public void pipeUp() {
        setPipePower(-PIPE_POWER);
    }

    /** 역방향(내리는 방향) 자유 회전. */
    public void pipeDown() {
        setPipePower(PIPE_POWER);
    }

    // ────────────────────────────────────────────────
    //  슈터 — 슈터가 설정에 없으면 전부 조용히 무시된다
    // ────────────────────────────────────────────────

    /** 수동 출력 시험용. RPM 제어를 해제한다. 일반 발사는 shooterOn/setShooterRpm 사용. */
    public void setShooterPower(double power) {
        shooterTargetRpm = 0.0;
        if (!shooterAvailable) {
            return;
        }
        if (shooter.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // 런모드를 벗어나면 Hub에 보낸 속도 PIDF 계수가 유지된다고 볼 수 없다.
            // 캐시를 비워서 다음 setShooterRpm()이 반드시 다시 보내게 한다.
            invalidateShooterPidfCache();
        }
        if (!Double.isFinite(power)) {
            power = 0.0;
        }
        shooter.setPower(Range.clip(power, -1.0, 1.0));
    }

    /**
     * "Hub에 이 계수를 이미 보냈다"는 기록을 지운다.
     * <p>
     * {@link #setShooterRpm(double)}은 계수가 바뀔 때만 전송해서 I2C 왕복을 아낀다.
     * 그런데 런모드를 {@code RUN_WITHOUT_ENCODER}로 바꿨다 돌아오면
     * Hub 쪽 계수가 남아 있다는 보장이 없다. 그 경우 캐시만 보고 전송을 건너뛰면
     * <b>커스텀 PIDF가 조용히 사라지고 모터 기본값으로 돌아간다.</b>
     * 그래서 런모드를 바꿀 때마다 이걸 호출한다.
     * <p>
     * NaN은 자기 자신과도 같지 않으므로({@code NaN != NaN}) 비교가 반드시 참이 되어
     * 다음 호출에서 전송이 일어난다.
     */
    private void invalidateShooterPidfCache() {
        appliedShooterP = Double.NaN;
        appliedShooterI = Double.NaN;
        appliedShooterD = Double.NaN;
        appliedShooterF = Double.NaN;
    }

    /**
     * Dashboard 목표 RPM으로 회전. TeleOp에서 매 루프 호출하면 설정 변경도 반영된다.
     * <p>
     * 폐루프(속도 유지)라서 부하가 걸려도 같은 속도를 내려고 보정한다 — 발사 일관성이 좋다.
     * 반대로 <b>무부하 최대 속도는 못 낸다</b> ({@link #SHOOTER_TARGET_RPM} 주석 참고).
     * <p>
     * 일관성보다 최대 속도가 중요하면 이 메소드 본문을 {@code setShooterPower(1.0)}으로
     * 바꾸면 개방루프 전속이 된다. 그때는 PIDF·목표 RPM·"속도 도달" 표시가 모두 무의미해지고,
     * 배터리가 닳을수록 속도가 떨어진다.
     */
    public void shooterOn() {
        setShooterRpm(SHOOTER_TARGET_RPM);
    }

    /**
     * REV Hub 내장 PIDF로 목표 속도를 유지한다. 엔코더 연결과 정확한 모터 설정이 필요하다.
     * 한 번 호출해도 Hub에서 제어가 계속된다. 0 이하/잘못된 설정은 정지 처리한다.
     */
    public void setShooterRpm(double rpm) {
        if (!shooterAvailable) {
            shooterTargetRpm = 0.0;
            return;
        }
        double ticksPerRev = getShooterTicksPerRev();
        double maxTicksPerSecond = SHOOTER_MAX_RPM * ticksPerRev / 60.0;
        double p = SHOOTER_P;
        double i = SHOOTER_I;
        double d = SHOOTER_D;
        double f = SHOOTER_F == -1.0 ? 32767.0 / maxTicksPerSecond : SHOOTER_F;
        if (!Double.isFinite(rpm) || rpm <= 0.0
                || !Double.isFinite(ticksPerRev) || ticksPerRev <= 0.0
                || !Double.isFinite(maxTicksPerSecond) || maxTicksPerSecond <= 0.0
                || !isValidShooterGain(p) || !isValidShooterGain(i)
                || !isValidShooterGain(d) || !isValidShooterGain(f)) {
            shooterOff();
            return;
        }

        if (shooter.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            shooter.setPower(0);
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // 런모드를 바꿨으니 Hub 쪽 계수를 신뢰할 수 없다 → 아래에서 반드시 재전송된다.
            invalidateShooterPidfCache();
        }
        // 설정이 바뀔 때만 Hub에 계수를 전송한다 (런모드가 바뀌었으면 위에서 캐시를 비웠다).
        if (p != appliedShooterP || i != appliedShooterI
                || d != appliedShooterD || f != appliedShooterF) {
            shooter.setVelocityPIDFCoefficients(p, i, d, f);
            appliedShooterP = p;
            appliedShooterI = i;
            appliedShooterD = d;
            appliedShooterF = f;
        }
        // Hub의 속도 명령 범위와 모터 최대 RPM을 모두 지킨다.
        shooterTargetRpm = Math.min(rpm, Math.min(SHOOTER_MAX_RPM, 32767.0 * 60.0 / ticksPerRev));
        shooter.setVelocity(shooterTargetRpm * ticksPerRev / 60.0);
    }

    private static boolean isValidShooterGain(double gain) {
        return Double.isFinite(gain) && gain >= 0.0;
    }

    public void shooterOff() {
        // FLOAT로 자연 감속: 0 RPM을 적극적으로 유지하려고 역토크를 걸지 않는다.
        setShooterPower(0);
    }

    public double getShooterTicksPerRev() {
        if (!shooterAvailable) {
            return 0.0;
        }
        return SHOOTER_TICKS_PER_REV == 0.0
                ? shooter.getMotorType().getTicksPerRev() : SHOOTER_TICKS_PER_REV;
    }

    public double getShooterTargetRpm() {
        return shooterTargetRpm;
    }

    /** 엔코더로 측정한 모터 출력축 RPM. */
    public double getShooterRpm() {
        double ticksPerRev = getShooterTicksPerRev();
        return shooterAvailable && Double.isFinite(ticksPerRev) && ticksPerRev > 0.0
                ? shooter.getVelocity() * 60.0 / ticksPerRev : 0.0;
    }

    public boolean isShooterAtSpeed() {
        return shooterAvailable && shooterTargetRpm > 0.0
                && Double.isFinite(SHOOTER_RPM_TOLERANCE) && SHOOTER_RPM_TOLERANCE >= 0.0
                && Math.abs(shooterTargetRpm - getShooterRpm()) <= SHOOTER_RPM_TOLERANCE;
    }

    /** SDK가 보고하는 power 값. 속도 제어 중 실제 PWM 출력으로 해석하지 말 것. */
    public double getShooterPower() {
        return shooterAvailable ? shooter.getPower() : 0.0;
    }

    // ────────────────────────────────────────────────

    /** 전부 정지. OpMode의 stop()에서 꼭 불러 줄 것. */
    public void stop() {
        setIntakePower(0);
        if (pipeOneTurnRunning) {
            stopPipeOneTurn(false);
        } else {
            setPipePower(0);
        }
        shooterOff();
    }

    /**
     * 현재 위치에서 파이프를 정방향으로 360도 회전시킨다.
     */
    public void startPipeOneTurn() {
        if (pipeOneTurnRunning) {
            return;
        }

        pipeOneTurnFailed = false;
        double power = Range.clip(Math.abs(PIPE_TURN_POWER), 0.0, 1.0);
        if (PIPE_TICKS_PER_REV <= 0 || power == 0) {
            stopPipeOneTurn(true);
            return;
        }

        int currentPosition = pipe.getCurrentPosition();
        int targetPosition = currentPosition - (int) Math.round(PIPE_TICKS_PER_REV);

        pipe.setTargetPosition(targetPosition);
        pipe.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pipeTurnTimer.reset();
        pipe.setPower(power);

        pipeOneTurnRunning = true;
    }

    /**
     * 회전 완료 여부를 확인하고 완료되면 정지한다.
     */
    public void updatePipeOneTurn() {
        if (!pipeOneTurnRunning) {
            return;
        }

        long remainingTicks = Math.abs((long) pipe.getTargetPosition() - pipe.getCurrentPosition());
        boolean reachedTarget = remainingTicks <= Math.max(0, PIPE_TURN_TOLERANCE);
        boolean startupGraceElapsed = pipeTurnTimer.seconds() >= PIPE_TURN_STARTUP_GRACE_SECONDS;
        boolean controllerStoppedEarly = startupGraceElapsed && !pipe.isBusy() && !reachedTarget;
        boolean timedOut = pipeTurnTimer.seconds() >= Math.max(0.1, PIPE_TURN_TIMEOUT_SECONDS);

        if (reachedTarget) {
            stopPipeOneTurn(false);
        } else if (controllerStoppedEarly || timedOut) {
            // 엔코더/모터 설정이 잘못돼도 RUN_TO_POSITION에 출력이 계속 남지 않게 한다.
            stopPipeOneTurn(true);
        }
    }

    private void stopPipeOneTurn(boolean failed) {
        pipe.setPower(0);
        pipe.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pipeOneTurnRunning = false;
        pipeOneTurnFailed = failed;
    }

    /** 360도 회전 진행 여부 */
    public boolean isPipeOneTurnRunning() {
        return pipeOneTurnRunning;
    }

    /** 직전 360도 동작이 엔코더 미도달/시간 초과로 끝났는가 */
    public boolean didPipeOneTurnFail() {
        return pipeOneTurnFailed;
    }

    /** 지금 실제로 등록된 모터들. 슈터가 없으면 2개만 돌려준다. */
    public DcMotorEx[] motors() {
        return shooterAvailable
                ? new DcMotorEx[] { pipe, intake, shooter }
                : new DcMotorEx[] { pipe, intake };
    }
}
