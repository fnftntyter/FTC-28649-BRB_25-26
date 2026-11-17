package org.firstinspires.ftc.teamcode.bioBuzz;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static double INTAKE_POWER = 0.8;
    /** 파이프가 공을 올릴 때 출력 (0~1) */
    public static double PIPE_POWER = 1.0;

    /** 5203 19.2:1 기어비의 출력축 1회전당 엔코더 카운트 (초기 추정값) */
    public static double PIPE_TICKS_PER_REV = 537.7;

    /** Y 버튼을 누르고 있는 동안의 파이프 출력 (기존 1회전 동작에도 사용) */
    public static double PIPE_TURN_POWER = 0.3;

    /** 1회전 완료 판정 오차 */
    public static int PIPE_TURN_TOLERANCE = 10;

    /** 엔코더가 목표에 도달하지 못할 때 계속 도는 것을 막는 안전 제한 시간 */
    public static double PIPE_TURN_TIMEOUT_SECONDS = 1.5;

    /** RUN_TO_POSITION 명령 직후 isBusy() 상태가 갱신될 때까지 기다리는 시간 */
    private static final double PIPE_TURN_STARTUP_GRACE_SECONDS = 0.15;

    /**
     * 슈터 출력 (0~1).
     * 발사 거리는 이 값으로 조절한다. 멀리 보내려면 올리고, 너무 멀리 가면 내린다.
     * 실제 발사 시험은 기구를 달고 나서 — 지금은 자리만 잡아 둔 값이다.
     */
    public static double SHOOTER_POWER = 0.5;

    // ────────────────────────────────────────────────
    //  하드웨어
    // ────────────────────────────────────────────────

    public final DcMotorEx pipe;
    public final DcMotorEx intake;
    /** 슈터. 설정에 없으면 null이다 — 직접 쓰기 전에 {@link #shooterAvailable} 확인. */
    public final DcMotorEx shooter;

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
            shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    /** 파이프 직접 제어. 양수 = 올리기, 음수 = 내리기 */
    public void setPipePower(double power) {
        pipe.setPower(Range.clip(power, -1.0, 1.0));
    }

    public void intakeIn() {
        setIntakePower(INTAKE_POWER);
    }

    public void intakeOut() {
        setIntakePower(-INTAKE_POWER);
    }

    public void pipeUp() {
        setPipePower(PIPE_POWER);
    }

    public void pipeDown() {
        setPipePower(-PIPE_POWER);
    }

    // ────────────────────────────────────────────────
    //  슈터 — 슈터가 설정에 없으면 전부 조용히 무시된다
    // ────────────────────────────────────────────────

    /** 슈터 직접 제어. 양수 = 발사 방향. 슈터가 없으면 아무 일도 하지 않는다. */
    public void setShooterPower(double power) {
        if (!shooterAvailable) {
            return;
        }
        shooter.setPower(Range.clip(power, -1.0, 1.0));
    }

    /** 슈터를 {@link #SHOOTER_POWER} 세기로 돌린다. */
    public void shooterOn() {
        setShooterPower(SHOOTER_POWER);
    }

    public void shooterOff() {
        setShooterPower(0);
    }

    /** 지금 슈터에 들어가는 출력. 슈터가 없으면 0. (텔레메트리용) */
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
