package org.firstinspires.ftc.teamcode.bioBuzz;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * BIOBUZZ (2026-27) 시즌 드라이브 베이스 하드웨어 — <b>모터 배선 확인 전용.</b>
 * <p>
 * 오도메트리 / IMU / Road Runner / 카메라에 <b>아무것도 의존하지 않는다.</b>
 * 주행 모터 4개만 설정에 등록돼 있으면 돌아가므로,
 * 로봇을 막 조립했을 때 제일 먼저 돌리는 {@code BioBuzzMotorTest}가 이걸 쓴다.
 * <p>
 * 실제 주행(수동/자율)은 Pedro Pathing을 쓴다 — {@code pedro.PedroConstants} 참고.
 * 방향 추정은 핀포인트에 내장된 IMU가 하므로, 여기서 허브 내장 IMU를 쓸 일이 없다.
 *
 * <h3>좌표 약속 (이 패키지 전체 공통)</h3>
 * <ul>
 *   <li>{@code y} — 전후. <b>앞이 양수</b></li>
 *   <li>{@code x} — 좌우. <b>오른쪽이 양수</b></li>
 *   <li>{@code rx} — 회전. <b>시계방향(위에서 봤을 때)이 양수</b></li>
 * </ul>
 * ※ {@code imReady}의 MecanumBase는 왼쪽/CCW를 양수로 쓴다. 섞어 쓰지 말 것.
 *
 * <h3>로봇 설정 파일(Robot Configuration)에 넣을 이름</h3>
 * <pre>
 *  Control Hub Motor 0 — leftFront
 *  Control Hub Motor 1 — leftBack
 *  Control Hub Motor 2 — rightFront
 *  Control Hub Motor 3 — rightBack
 * </pre>
 * 이 네 개만 필요하다. 핀포인트나 IMU가 설정에 없어도 MotorTest는 실행된다.
 */
@Config
public class BioBuzzHardware {

    // ────────────────────────────────────────────────
    //  설정 파일 이름 — 드라이버 허브의 Configure Robot과 똑같이 맞출 것
    // ────────────────────────────────────────────────

    public static final String NAME_LEFT_FRONT  = "leftFront";
    public static final String NAME_LEFT_BACK   = "leftBack";
    public static final String NAME_RIGHT_FRONT = "rightFront";
    public static final String NAME_RIGHT_BACK  = "rightBack";

    // ────────────────────────────────────────────────
    //  주행 설정 — FtcDashboard에서 실시간 조정 가능
    // ────────────────────────────────────────────────

    /** 일반 주행 속도 배율 */
    public static double SPEED_NORMAL = 0.8;
    /** 범퍼를 눌렀을 때의 정밀 주행 배율 */
    public static double SPEED_SLOW = 0.3;

    /** 스틱 중앙 흔들림을 무시하는 범위 */
    public static double STICK_DEADZONE = 0.05;

    // ────────────────────────────────────────────────
    //  엔코더 제원 — 바퀴/기어를 바꾸면 반드시 다시 적을 것
    // ────────────────────────────────────────────────

    /**
     * 모터 1회전당 엔코더 틱.
     * goBILDA 5203 312RPM(19.2:1) = 537.7
     * goBILDA 5203 435RPM(13.7:1) = 384.5
     */
    public static double TICKS_PER_REV = 537.7;

    /** 바퀴 지름 (inch). goBILDA 104mm 메카넘 = 4.0945 */
    public static double WHEEL_DIAMETER_IN = 4.0945;

    /**
     * 좌우 이동(스트레이프) 보정계수.
     * 메카넘은 옆으로 갈 때 롤러가 미끄러져 실제 이동거리가 짧다.
     * 1칸(24인치) 옆으로 보내보고 (목표거리 / 실제거리)를 여기 적는다.
     */
    public static double STRAFE_CORRECTION = 1.15;

    /**
     * 제자리 회전 시 바퀴 중심 간 대각선 길이 (inch).
     * (트랙폭² + 휠베이스²)의 제곱근. 줄자로 실측해서 적을 것.
     */
    public static double TURN_DIAMETER_IN = 18.0;

    // ────────────────────────────────────────────────
    //  하드웨어
    // ────────────────────────────────────────────────

    public final DcMotorEx leftFront, leftBack, rightFront, rightBack;

    public BioBuzzHardware(HardwareMap hardwareMap) {
        leftFront  = hardwareMap.get(DcMotorEx.class, NAME_LEFT_FRONT);
        leftBack   = hardwareMap.get(DcMotorEx.class, NAME_LEFT_BACK);
        rightFront = hardwareMap.get(DcMotorEx.class, NAME_RIGHT_FRONT);
        rightBack  = hardwareMap.get(DcMotorEx.class, NAME_RIGHT_BACK);

        initMotors();
    }

    /**
     * 모터 방향 / 런모드 / 정지동작 초기화.
     * <p>
     * 방향은 {@code imReady.MecanumHardware}와 같은 전제(왼쪽 FORWARD, 오른쪽 REVERSE)를 쓴다.
     * 이러면 네 모터에 <b>같은 양수 출력</b>을 주면 로봇이 앞으로 간다.
     * 로봇을 새로 조립해서 배선이 다르면 BioBuzzMotorTest로 한 개씩 확인한 뒤 여기만 고친다.
     */
    private void initMotors() {
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        resetEncoders();

        for (DcMotorEx m : motors()) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // ────────────────────────────────────────────────
    //  기본 움직임 — 앞뒤좌우 + 회전
    // ────────────────────────────────────────────────

    /**
     * 메카넘 출력 믹싱. 수동조작과 자율 모두 결국 이 메서드를 거친다.
     * <p>
     * 데드존은 여기서 걸지 않는다. 저속 배율을 곱한 뒤에 걸면 작은 입력이
     * 통째로 사라지기 때문에, 조이스틱 생값에 {@link #deadzone(double)}를
     * 먼저 적용한 다음 배율을 곱해서 넘겨야 한다.
     *
     * @param x  좌우 — 오른쪽이 양수
     * @param y  전후 — 앞이 양수
     * @param rx 회전 — 시계방향이 양수
     */
    public void drive(double x, double y, double rx) {
        double lf = y + x + rx;
        double lb = y - x + rx;
        double rf = y - x - rx;
        double rb = y + x - rx;

        // 어느 하나가 1을 넘으면 네 바퀴를 같은 비율로 줄인다.
        // 그냥 clip하면 비율이 깨져서 로봇이 엉뚱한 방향으로 흐른다.
        double max = Math.max(Math.max(Math.abs(lf), Math.abs(lb)),
                              Math.max(Math.abs(rf), Math.abs(rb)));
        if (max > 1.0) {
            lf /= max;
            lb /= max;
            rf /= max;
            rb /= max;
        }

        setPowers(lf, lb, rf, rb);
    }

    /** 앞으로 ({@code power} 양수) / 뒤로 (음수) */
    public void forward(double power) {
        drive(0, power, 0);
    }

    /** 오른쪽으로 ({@code power} 양수) / 왼쪽으로 (음수) */
    public void strafe(double power) {
        drive(power, 0, 0);
    }

    /** 제자리 회전. 시계방향이 양수 */
    public void turn(double power) {
        drive(0, 0, power);
    }

    public void setPowers(double lf, double lb, double rf, double rb) {
        leftFront.setPower(Range.clip(lf, -1.0, 1.0));
        leftBack.setPower(Range.clip(lb, -1.0, 1.0));
        rightFront.setPower(Range.clip(rf, -1.0, 1.0));
        rightBack.setPower(Range.clip(rb, -1.0, 1.0));
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    // ────────────────────────────────────────────────
    //  엔코더 — 거리 기반 이동 (자율 기초)
    // ────────────────────────────────────────────────

    /** inch → 엔코더 틱 */
    public int inchesToTicks(double inches) {
        return (int) Math.round(inches * TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_IN));
    }

    /**
     * 목표 지점을 설정하고 RUN_TO_POSITION으로 출발시킨다.
     * <b>블로킹이 아니다</b> — 호출한 쪽에서 {@link #isBusy()}가 false가 될 때까지 돌아야 한다.
     * (OpMode를 멈출 수 없게 만들지 않으려고 일부러 이렇게 나눴다.)
     * <p>
     * 세 성분을 동시에 넣을 수도 있지만, 메카넘 엔코더 주행은 조합하면 오차가 커진다.
     * 한 번에 하나씩 쓰는 편이 낫다.
     *
     * @param forwardIn 앞으로 갈 거리 (inch, 음수면 뒤로)
     * @param strafeIn  오른쪽으로 갈 거리 (inch, 음수면 왼쪽)
     * @param turnDeg   시계방향 회전각 (도, 음수면 반시계)
     * @param power     이동 속도 (0~1)
     */
    public void startMove(double forwardIn, double strafeIn, double turnDeg, double power) {
        int f = inchesToTicks(forwardIn);
        int s = inchesToTicks(strafeIn * STRAFE_CORRECTION);
        int t = inchesToTicks(Math.PI * TURN_DIAMETER_IN * turnDeg / 360.0);

        // drive()의 믹싱과 부호가 같아야 한다
        int dLf = f + s + t;
        int dLb = f - s + t;
        int dRf = f - s - t;
        int dRb = f + s - t;

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + dLf);
        leftBack.setTargetPosition(leftBack.getCurrentPosition() + dLb);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + dRf);
        rightBack.setTargetPosition(rightBack.getCurrentPosition() + dRb);

        for (DcMotorEx m : motors()) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(Math.abs(power));
        }
    }

    /** RUN_TO_POSITION 이동이 아직 끝나지 않았는가 */
    public boolean isBusy() {
        for (DcMotorEx m : motors()) {
            if (m.isBusy()) return true;
        }
        return false;
    }

    /** RUN_TO_POSITION을 끝내고 일반 수동 주행 모드로 되돌린다. */
    public void finishMove() {
        stop();
        for (DcMotorEx m : motors()) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void resetEncoders() {
        for (DcMotorEx m : motors()) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    // ────────────────────────────────────────────────
    //  내부 헬퍼
    // ────────────────────────────────────────────────

    public DcMotorEx[] motors() {
        return new DcMotorEx[] { leftFront, leftBack, rightFront, rightBack };
    }

    /** 조이스틱 생값의 중앙 흔들림을 잘라낸다. 배율을 곱하기 전에 쓸 것. */
    public static double deadzone(double v) {
        return Math.abs(v) < STICK_DEADZONE ? 0 : v;
    }
        

}
