package org.firstinspires.ftc.teamcode.bioBuzz;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * 임시 자율주행 — <b>앞으로 갔다가 뒤로 돌아오기만</b> 한다.
 * <p>
 * Pedro Pathing 튜닝이 끝나기 전까지 쓰는 대체용이다.
 * 주행 모터 4개의 <b>엔코더만</b> 쓰므로 오도메트리(핀포인트)도, IMU도,
 * 카메라도 필요 없다. 핀포인트 I2C 문제와 무관하게 동작한다.
 *
 * <h3>노리는 점수 (BIOBUZZ)</h3>
 * <pre>
 *   LEAVE 3점 — AUTO가 끝날 때 로봇이 필드 벽에 닿아 있지 않으면
 *   PARK  5점 — AUTO가 끝날 때 로봇이 LOADING ZONE에 일부라도 들어가 있으면
 * </pre>
 * 둘 다 <b>끝나는 순간의 위치</b>로만 판정된다. 그래서 중요한 건 마지막에 멈춘 자리다.
 * <p>
 * {@link #BACK_IN}을 {@link #FORWARD_IN}보다 <b>작게</b> 두면 출발선보다 앞에서 멈춘다.
 * 출발할 때 벽에 닿아 있었으니, 조금이라도 앞에 있으면 LEAVE가 성립한다.
 * 출발 위치를 LOADING ZONE 옆에 잡으면 PARK까지 같이 받는다.
 *
 * <h3>거리 보정 — 처음 한 번은 해야 한다</h3>
 * 엔코더로 거리를 재니 {@link BioBuzzHardware#TICKS_PER_REV}와
 * {@link BioBuzzHardware#WHEEL_DIAMETER_IN}이 맞아야 한다. 지금 값은
 * goBILDA 312RPM + 104mm 바퀴 가정이다.
 * <ol>
 *   <li>{@code FORWARD_IN = 24}, {@code BACK_IN = 0}으로 두고 돌린다</li>
 *   <li>로봇이 실제로 몇 인치 갔는지 줄자로 잰다</li>
 *   <li>{@code WHEEL_DIAMETER_IN}에 {@code 기존값 × (실제거리 ÷ 24)}를 적는다</li>
 * </ol>
 * 값은 FtcDashboard에서 바로 바꿔 시험할 수 있다.
 *
 * <h3>안전</h3>
 * 각 이동은 {@link #MOVE_TIMEOUT_S}초가 지나면 포기하고 다음 단계로 넘어간다.
 * 로봇이 걸려도 AUTO가 거기서 멈춰 있지 않는다.
 */
@Config
@Autonomous(name = "TEM_auto", group = "BioBuzz", preselectTeleOp = "BioBuzz Driving")
public class TEM_auto extends LinearOpMode {

    // ────────────────────────────────────────────────
    //  FtcDashboard에서 실시간으로 바꿀 수 있다
    // ────────────────────────────────────────────────

    /** 앞으로 갈 거리 (inch). 벽에서 확실히 떨어질 만큼. */
    public static double FORWARD_IN = 24.0;

    /**
     * 돌아올 거리 (inch).
     * <b>FORWARD_IN보다 작게</b> 두면 출발선보다 앞에서 멈춰 LEAVE 3점이 유지된다.
     * 같게 두면 출발 자리로 돌아가 벽에 닿을 수 있다 (LEAVE 놓침).
     */
    public static double BACK_IN = 18.0;

    /** 이동 출력 (0~1). 낮으면 정확하고 느리다. */
    public static double POWER = 0.4;

    /** 앞으로 간 뒤 잠깐 멈추는 시간 (초). 0이면 바로 돌아온다. */
    public static double PAUSE_S = 0.5;

    /** 한 번의 이동에 허용하는 최대 시간 (초). 넘으면 포기하고 다음으로. */
    public static double MOVE_TIMEOUT_S = 5.0;

    // ────────────────────────────────────────────────

    private BioBuzzHardware robot;
    private MultipleTelemetry tel;

    @Override
    public void runOpMode() {
        robot = new BioBuzzHardware(hardwareMap);
        tel = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        tel.addLine("TEM_auto — 앞으로 갔다가 돌아온다 (엔코더 기반)");
        tel.addData("앞으로 / 돌아오기 (in)", "%.0f / %.0f", FORWARD_IN, BACK_IN);
        tel.addData("출력", "%.2f", POWER);
        tel.addLine();
        tel.addLine("로봇을 벽에 붙여 놓고 START");
        tel.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        // ① 앞으로
        drive(FORWARD_IN, "① 앞으로");

        // ② 잠깐 정지
        if (opModeIsActive() && PAUSE_S > 0) {
            ElapsedTime pause = new ElapsedTime();
            while (opModeIsActive() && pause.seconds() < PAUSE_S) {
                tel.addLine("② 정지 중");
                tel.addData("남은 시간", "%.1f", PAUSE_S - pause.seconds());
                tel.update();
            }
        }

        // ③ 뒤로 (앞으로 간 거리보다 짧게 → 출발선보다 앞에서 멈춘다)
        drive(-BACK_IN, "③ 돌아오기");

        // 끝. 브레이크가 걸린 상태로 제자리 유지.
        robot.stop();

        while (opModeIsActive()) {
            tel.addLine(">>> 완료");
            tel.addData("최종 위치 (출발선 기준, in)", "%.1f", FORWARD_IN - BACK_IN);
            tel.addLine("벽에 닿아 있지 않은지 확인 — LEAVE 3점 조건");
            showEncoders();
            tel.update();
        }
    }

    /**
     * 엔코더로 {@code inches}만큼 이동한다. 음수면 뒤로.
     * 도착하거나 {@link #MOVE_TIMEOUT_S}를 넘기면 돌아온다.
     */
    private void drive(double inches, String label) {
        if (!opModeIsActive()) {
            return;
        }

        robot.startMove(inches, 0, 0, POWER);
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && robot.isBusy() && timer.seconds() < MOVE_TIMEOUT_S) {
            tel.addLine(label);
            tel.addData("목표 (in)", "%.1f", inches);
            tel.addData("경과 (s)", "%.1f", timer.seconds());
            showEncoders();
            tel.update();
        }

        boolean timedOut = timer.seconds() >= MOVE_TIMEOUT_S;
        robot.finishMove();

        if (timedOut) {
            tel.addLine("! " + label + " 시간 초과 — 걸렸거나 거리 설정이 과하다");
            tel.update();
        }
    }

    private void showEncoders() {
        tel.addLine();
        for (DcMotorEx m : robot.motors()) {
            // 네 바퀴가 비슷하게 움직여야 정상. 한쪽만 다르면 그 모터를 의심.
            tel.addData("엔코더", "%d / 목표 %d", m.getCurrentPosition(), m.getTargetPosition());
        }
    }
}
