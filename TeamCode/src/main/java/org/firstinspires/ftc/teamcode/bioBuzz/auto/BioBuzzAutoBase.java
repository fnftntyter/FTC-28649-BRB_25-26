package org.firstinspires.ftc.teamcode.bioBuzz.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bioBuzz.pedro.PedroConstants;
import org.firstinspires.ftc.teamcode.bioBuzz.pedro.PoseStorage;

import static com.pedropathing.api.Paths.line;

/**
 * BIOBUZZ 자율주행 공통 뼈대. RED / BLUE 파일이 이 클래스를 물려받는다.
 * <p>
 * 이 파일에는 "무엇을 어떤 순서로 하는가"만 있다.
 * "어디로 가는가"(좌표)는 {@link BioBuzzAutoRed}, {@link BioBuzzAutoBlue}에 따로 있다.
 *
 * <h3>AUTO 30초 동안 하는 일</h3>
 * <pre>
 *  ① 출발 위치 → 앞으로 이동 (벽에서 떨어진다)
 *  ② 돌아와서 LOADING ZONE으로 이동 → 거기서 제자리 유지
 * </pre>
 * 노리는 점수는 <b>LEAVE 3점 + PARK 5점 = 8점</b>이다. 발사(HIVE TIP)는 안 한다.
 *
 * <h3>두 점수 모두 "AUTO가 끝나는 순간"에 판정된다</h3>
 * <ul>
 *   <li>LEAVE 3점 — 끝났을 때 로봇이 필드 벽에 <b>닿아 있지 않으면</b></li>
 *   <li>PARK 5점 — 끝났을 때 로봇이 LOADING ZONE에 <b>일부라도 들어가 있으면</b></li>
 * </ul>
 * 즉 중요한 건 <b>마지막에 멈춘 자리</b>다. 파킹 지점이 벽에 닿아 있으면
 * PARK는 받고 LEAVE를 놓치므로, {@code parkPose()}는 존 안에 걸치되 벽에서 떨어뜨려야 한다.
 * <p>
 * 발사 동작을 넣을 때는 {@code loop()}의 {@code DRIVE_FORWARD} 단계가 끝나는 자리에 끼운다.
 *
 * <h3>좌표계 (팀 약속 — RED/BLUE 파일 모두 이 기준)</h3>
 * 공식 타일 격자(A~F열 × 1~6행)를 그대로 좌표로 쓴다. 타일 한 칸 = 24인치.
 * <pre>
 *                     먼 쪽 벽 (관중 반대편)
 *          y=144 ┌─────┬─────┬─────┬─────┬─────┬─────┐
 *                │ A6  │ B6  │ C6  │ D6  │ E6  │ F6  │ ← BLUE GARDEN
 *          y=120 ├─────┼─────┼─────┼─────┼─────┼─────┤
 *  RED LZ →      │ A5  │ B5  │ C5  │ D5  │ E5  │ F5  │
 *          y= 96 ├─────┼─────┼─────┼─────┼─────┼─────┤
 *   RED 벽       │ A4  │ B4  │ C4  │ D4  │ E4  │ F4  │      BLUE 벽
 *   (x = 0)  y=72├─────┼─────┼─────┼─────┼─────┼─────┤      (x = 144)
 *                │ A3  │ B3  │ C3  │ D3  │ E3  │ F3  │
 *          y= 48 ├─────┼─────┼─────┼─────┼─────┼─────┤
 *                │ A2  │ B2  │ C2  │ D2  │ E2  │ F2  │ ← BLUE LZ
 *          y= 24 ├─────┼─────┼─────┼─────┼─────┼─────┤
 *  RED GARDEN →  │ A1  │ B1  │ C1  │ D1  │ E1  │ F1  │
 *           y= 0 └─────┴─────┴─────┴─────┴─────┴─────┘
 *                x=0        관중석 쪽 벽          x=144
 *                ╰── RED 쪽 (A·B·C) ─╯╰─ BLUE 쪽 (D·E·F) ─╯
 * </pre>
 * <ul>
 *   <li>원점 (0, 0) = 관중석 쪽 벽과 RED 쪽 벽이 만나는 모서리</li>
 *   <li>+x = RED 벽 → BLUE 벽 방향, +y = 관중석 → 먼 쪽 방향, 단위 inch</li>
 *   <li>방향(heading) 0° = +x를 바라봄, 반시계(위에서 볼 때)가 +</li>
 *   <li>필드 중앙 = (72, 72) — 여기에 HIVE 구조물이 있다</li>
 *   <li>타일 중심 좌표: x = (열번호−1)×24 + 12, y = (행번호−1)×24 + 12</li>
 * </ul>
 *
 * <h3>⚠ 한 가지 확인 안 된 것 — 1행이 관중석 쪽인지</h3>
 * LOADING ZONE과 GARDEN의 <b>타일 이름</b>(A5/F2, A1/F6)은 Event Field Setup Guide
 * 8.3·8.4에 글로 적혀 있어 확실하다. 하지만 1행이 관중석 쪽인지 먼 쪽인지는
 * 그림(Figure 6-2)에만 있어서 읽지 못했다. 위 그림은 <b>1행 = 관중석 쪽</b>으로 가정한 것이다.
 * <p>
 * 실제 필드에서 RED LOADING ZONE 테이프가 관중석 쪽에 가깝다면 가정이 뒤집힌 것이다.
 * 그때는 RED/BLUE 파일의 <b>모든 y값을 {@code 144 − y}로</b> 바꾸면 된다
 * (x와 방향은 그대로). 필드가 180° 회전 대칭이라 그 변환만으로 맞는다.
 */
@Config
public abstract class BioBuzzAutoBase extends OpMode {

    // ────────────────────────────────────────────────
    //  공통 조정값 — FtcDashboard에서 실시간으로 바꿀 수 있다
    // ────────────────────────────────────────────────

    /**
     * 이 시각(AUTO 시작 후 초)이 지나면 앞으로 가던 걸 포기하고 바로 파킹하러 간다.
     * AUTO는 30초다. 파킹 이동 시간을 재서 30 − (이동시간 + 여유 2초) 정도로 맞출 것.
     */
    public static double PARK_DEADLINE_S = 25.0;

    /**
     * 한 경로가 이 시간(초) 안에 안 끝나면 포기하고 다음 단계로 넘어간다.
     * 로봇이 어딘가에 걸려 멈췄을 때 AUTO 전체가 거기서 끝나버리는 걸 막는다.
     */
    public static double PATH_TIMEOUT_S = 6.0;

    // ────────────────────────────────────────────────
    //  RED / BLUE 파일이 채워야 하는 것
    // ────────────────────────────────────────────────

    /** 텔레메트리에 띄울 진영 이름 */
    protected abstract String allianceName();

    /** 출발 위치 — 관중 반대편(먼 쪽) */
    protected abstract Pose startFarPose();

    /** 출발 위치 — 관중석 쪽(가까운 쪽) */
    protected abstract Pose startNearPose();

    /** 앞으로 나갔다가 돌아올 지점. 벽에서 확실히 떨어지게만 하면 된다. */
    protected abstract Pose forwardPose();

    /** 파킹 위치 (LOADING ZONE) */
    protected abstract Pose parkPose();

    /** BLUE/RED 파일에서 좌표를 적을 때 쓰는 도우미. 각도는 도(degree)로 받는다. */
    protected static Pose pose(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    // ────────────────────────────────────────────────
    //  진행 상태
    // ────────────────────────────────────────────────

    public enum StartSpot { FAR, NEAR }

    private enum Step {
        DRIVE_FORWARD,     // ① 앞으로 나가는 중 (벽에서 떨어진다)
        DRIVE_TO_PARK,     // ② 돌아와서 파킹하러 이동 중
        DONE               // 끝. 제자리 유지
    }

    protected Follower follower;
    private MultipleTelemetry multipleTelemetry;

    private StartSpot startSpot = StartSpot.FAR;
    private Step step = Step.DRIVE_FORWARD;

    /** AUTO 시작부터 흐른 시간 */
    private final ElapsedTime autoTimer = new ElapsedTime();
    /** 지금 단계를 시작하고 흐른 시간 */
    protected final ElapsedTime stepTimer = new ElapsedTime();

    // ────────────────────────────────────────────────
    //  OpMode 흐름
    // ────────────────────────────────────────────────

    @Override
    public void init() {
        follower = PedroConstants.createFollower(hardwareMap);
        multipleTelemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /** INIT 상태에서 출발 위치를 고른다. 같은 편 로봇과 겹치지 않게 미리 정할 것. */
    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) {
            startSpot = StartSpot.FAR;
        } else if (gamepad1.dpad_down) {
            startSpot = StartSpot.NEAR;
        }

        Pose start = selectedStartPose();
        telemetry.addLine(">> dpad ↑: 먼 쪽 출발   dpad ↓: 관중석 쪽 출발");
        telemetry.addData("진영", allianceName());
        telemetry.addData("출발 위치", "%s  (%.1f, %.1f, %.0f°)",
                startSpot, start.x(), start.y(), Math.toDegrees(start.heading()));
        telemetry.addLine("로봇을 이 좌표에 정확히 놓았는지 확인할 것");
        telemetry.update();
    }

    @Override
    public void start() {
        autoTimer.reset();

        // 로봇이 지금 어디 있는지 Pedro에게 알려준다. 이게 틀리면 모든 경로가 밀린다.
        follower.setPose(selectedStartPose());

        goTo(forwardPose());
        step = Step.DRIVE_FORWARD;
    }

    @Override
    public void loop() {
        // 매 루프 반드시 한 번. 위치를 갱신하고 모터 출력을 계산한다.
        follower.update();

        // 시간이 다 돼 가면 앞으로 가던 걸 포기하고 바로 파킹 (파킹 5점을 놓치지 않도록)
        if (autoTimer.seconds() > PARK_DEADLINE_S && step == Step.DRIVE_FORWARD) {
            goTo(parkPose());
            step = Step.DRIVE_TO_PARK;
        }

        switch (step) {
            case DRIVE_FORWARD:
                if (pathFinished()) {
                    // 도착했으면 바로 돌아서 파킹하러 간다.
                    // (여기가 나중에 발사 동작을 끼워 넣을 자리다)
                    goTo(parkPose());
                    step = Step.DRIVE_TO_PARK;
                }
                break;

            case DRIVE_TO_PARK:
                if (pathFinished()) {
                    follower.hold(parkPose());
                    step = Step.DONE;
                }
                break;

            case DONE:
                break;
        }

        // 수동조작이 이어서 쓸 수 있게 매 루프 위치를 저장해 둔다
        PoseStorage.lastPose = follower.pose();

        showData();
    }

    @Override
    public void stop() {
        PoseStorage.lastPose = follower.pose();
        follower.stop();
        super.stop();
    }

    // ────────────────────────────────────────────────
    //  이동 도우미
    // ────────────────────────────────────────────────

    /**
     * 지금 위치에서 {@code target}까지 직선으로 간다.
     * 가는 동안 로봇 방향은 출발 방향 → 도착 방향으로 서서히 바뀐다.
     */
    protected void goTo(Pose target) {
        Pose from = follower.pose();
        Path path = line(from, target).linear(from, target);
        follower.follow(path);
        stepTimer.reset();
    }

    /** 현재 경로가 끝났는가 (또는 너무 오래 걸려서 포기했는가) */
    protected boolean pathFinished() {
        return !follower.isBusy() || stepTimer.seconds() > PATH_TIMEOUT_S;
    }

    private Pose selectedStartPose() {
        return startSpot == StartSpot.FAR ? startFarPose() : startNearPose();
    }

    // ────────────────────────────────────────────────

    private void showData() {
        Pose pose = follower.pose();

        multipleTelemetry.addData("진영 / 출발", "%s / %s", allianceName(), startSpot);
        multipleTelemetry.addData("단계", step);
        multipleTelemetry.addData("AUTO 경과 (s)", "%.1f", autoTimer.seconds());
        multipleTelemetry.addLine();
        multipleTelemetry.addData("위치 X / Y (in)", "%.1f / %.1f", pose.x(), pose.y());
        multipleTelemetry.addData("방향 (deg)", "%.1f", Math.toDegrees(pose.heading()));
        multipleTelemetry.addData("경로 이동 중", follower.isBusy());
        multipleTelemetry.addData("남은 거리 (in)", "%.1f", follower.remainingDistance());
        multipleTelemetry.update();
    }
}
