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
 * <h3>AUTO 30초 동안 하는 일 (BIOBUZZ 득점 기준)</h3>
 * <pre>
 *  ① 출발 위치 → 발사 위치로 이동          LEAVE 3점 (끝날 때 벽에 안 닿아 있으면)
 *  ② 미리 실은 POLLEN 4개 발사             HIVE TIP 20점 (위를 향한 CELL에 넣어 기울이면)
 *  ③ 발사 위치 → LOADING ZONE으로 이동     PARK 5점 (끝날 때 일부라도 들어가 있으면)
 * </pre>
 * ②의 발사 동작은 아직 발사 장치가 없어서 <b>그냥 기다리기만 한다.</b>
 * 장치를 만들면 아래 "발사 장치 자리" 메소드 4개만 채우면 된다.
 *
 * <h3>좌표계 (팀 약속 — RED/BLUE 파일 모두 이 기준)</h3>
 * <pre>
 *            먼 쪽 벽 (관중 반대편)
 *        y=144 ┌──────────────────┐
 *              │  A  B  C │ D  E  F │
 *   RED 벽     │   RED 쪽 │ BLUE 쪽 │    BLUE 벽
 *   (x = 0)    │          │         │    (x = 144)
 *              │          │         │
 *          y=0 └──────────────────┘
 *            x=0   관중석 쪽 벽    x=144
 * </pre>
 * <ul>
 *   <li>원점 (0, 0) = 관중석 쪽 벽과 RED 쪽 벽이 만나는 모서리</li>
 *   <li>+x = RED 벽 → BLUE 벽 방향, +y = 관중석 → 먼 쪽 방향, 단위 inch</li>
 *   <li>방향(heading) 0° = +x를 바라봄, 반시계(위에서 볼 때)가 +</li>
 *   <li>필드 중앙 = (72, 72). RED 쪽은 x &lt; 72, BLUE 쪽은 x &gt; 72</li>
 * </ul>
 */
@Config
public abstract class BioBuzzAutoBase extends OpMode {

    // ────────────────────────────────────────────────
    //  공통 조정값 — FtcDashboard에서 실시간으로 바꿀 수 있다
    // ────────────────────────────────────────────────

    /** 발사에 쓰는 시간 (초). 발사 장치가 생기기 전까지는 그냥 이만큼 기다린다. */
    public static double LAUNCH_WAIT_S = 3.0;

    /**
     * 이 시각(AUTO 시작 후 초)이 지나면 하던 일을 버리고 바로 파킹하러 간다.
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

    /** 발사할 때 서 있을 위치와 방향 */
    protected abstract Pose launchPose();

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
        DRIVE_TO_LAUNCH,   // ① 발사 위치로 이동 중
        LAUNCHING,         // ② 발사 중
        DRIVE_TO_PARK,     // ③ 파킹하러 이동 중
        DONE               // 끝. 제자리 유지
    }

    protected Follower follower;
    private MultipleTelemetry multipleTelemetry;

    private StartSpot startSpot = StartSpot.FAR;
    private Step step = Step.DRIVE_TO_LAUNCH;

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

        goTo(launchPose());
        step = Step.DRIVE_TO_LAUNCH;
    }

    @Override
    public void loop() {
        // 매 루프 반드시 한 번. 위치를 갱신하고 모터 출력을 계산한다.
        follower.update();

        // 시간이 다 돼 가면 무조건 파킹 (파킹 5점을 놓치지 않도록)
        if (autoTimer.seconds() > PARK_DEADLINE_S
                && (step == Step.DRIVE_TO_LAUNCH || step == Step.LAUNCHING)) {
            stopLaunch();
            goTo(parkPose());
            step = Step.DRIVE_TO_PARK;
        }

        switch (step) {
            case DRIVE_TO_LAUNCH:
                if (pathFinished()) {
                    follower.hold(launchPose());   // 발사하는 동안 밀려도 제자리 유지
                    stepTimer.reset();
                    startLaunch();
                    step = Step.LAUNCHING;
                }
                break;

            case LAUNCHING:
                updateLaunch();
                if (launchFinished()) {
                    stopLaunch();
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
        stopLaunch();
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
    //  발사 장치 자리 — 아직 장치가 없어서 비어 있다
    //  장치를 만들면 여기 4개만 채우면 된다. (RED/BLUE는 안 건드려도 됨)
    // ────────────────────────────────────────────────

    /** 발사 시작. 예: 슈터 휠 켜기 */
    protected void startLaunch() {
        // TODO 발사 장치를 만들면 채울 것
    }

    /** 발사 중 매 루프 호출. 예: 휠 속도가 맞으면 한 발씩 밀어넣기 */
    protected void updateLaunch() {
        // TODO
    }

    /** 발사가 끝났는가. 지금은 시간만 본다. 장치가 생기면 "4발 다 쐈는가"로 바꿀 것. */
    protected boolean launchFinished() {
        return stepTimer.seconds() >= LAUNCH_WAIT_S;
    }

    /** 발사 장치 정지. 여러 번 불려도 안전해야 한다. */
    protected void stopLaunch() {
        // TODO
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
