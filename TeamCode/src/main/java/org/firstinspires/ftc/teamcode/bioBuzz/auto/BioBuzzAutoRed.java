package org.firstinspires.ftc.teamcode.bioBuzz.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BIOBUZZ 자율 — <b>RED</b> 진영.
 * <p>
 * 이 파일에는 RED 쪽 좌표만 있다. 하는 일의 순서는 {@link BioBuzzAutoBase}에 있다.
 * 좌표계는 BioBuzzAutoBase 맨 위 그림 참고 (원점 = 관중석 쪽 벽 × RED 벽 모서리).
 *
 * <h3>⚠ 아래 좌표는 전부 임시값이다</h3>
 * 경기장 그림/실물이 없는 상태에서 "대충 이쯤"으로 넣은 값이다.
 * 연습 필드에서 줄자로 재서 하나씩 고쳐야 한다. FtcDashboard의
 * {@code BioBuzzAutoRed} 항목에서 실시간으로 바꿔 보고, 맞는 값을 찾으면 여기 다시 적을 것
 * (Dashboard에서 바꾼 값은 앱을 재시작하면 사라진다).
 *
 * <p>RED 로봇이 지켜야 할 규칙 (G304): RED 쪽(A~C열)에 완전히 들어가 있을 것,
 * 필드 벽에 닿아 있을 것, LOADING ZONE 안이 아닐 것, POLLEN 4개와 닿아 있을 것.
 */
@Config
@Autonomous(name = "BioBuzz Auto RED", group = "BioBuzz", preselectTeleOp = "BioBuzz Driving")
public class BioBuzzAutoRed extends BioBuzzAutoBase {

    // ── 출발 위치: 먼 쪽 ── [측정 필요]
    // X = 로봇 길이의 절반 (18인치 로봇이 RED 벽에 붙어 있으면 9).
    // 방향 0° = 필드 중앙(BLUE 벽 쪽)을 바라봄.
    public static double START_FAR_X = 9.0;
    public static double START_FAR_Y = 108.0;
    public static double START_FAR_DEG = 0.0;

    // ── 출발 위치: 관중석 쪽 ── [측정 필요]
    public static double START_NEAR_X = 9.0;
    public static double START_NEAR_Y = 36.0;
    public static double START_NEAR_DEG = 0.0;

    // ── 발사 위치 ── [측정 필요]
    // RED HIVE의 "위를 향한 CELL"에 공이 들어가는 자리와 방향.
    // 발사 장치를 만든 뒤 실제로 쏴 보면서 맞출 것.
    public static double LAUNCH_X = 48.0;
    public static double LAUNCH_Y = 72.0;
    public static double LAUNCH_DEG = 0.0;

    // ── 파킹 위치 (RED LOADING ZONE) ── [측정 필요]
    // PARK 5점: 로봇 일부라도 LOADING ZONE 안. LEAVE 3점: 벽에 닿아 있지 않을 것.
    // 둘 다 받으려면 존 안에 걸치되 벽에서 살짝 떨어뜨린다.
    // (LOADING ZONE 깊이 약 11인치. 18인치 로봇이면 X ≈ 11~12가 적당)
    public static double PARK_X = 12.0;
    public static double PARK_Y = 72.0;
    public static double PARK_DEG = 0.0;

    // ────────────────────────────────────────────────

    @Override
    protected String allianceName() {
        return "RED";
    }

    @Override
    protected Pose startFarPose() {
        return pose(START_FAR_X, START_FAR_Y, START_FAR_DEG);
    }

    @Override
    protected Pose startNearPose() {
        return pose(START_NEAR_X, START_NEAR_Y, START_NEAR_DEG);
    }

    @Override
    protected Pose launchPose() {
        return pose(LAUNCH_X, LAUNCH_Y, LAUNCH_DEG);
    }

    @Override
    protected Pose parkPose() {
        return pose(PARK_X, PARK_Y, PARK_DEG);
    }
}
