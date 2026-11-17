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
 * <h3>값의 근거 — Event Field Setup Guide (V1.0, 2026-09-12)</h3>
 * 공식 문서에서 확인한 사실:
 * <ul>
 *   <li>8.3 — RED LOADING ZONE은 <b>타일 A5</b>, 벽에서 11인치 깊이</li>
 *   <li>8.4 — RED GARDEN은 <b>타일 A1</b></li>
 *   <li>8.5 — ALLIANCE AREA는 Tile Seam 1~5 (= 타일 2~5칸) 폭</li>
 *   <li>매뉴얼 G402 — A·B·C열이 RED 쪽</li>
 * </ul>
 * 그래서 좌표는 "대충"이 아니라 타일 격자에서 계산한 값이다.
 * 다만 <b>로봇 크기를 18인치로 가정</b>했으므로(벽에 붙으면 중심이 벽에서 9인치),
 * 로봇이 더 작으면 출발 X와 파킹 X를 줄여야 한다.
 *
 * <p>RED 로봇이 지켜야 할 규칙 (G304): RED 쪽(A~C열)에 완전히 들어가 있을 것,
 * 필드 벽에 닿아 있을 것, LOADING ZONE(A5) 안이 아닐 것, POLLEN 4개와 닿아 있을 것.
 */
@Config
@Autonomous(name = "BioBuzz Auto RED", group = "BioBuzz", preselectTeleOp = "BioBuzz Driving")
public class BioBuzzAutoRed extends BioBuzzAutoBase {

    // ── 출발 위치: 먼 쪽 (타일 A6) ──
    // RED 벽(x=0)에 붙어서 선다. 18인치 로봇이면 중심이 벽에서 9인치.
    // A6 = x 0~24, y 120~144 → 중심 y = 132.
    // LOADING ZONE(A5) 바로 옆이라 파킹까지 거리가 가장 짧다.
    // 방향 0° = 필드 중앙(BLUE 벽 쪽)을 바라봄.
    public static double START_FAR_X = 9.0;
    public static double START_FAR_Y = 132.0;
    public static double START_FAR_DEG = 0.0;

    // ── 출발 위치: 관중석 쪽 (타일 A2) ──
    // A2 = x 0~24, y 24~48 → 중심 y = 36. GARDEN(A1) 바로 옆이지만 테이프라 상관없다.
    public static double START_NEAR_X = 9.0;
    public static double START_NEAR_Y = 36.0;
    public static double START_NEAR_DEG = 0.0;

    // ── 앞으로 나갔다 돌아올 지점 ──
    // 파킹 지점에서 필드 안쪽으로 한 타일 정도 나간 자리.
    // 벽에서 떨어지기만 하면 되니 멀리 갈 필요 없다 (돌아올 시간만 더 든다).
    public static double FORWARD_X = 36.0;
    public static double FORWARD_Y = 108.0;
    public static double FORWARD_DEG = 0.0;

    // ── 파킹 위치 (RED LOADING ZONE = 타일 A5) ──
    // 존은 x 0~11, y 약 96.5~119.5 (타일 A5 안에 23×11인치).
    //
    // LEAVE 3점 + PARK 5점을 둘 다 받는 조건 (18인치 로봇, 반폭 9인치 기준):
    //   벽에 안 닿기  → 중심 X > 9
    //   존에 걸치기   → 중심 X − 9 < 11  즉 중심 X < 20
    // X = 13이면 앞 끝이 벽에서 4인치 떨어지고 존 안으로 7인치 들어간다. 여유 있는 값.
    public static double PARK_X = 13.0;
    public static double PARK_Y = 108.0;
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
    protected Pose forwardPose() {
        return pose(FORWARD_X, FORWARD_Y, FORWARD_DEG);
    }

    @Override
    protected Pose parkPose() {
        return pose(PARK_X, PARK_Y, PARK_DEG);
    }
}
