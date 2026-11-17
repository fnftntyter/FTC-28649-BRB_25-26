package org.firstinspires.ftc.teamcode.bioBuzz.auto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BIOBUZZ 자율 — <b>BLUE</b> 진영.
 * <p>
 * 이 파일에는 BLUE 쪽 좌표만 있다. 하는 일의 순서는 {@link BioBuzzAutoBase}에 있다.
 * 좌표계는 RED와 <b>같은 기준</b>이다 (원점 = 관중석 쪽 벽 × RED 벽 모서리).
 * 그래서 BLUE 벽은 x = 144이고, BLUE 로봇은 x가 큰 쪽에서 출발한다.
 *
 * <h3>BIOBUZZ 필드는 좌우 대칭이 아니라 180° 회전 대칭이다</h3>
 * Event Field Setup Guide(V1.0)에서 확인:
 * <ul>
 *   <li>8.3 — LOADING ZONE: RED는 <b>타일 A5</b>, BLUE는 <b>타일 F2</b></li>
 *   <li>8.4 — GARDEN: RED는 <b>타일 A1</b>, BLUE는 <b>타일 F6</b></li>
 * </ul>
 * A↔F, 5↔2, 1↔6 — 전부 필드 중심을 기준으로 180° 돌린 관계다.
 * 좌우 반전(x만 뒤집기)으로 만들면 <b>틀린다.</b> 변환은 이렇게 된다:
 * <pre>
 *   x → 144 − x,   y → 144 − y,   방향 → 방향 + 180°
 * </pre>
 * 단, 출발 위치만은 회전으로 만들지 않고 BLUE 쪽에서 쓸 수 있는 타일을 직접 골랐다
 * (회전시키면 RED의 "먼 쪽"이 BLUE의 "관중석 쪽"이 되어 버려서 헷갈린다).
 *
 * <p>BLUE 로봇이 지켜야 할 규칙 (G304): BLUE 쪽(D~F열)에 완전히 들어가 있을 것,
 * 필드 벽에 닿아 있을 것, LOADING ZONE(F2) 안이 아닐 것, POLLEN 4개와 닿아 있을 것.
 */
@Config
@Autonomous(name = "BioBuzz Auto BLUE", group = "BioBuzz", preselectTeleOp = "BioBuzz Driving")
public class BioBuzzAutoBlue extends BioBuzzAutoBase {

    // ── 출발 위치: 먼 쪽 (타일 F5) ──
    // BLUE 벽(x=144)에 붙어서 선다. 18인치 로봇이면 중심이 벽에서 9인치 → x = 135.
    // F5 = x 120~144, y 96~120 → 중심 y = 108.
    // 방향 180° = 필드 중앙(RED 벽 쪽)을 바라봄.
    public static double START_FAR_X = 135.0;
    public static double START_FAR_Y = 108.0;
    public static double START_FAR_DEG = 180.0;

    // ── 출발 위치: 관중석 쪽 (타일 F1) ──
    // F1 = x 120~144, y 0~24 → 중심 y = 12. LOADING ZONE(F2) 바로 옆이라 파킹이 가깝다.
    public static double START_NEAR_X = 135.0;
    public static double START_NEAR_Y = 12.0;
    public static double START_NEAR_DEG = 180.0;

    // ── 앞으로 나갔다 돌아올 지점 ──
    // 파킹 지점에서 필드 안쪽으로 한 타일 정도. (RED의 (36,108)을 180° 돌린 자리)
    public static double FORWARD_X = 108.0;
    public static double FORWARD_Y = 36.0;
    public static double FORWARD_DEG = 180.0;

    // ── 파킹 위치 (BLUE LOADING ZONE = 타일 F2) ──
    // 존은 x 133~144, y 약 24.5~47.5 (타일 F2 안에 23×11인치).
    //
    // LEAVE 3점 + PARK 5점을 둘 다 받는 조건 (18인치 로봇, 반폭 9인치 기준):
    //   벽에 안 닿기  → 중심 X < 135
    //   존에 걸치기   → 중심 X + 9 > 133  즉 중심 X > 124
    // X = 131이면 끝이 벽에서 4인치 떨어지고 존 안으로 7인치 들어간다. (RED X=13의 180° 대응)
    public static double PARK_X = 131.0;
    public static double PARK_Y = 36.0;
    public static double PARK_DEG = 180.0;

    // ────────────────────────────────────────────────

    @Override
    protected String allianceName() {
        return "BLUE";
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
