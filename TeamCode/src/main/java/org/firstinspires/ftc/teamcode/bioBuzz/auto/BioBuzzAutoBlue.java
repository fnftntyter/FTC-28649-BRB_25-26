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
 * <h3>⚠ 아래 좌표는 전부 임시값이다</h3>
 * RED 값을 필드 중앙선(x = 72) 기준으로 좌우 반전해서 만든 값이다:
 * {@code X → 144 − X},  {@code 방향 → 180° − 방향}.
 * 하지만 BIOBUZZ 필드가 정말 좌우 대칭인지는 확인되지 않았다
 * (매뉴얼상 GARDEN은 "서로 반대쪽 모서리"에 있어서 회전 대칭일 수도 있다).
 * <b>RED를 맞췄다고 BLUE가 맞는 게 아니다.</b> BLUE도 따로 재서 고칠 것.
 *
 * <p>BLUE 로봇이 지켜야 할 규칙 (G304): BLUE 쪽(D~F열)에 완전히 들어가 있을 것,
 * 필드 벽에 닿아 있을 것, LOADING ZONE 안이 아닐 것, POLLEN 4개와 닿아 있을 것.
 */
@Config
@Autonomous(name = "BioBuzz Auto BLUE", group = "BioBuzz", preselectTeleOp = "BioBuzz Driving")
public class BioBuzzAutoBlue extends BioBuzzAutoBase {

    // ── 출발 위치: 먼 쪽 ── [측정 필요]
    // X = 144 − 로봇 길이의 절반 (18인치 로봇이 BLUE 벽에 붙어 있으면 135).
    // 방향 180° = 필드 중앙(RED 벽 쪽)을 바라봄.
    public static double START_FAR_X = 135.0;
    public static double START_FAR_Y = 108.0;
    public static double START_FAR_DEG = 180.0;

    // ── 출발 위치: 관중석 쪽 ── [측정 필요]
    public static double START_NEAR_X = 135.0;
    public static double START_NEAR_Y = 36.0;
    public static double START_NEAR_DEG = 180.0;

    // ── 발사 위치 ── [측정 필요]
    // BLUE HIVE의 "위를 향한 CELL"에 공이 들어가는 자리와 방향.
    public static double LAUNCH_X = 96.0;
    public static double LAUNCH_Y = 72.0;
    public static double LAUNCH_DEG = 180.0;

    // ── 파킹 위치 (BLUE LOADING ZONE) ── [측정 필요]
    // 존 안에 걸치되 BLUE 벽(x = 144)에는 닿지 않게.
    public static double PARK_X = 132.0;
    public static double PARK_Y = 72.0;
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
    protected Pose launchPose() {
        return pose(LAUNCH_X, LAUNCH_Y, LAUNCH_DEG);
    }

    @Override
    protected Pose parkPose() {
        return pose(PARK_X, PARK_Y, PARK_DEG);
    }
}
