package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * 미션(게임 룰)에 의존하는 값만 모아둔 클래스.
 * <p>
 * 대회 룰은 매년 바뀌므로, 룰이 바뀌면 <b>원칙적으로 이 파일만</b> 수정한다.
 * 로봇 제어 로직(ShooterBase / MecanumBase / CameraBase)에는
 * 태그 번호나 필드 좌표를 절대 직접 쓰지 않는다.
 * <p>
 * 모든 값은 FtcDashboard에서 실시간으로 바꿀 수 있다.
 * 대회장에서 값을 맞춘 뒤에는 반드시 이 파일에 다시 적어 넣을 것
 * (Dashboard 수정값은 앱을 재시작하면 사라진다).
 */
@Config
public class GameConfig {

    /** 진영. RED 경로는 BLUE 경로를 X축 기준으로 대칭시켜 만든다. */
    public enum Alliance {
        BLUE, RED;

        /** BLUE는 +1, RED는 -1. y좌표와 각도에 곱해 좌우 대칭을 만든다. */
        public double mirror() {
            return this == BLUE ? 1.0 : -1.0;
        }
    }

    // ────────────────────────────────────────────────
    //  AprilTag ID
    // ────────────────────────────────────────────────

    /** 우리 진영이 득점하는 골에 붙은 태그 */
    public static int TAG_GOAL_BLUE = 20;
    public static int TAG_GOAL_RED = 24;

    /**
     * 자율 시작 시 읽는 "패턴/모티프" 태그 후보.
     * 어느 하나가 보이면 그 ID가 이번 판의 패턴이 된다.
     * 룰에 이런 요소가 없으면 빈 배열로 두면 된다.
     */
    public static int[] TAG_MOTIF_CANDIDATES = { 21, 22, 23 };

    /** 진영에 해당하는 골 태그 ID */
    public static int goalTag(Alliance alliance) {
        return alliance == Alliance.BLUE ? TAG_GOAL_BLUE : TAG_GOAL_RED;
    }

    // ────────────────────────────────────────────────
    //  득점 목표의 물리 제원 (슈터 조준 계산에 사용)
    // ────────────────────────────────────────────────

    /** 슈터 발사구 높이 → 골 입구 높이의 차이 (inch). 줄자로 실측할 것. */
    public static double GOAL_HEIGHT_DIFF_IN = 30.098425;

    /** 고정 발사각 (도). 슈터 각도를 바꾸면 여기도 바꿀 것. */
    public static double LAUNCH_ANGLE_DEG = 50.0;

    // ────────────────────────────────────────────────
    //  자율주행 — 시작 위치
    // ────────────────────────────────────────────────

    public static double START_X = 64.45;
    /** BLUE 기준. RED는 자동으로 부호가 뒤집힌다. */
    public static double START_Y = 12.0;
    /** BLUE 기준 (도). RED는 자동으로 부호가 뒤집힌다. */
    public static double START_HEADING_DEG = 180.0;

    // ────────────────────────────────────────────────
    //  자율주행 — 시퀀스
    // ────────────────────────────────────────────────

    /** 시작 직후 골 쪽으로 대략 조준하는 제자리 회전량 (도, BLUE 기준 CCW 양수) */
    public static double AIM_TURN_DEG = 20.0;

    /** 자율에서 발사할 개수 */
    public static int AUTO_SHOT_COUNT = 3;

    /** 발사 전 카메라로 미세 조준할 시간 (초). 0이면 조준 생략. */
    public static double AUTO_AIM_TIMEOUT_S = 1.5;

    /** 슈터 휠이 목표 속도에 도달할 때까지 최대 대기 (초) */
    public static double AUTO_SPINUP_TIMEOUT_S = 2.5;

    // ────────────────────────────────────────────────
    //  자율주행 — 파킹
    // ────────────────────────────────────────────────

    public static boolean AUTO_DO_PARK = false;
    public static double PARK_X = 64.45;
    /** BLUE 기준. RED는 자동으로 부호가 뒤집힌다. */
    public static double PARK_Y = 64.45;
    /** 파킹 지점에서의 최종 로봇 방향 (도, BLUE 기준) */
    public static double PARK_HEADING_DEG = 180.0;

    // ────────────────────────────────────────────────
    //  헬퍼 — 진영에 맞춰 좌표를 대칭시켜 돌려준다
    // ────────────────────────────────────────────────

    public static Pose2d startPose(Alliance alliance) {
        double m = alliance.mirror();
        return new Pose2d(START_X, START_Y * m, Math.toRadians(START_HEADING_DEG * m));
    }

    public static Vector2d parkPosition(Alliance alliance) {
        return new Vector2d(PARK_X, PARK_Y * alliance.mirror());
    }

    public static double parkHeadingRad(Alliance alliance) {
        return Math.toRadians(PARK_HEADING_DEG * alliance.mirror());
    }

    /** 시작 직후 조준 회전량 (라디안). Road Runner의 turn()은 라디안을 받는다. */
    public static double aimTurnRad(Alliance alliance) {
        return Math.toRadians(AIM_TURN_DEG * alliance.mirror());
    }
}
