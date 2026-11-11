package org.firstinspires.ftc.teamcode.bioBuzz.pedro;

import com.pedropathing.math.Pose;

/**
 * 자율이 끝난 위치를 수동조작으로 넘겨주는 보관함.
 * <p>
 * OpMode가 바뀌어도 static 값은 앱이 살아 있는 동안 유지된다.
 * 자율이 끝날 때 {@link #lastPose}에 위치를 저장하고,
 * 수동조작은 시작할 때 그 위치에서 출발한다.
 * <p>
 * 로봇 컨트롤러 앱을 재시작하거나 전원을 껐다 켜면 null로 돌아간다.
 * 그때 수동조작은 (0, 0, 0)에서 시작한다.
 */
public class PoseStorage {

    /** 마지막 자율이 끝난 위치. 자율을 안 돌렸으면 null. */
    public static Pose lastPose = null;

    private PoseStorage() {
    }
}
