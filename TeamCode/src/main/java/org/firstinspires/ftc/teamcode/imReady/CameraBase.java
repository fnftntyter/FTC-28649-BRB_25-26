package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * AprilTag 조회/조준 계산.
 * <p>
 * <b>SDK 12.0 주의.</b> {@code AprilTagDetection}이 추상 클래스가 되고
 * {@code AprilTagSingleDetection}(태그 1개)과 {@code AprilTagClusterDetection}(BIOBUZZ의
 * 태그 묶음)으로 갈라졌다. {@code id}/{@code metadata}/{@code center}는 Single에만 있고,
 * {@code ftcPose}/{@code robotPose}는 공통이다.
 * 이 클래스는 ID로 찾는 메소드들이 Single만 보도록 걸러낸다.
 */
@Config
public class CameraBase extends CameraHardware {

    /**
     * 조준 보정각 (rad). 실제로 골 중앙에 맞을 때까지 대회장에서 조절한다.
     */
    public static double BEARING_OFFSET = 0.1;

    /**
     * 방위각 부호. 태그를 향해 <b>반대로</b> 돌면 -1로 바꾼다.
     * <p>
     * FTC SDK 규약상 {@code ftcPose.bearing}은 "태그를 정면으로 보려면 로봇이 CCW로
     * 돌아야 하는 각"이라 Road Runner 헤딩(CCW 양수)과 부호가 같다. 즉 기본값은 +1.
     * 다만 이 로봇은 카메라를 roll 180°(뒤집어서)로 달았기 때문에
     * 실측에서 반대로 나올 수 있어 토글로 빼 두었다.
     */
    public static double BEARING_SIGN = 1.0;

    private static CameraBase instance = null;

    /** 어떤 OpMode의 hardwareMap으로 만들어졌는지. OpMode가 바뀌면 카메라를 다시 만든다. */
    private final HardwareMap ownerHardwareMap;

    /** update() 전에 접근해도 NPE가 나지 않도록 빈 리스트로 시작한다. */
    public List<AprilTagDetection> currentDetections = Collections.emptyList();

    private CameraBase(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.ownerHardwareMap = hardwareMap;
    }

    /**
     * 싱글톤 인스턴스를 반환한다.
     * <p>
     * 이전 OpMode가 {@link #closeCamera()}를 부르지 못하고 끝난 경우(예: 자율 도중 STOP),
     * 죽은 hardwareMap을 물고 있는 인스턴스가 남아 다음 OpMode에서 크래시가 난다.
     * hardwareMap이 달라지면 강제로 다시 만들어 그 상황을 막는다.
     */
    public static CameraBase getInstance(HardwareMap hardwareMap) {
        if (instance != null && instance.ownerHardwareMap != hardwareMap) {
            instance.closeCamera();
        }
        if (instance == null) {
            instance = new CameraBase(hardwareMap);
        }
        return instance;
    }

    /** 매 루프 1회 호출. 이걸 부르지 않으면 태그 정보가 갱신되지 않는다. */
    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        currentDetections = (detections != null) ? detections : Collections.<AprilTagDetection>emptyList();
    }

    public void enableCameraStreaming(int maxFps) {
        FtcDashboard.getInstance().startCameraStream(visionPortal, maxFps);
    }

    // ────────────────────────────────────────────────
    //  태그 조회
    // ────────────────────────────────────────────────

    /** 해당 ID의 태그. 안 보이면 null. 클러스터 검출은 ID가 없으므로 건너뛴다. */
    public AprilTagSingleDetection getDetectionById(int id) {
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection
                    && ((AprilTagSingleDetection) detection).id == id) {
                return (AprilTagSingleDetection) detection;
            }
        }
        return null;
    }

    public boolean isTagVisible(int id) {
        return getDetectionById(id) != null;
    }

    /** 후보 ID 중 가장 가까운(= 가장 신뢰할 만한) 태그. 하나도 안 보이면 null. */
    public AprilTagSingleDetection getNearestOf(int... ids) {
        AprilTagSingleDetection best = null;
        for (int id : ids) {
            AprilTagSingleDetection d = getDetectionById(id);
            if (d == null || d.ftcPose == null) continue;
            if (best == null || d.ftcPose.range < best.ftcPose.range) {
                best = d;
            }
        }
        return best;
    }

    /**
     * 이번 판의 패턴/모티프 태그 ID.
     * {@link GameConfig#TAG_MOTIF_CANDIDATES} 중 보이는 것을 돌려주고,
     * 하나도 없으면 -1을 반환한다.
     */
    public int getMotifTagId() {
        AprilTagSingleDetection d = getNearestOf(GameConfig.TAG_MOTIF_CANDIDATES);
        return (d != null) ? d.id : -1;
    }

    /** 지금 보이는 모든 태그 ID (텔레메트리용). 클러스터는 ID가 없어 제외된다. */
    public List<Integer> getVisibleTagIds() {
        List<Integer> ids = new ArrayList<>();
        for (AprilTagDetection detection : currentDetections) {
            if (detection instanceof AprilTagSingleDetection) {
                ids.add(((AprilTagSingleDetection) detection).id);
            }
        }
        return ids;
    }

    // ────────────────────────────────────────────────
    //  조준 계산 — 부호 규약을 여기 한 곳에만 둔다
    // ────────────────────────────────────────────────

    /**
     * 태그를 정면으로 보기 위해 로봇이 추가로 돌아야 하는 각 (rad, CCW 양수).
     * <p>
     * <b>이 메소드가 방위각 부호의 유일한 출처다.</b> 다른 파일에서
     * {@code ftcPose.bearing}을 직접 쓰지 말 것. (예전 코드가 파일마다
     * {@code +bearing} / {@code -bearing}을 섞어 써서 한쪽이 반대로 돌았다.)
     *
     * @return 보정까지 반영된 회전량. 태그가 안 보이면 0.
     */
    public double getAimErrorRad(int id) {
        AprilTagDetection d = getDetectionById(id);
        return (d != null) ? getAimErrorRad(d) : 0.0;
    }

    public double getAimErrorRad(AprilTagDetection detection) {
        if (detection == null || detection.ftcPose == null) return 0.0;
        return (BEARING_SIGN * detection.ftcPose.bearing) + BEARING_OFFSET;
    }

    /** 카메라에서 태그까지의 수평 거리 (inch). 안 보이면 0. */
    public double getDistanceByTag(AprilTagDetection detection) {
        if (detection == null || detection.ftcPose == null) return 0.0;
        return detection.ftcPose.y;
    }

    public double getDistanceById(int id) {
        return getDistanceByTag(getDetectionById(id));
    }

    /** 태그 기반 로봇 필드 좌표. 안 보이면 null. */
    public Position getRobotPoseFromTag(int id) {
        AprilTagDetection detection = getDetectionById(id);
        if (detection != null && detection.robotPose != null) {
            return detection.robotPose.getPosition();
        }
        return null;
    }

    // ────────────────────────────────────────────────

    /** 여러 번 불러도 안전하다. */
    public void closeCamera() {
        try {
            dashboard.stopCameraStream();
            if (visionPortal != null) {
                visionPortal.close();
            }
        } catch (Exception ignored) {
            // 이미 닫혀 있으면 무시 — OpMode 종료 경로를 막지 않는다.
        }
        currentDetections = Collections.emptyList();
        if (instance == this) {
            instance = null;
        }
    }
}
