package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

@Config
public class MecanumBase {

    /** 헤딩 유지 PD 제어 게인 */
    public static double KP = 2;
    public static double KD = 0.2;

    /** 일반 주행 속도 배율 */
    public static double SPEED_NORMAL = 0.7;
    /** 범퍼를 눌렀을 때의 정밀 주행 배율 */
    public static double SPEED_SLOW = 0.3;

    /** 자동 조준이 "맞았다"고 인정하는 오차 (rad) */
    public static double AIM_TOLERANCE_RAD = 0.03;

    public double lastHeading, currentHeading, targetHeading, postHeading;

    private final ElapsedTime time = new ElapsedTime();
    private double lastTime = 0;
    private double speedMultiplier = SPEED_NORMAL;

    private boolean isFieldCentric = true;
    private boolean isAutoAiming = false;

    private final MecanumDrive drive;
    private final CameraBase camera;

    public MecanumBase(HardwareMap hardwareMap, Pose2d pose2d) {
        camera = CameraBase.getInstance(hardwareMap);
        drive = new MecanumDrive(hardwareMap, pose2d);

        lastHeading = currentHeading = targetHeading = postHeading =
                drive.localizer.getPose().heading.log();
    }

    /**
     * gamepad1 기준 조작:
     * <ul>
     *   <li>좌스틱 — 이동 / 우스틱 — 회전</li>
     *   <li>A — 필드중심 ↔ 로봇중심 전환</li>
     *   <li>B — 자동 조준 on/off (우리 진영 골 태그를 계속 추적)</li>
     *   <li>dpad — 헤딩을 0/90/180/270°로 스냅 (스냅 중에도 이동은 가능)</li>
     *   <li>범퍼 — 정밀 저속</li>
     *   <li>back — 지금 방향을 필드중심의 새 기준(0°)으로 재설정</li>
     * </ul>
     *
     * @param alliance 우리 진영. 자동 조준이 볼 태그를 결정한다.
     */
    public void update(Gamepad gamepad, GameConfig.Alliance alliance) {
        drive.updatePoseEstimate();
        currentHeading = drive.localizer.getPose().heading.log();

        if (gamepad.aWasPressed()) {
            isFieldCentric = !isFieldCentric;
        }

        // 필드중심 기준 방향 재설정 — 짐벌이 틀어졌을 때 대회장에서 바로 잡는다.
        if (gamepad.backWasPressed()) {
            postHeading = currentHeading;
            targetHeading = currentHeading;
        }

        if (gamepad.bWasPressed()) {
            isAutoAiming = !isAutoAiming;
        }

        // 자동 조준: 태그가 보이는 동안 매 루프 목표 헤딩을 갱신한다.
        // (예전 코드는 B를 누른 그 순간에만 한 번 갱신해서, 로봇이 움직이면 바로 틀어졌다.)
        if (isAutoAiming) {
            int goalTag = GameConfig.goalTag(alliance);
            if (camera.isTagVisible(goalTag)) {
                targetHeading = currentHeading + camera.getAimErrorRad(goalTag);
            }
        }

        // 헤딩 스냅. 예전 코드는 여기서 move()를 호출하지 않아
        // dpad를 누르는 동안 로봇이 아예 멈춰 있었다.
        if (gamepad.dpad_up) {
            targetHeading = postHeading;
        } else if (gamepad.dpad_left) {
            targetHeading = postHeading + Math.toRadians(90);
        } else if (gamepad.dpad_down) {
            targetHeading = postHeading + Math.toRadians(180);
        } else if (gamepad.dpad_right) {
            targetHeading = postHeading + Math.toRadians(270);
        }

        speedMultiplier = (gamepad.right_bumper || gamepad.left_bumper) ? SPEED_SLOW : SPEED_NORMAL;

        move(-gamepad.left_stick_x, -gamepad.left_stick_y, -gamepad.right_stick_x);
    }

    /**
     * @param x  좌우 (왼쪽이 양수)
     * @param y  전후 (앞이 양수)
     * @param rx 회전 (CCW가 양수). 0이면 헤딩 유지 PD가 작동한다.
     */
    public void move(double x, double y, double rx) {
        double xPower, yPower, rotationPower;

        double now = time.seconds();
        double dt = now - lastTime;
        lastTime = now;

        double derivative = 0;
        if (dt > 0) {
            derivative = normalizeAngle(currentHeading - lastHeading) / dt;
        }
        lastHeading = currentHeading;

        if (rx == 0) {
            // 조이스틱을 완전히 놨을 때만 헤딩 유지 PD가 작동
            double headingError = normalizeAngle(targetHeading - currentHeading);
            rotationPower = Range.clip((KP * headingError) - (KD * derivative), -1.0, 1.0);
        } else {
            // 수동 회전 중에는 현재 방향을 새 목표로 삼는다
            rotationPower = rx;
            targetHeading = currentHeading;
            isAutoAiming = false; // 손으로 돌리면 자동 조준 해제
        }

        if (isFieldCentric) {
            // 조이스틱 입력(필드 기준)을 로봇 기준으로 회전 변환
            double sin = Math.sin(-currentHeading);
            double cos = Math.cos(-currentHeading);
            yPower = y * cos - x * sin;
            xPower = y * sin + x * cos;
        } else {
            yPower = y;
            xPower = x;
        }

        Vector2d input = new Vector2d(yPower, xPower);

        drive.setDrivePowers(new PoseVelocity2d(
                input.times(speedMultiplier),
                rotationPower * speedMultiplier));
    }

    /** 제자리에서 헤딩만 유지 (자율에서 조준 중 사용) */
    public void holdHeading() {
        move(0, 0, 0);
    }

    /** 네 바퀴 출력을 완전히 0으로. 자율 구간 사이에서 잔여 회전을 끊는다. */
    public void stopMotors() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    // ────────────────────────────────────────────────
    //  자율주행에서 쓰는 조준 헬퍼
    // ────────────────────────────────────────────────

    /**
     * 태그를 향해 목표 헤딩을 갱신한다. 실제 회전은 {@link #holdHeading()}이 수행한다.
     *
     * @return 태그가 보였으면 true
     */
    public boolean aimAt(int tagId) {
        drive.updatePoseEstimate();
        currentHeading = drive.localizer.getPose().heading.log();

        if (!camera.isTagVisible(tagId)) {
            return false;
        }
        targetHeading = currentHeading + camera.getAimErrorRad(tagId);
        return true;
    }

    /** 목표 헤딩에 충분히 가까운가 */
    public boolean isAimed() {
        return Math.abs(normalizeAngle(targetHeading - currentHeading)) < AIM_TOLERANCE_RAD;
    }

    /** 현재 헤딩을 목표로 고정해 잔여 회전을 멈춘다. */
    public void freezeHeading() {
        drive.updatePoseEstimate();
        currentHeading = drive.localizer.getPose().heading.log();
        targetHeading = currentHeading;
    }

    public MecanumDrive getRoadRunnerDrive() {
        return drive;
    }

    public boolean isFieldCentric() {
        return isFieldCentric;
    }

    public boolean isAutoAiming() {
        return isAutoAiming;
    }

    public double getHeadingErrorRad() {
        return normalizeAngle(targetHeading - currentHeading);
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
