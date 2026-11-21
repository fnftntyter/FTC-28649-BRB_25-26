package org.firstinspires.ftc.teamcode.bioBuzz;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * 핀포인트 통신 진단. <b>모터를 전혀 쓰지 않는다</b> — 안전하게 돌려도 된다.
 * <p>
 * "error communicating with I2C device 'pinpoint'" 가 뜰 때
 * 배선 문제인지, 포드가 안 꽂힌 건지, 설정 이름 문제인지 구분해 준다.
 *
 * <h3>읽는 법</h3>
 * <table>
 *   <tr><th>상태</th><th>뜻</th></tr>
 *   <tr><td>장치 못 찾음</td><td>설정 이름이 "pinpoint"이 아니거나 장치를 안 추가했다</td></tr>
 *   <tr><td>통신 실패 (예외)</td><td>케이블/포트 문제. 허브가 아예 말을 못 건다</td></tr>
 *   <tr><td>ID가 0 또는 이상한 값</td><td>통신이 불안정하다. 케이블·포트 의심</td></tr>
 *   <tr><td>FAULT_NO_PODS_DETECTED</td><td>통신은 되는데 <b>오도메트리 포드가 안 꽂혔다</b></td></tr>
 *   <tr><td>FAULT_X_POD_NOT_DETECTED</td><td>X(전진) 포드만 안 꽂혔거나 불량</td></tr>
 *   <tr><td>FAULT_Y_POD_NOT_DETECTED</td><td>Y(횡이동) 포드만 안 꽂혔거나 불량</td></tr>
 *   <tr><td>CALIBRATING</td><td>IMU 보정 중. 로봇을 가만히 두고 기다린다</td></tr>
 *   <tr><td>READY</td><td>정상. 로봇을 손으로 밀어 X/Y가 변하는지 보면 된다</td></tr>
 * </table>
 *
 * <p>핀포인트 기본 I2C 주소는 <b>0x31</b>이다 (드라이버에서 확인).
 * 같은 버스에 0x31을 쓰는 다른 장치가 있으면 충돌한다.
 */
@TeleOp(name = "Pinpoint 진단", group = "BioBuzz")
public class PinpointDiagnostic extends LinearOpMode {

    @Override
    public void runOpMode() {
        // tryGet: 설정에 없어도 OpMode가 죽지 않게
        GoBildaPinpointDriver pinpoint =
                hardwareMap.tryGet(GoBildaPinpointDriver.class, "pinpoint");

        if (pinpoint == null) {
            telemetry.addLine("!! 설정에서 'pinpoint'을 못 찾았다");
            telemetry.addLine();
            telemetry.addLine("드라이버 허브 → Configure Robot → Control Hub → I2C");
            telemetry.addLine("장치 종류: goBILDA® Pinpoint Odometry Computer");
            telemetry.addLine("이름: pinpoint   (소문자, 오타 주의)");
            telemetry.addLine();
            telemetry.addLine("설정을 저장하고 Activate 했는지도 확인할 것.");
            telemetry.update();
            waitForStart();
            return;
        }

        // 통신이 되는지 먼저 한 번 떠본다. 안 되면 여기서 예외가 난다.
        String initError = null;
        try {
            pinpoint.update();
        } catch (Exception e) {
            initError = e.getClass().getSimpleName() + ": " + e.getMessage();
        }

        telemetry.addLine("핀포인트 진단 — 모터는 돌지 않는다. START를 누르면 계속 읽는다.");
        if (initError != null) {
            telemetry.addLine();
            telemetry.addLine("!! 첫 통신부터 실패: " + initError);
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            String error = null;
            GoBildaPinpointDriver.DeviceStatus status = null;
            int id = -1, version = -1;
            double freq = Double.NaN;
            int loopTime = -1;
            Pose2D pose = null;

            try {
                pinpoint.update();
                status = pinpoint.getDeviceStatus();
                id = pinpoint.getDeviceID();
                version = pinpoint.getDeviceVersion();
                freq = pinpoint.getFrequency();
                loopTime = pinpoint.getLoopTime();
                pose = pinpoint.getPosition();
            } catch (Exception e) {
                error = e.getClass().getSimpleName() + ": " + e.getMessage();
            }

            if (error != null) {
                telemetry.addLine(">>> 통신 실패 — 배선/포트 문제");
                telemetry.addData("예외", error);
                telemetry.addLine();
                telemetry.addLine("1) I2C 케이블 양쪽을 다시 꽂아볼 것");
                telemetry.addLine("2) 다른 4핀 케이블로 바꿔볼 것 (제일 흔한 원인)");
                telemetry.addLine("3) 다른 I2C 포트(1/2/3)로 옮기고 설정도 같이 옮길 것");
                telemetry.addLine("4) 핀포인트 LED가 켜지는지 볼 것 — 안 켜지면 전원/케이블");
            } else {
                telemetry.addData(">>> 상태", status);
                telemetry.addData("장치 ID", id);
                telemetry.addData("버전", version);
                telemetry.addData("갱신 주기(Hz)", "%.1f", freq);
                telemetry.addData("루프 시간(us)", loopTime);
                telemetry.addLine();
                if (pose != null) {
                    telemetry.addData("X (mm)", "%.1f", pose.getX(DistanceUnit.MM));
                    telemetry.addData("Y (mm)", "%.1f", pose.getY(DistanceUnit.MM));
                    telemetry.addData("방향 (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
                }
                telemetry.addLine();

                if (status == GoBildaPinpointDriver.DeviceStatus.READY) {
                    telemetry.addLine("정상. 로봇을 손으로 밀어 X/Y가 변하는지 확인할 것.");
                    telemetry.addLine("  앞으로 밀기 → X 증가 / 왼쪽으로 밀기 → Y 증가 가 맞다.");
                } else if (status == GoBildaPinpointDriver.DeviceStatus.CALIBRATING) {
                    telemetry.addLine("IMU 보정 중. 로봇을 가만히 두고 기다린다.");
                } else if (status != null && status.name().startsWith("FAULT")) {
                    telemetry.addLine("통신은 되는데 포드/IMU 쪽 문제다:");
                    telemetry.addLine("  포드 케이블이 핀포인트에 제대로 꽂혔는지 확인할 것.");
                    telemetry.addLine("  X 포트 = 전진 포드, Y 포트 = 횡이동 포드.");
                }
            }

            telemetry.update();
            sleep(100);
        }
    }
}
