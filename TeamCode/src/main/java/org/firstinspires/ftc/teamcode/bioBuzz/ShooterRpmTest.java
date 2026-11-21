package org.firstinspires.ftc.teamcode.bioBuzz;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * 슈터의 <b>실제</b> 최대 RPM을 재는 OpMode.
 * <p>
 * "목표 RPM을 낮춰도 실제 속도가 안 변한다"는 증상은 보통 명령 속도가
 * 모터 능력을 넘어서 100% 출력으로 포화된 상태다. 그걸 확인하고
 * {@link BioBuzzMechanism#SHOOTER_MAX_RPM} 에 넣을 참값을 찾는다.
 *
 * <h3>왜 출력 제어로 재는가</h3>
 * {@code RUN_WITHOUT_ENCODER}로 <b>출력(%)을 직접</b> 주고 엔코더로 속도만 읽는다.
 * PIDF가 끼어들지 않으니 "이 모터가 이 출력에서 몇 RPM을 내는가"가 그대로 나온다.
 * ({@code getVelocity()}는 런모드와 무관하게 엔코더 속도를 돌려준다.)
 *
 * <h3>쓰는 법</h3>
 * <ol>
 *   <li>슈터를 돌려도 안전한 상태로 만든다 (공 빼고, 사람 비키고)</li>
 *   <li>START</li>
 *   <li>dpad ↑↓ 로 출력을 0.05씩 올린다. <b>1.0까지 올려 속도가 안정될 때까지 기다린다</b></li>
 *   <li>그때의 "실제 RPM"이 이 모터의 최대 RPM이다 → {@code SHOOTER_MAX_RPM} 에 적는다</li>
 *   <li>{@code SHOOTER_TARGET_RPM} 은 그 값의 60~80% 정도로 잡는다</li>
 * </ol>
 * B를 누르면 즉시 출력 0. 화면의 "엔코더 살아있나"가 아니오면 엔코더 케이블부터 확인할 것.
 *
 * <pre>
 *  dpad ↑ / ↓   출력 +0.05 / −0.05
 *  dpad → / ←   출력 +0.01 / −0.01
 *  B            즉시 정지 (출력 0)
 *  A            최고 기록 초기화
 * </pre>
 */
@TeleOp(name = "Shooter RPM 측정", group = "BioBuzz")
public class ShooterRpmTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.tryGet(DcMotorEx.class, BioBuzzMechanism.NAME_SHOOTER);

        MultipleTelemetry tel = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());

        if (shooter == null) {
            tel.addLine("!! 설정에서 '" + BioBuzzMechanism.NAME_SHOOTER + "'을 못 찾았다");
            tel.update();
            waitForStart();
            return;
        }

        // 설정에 적힌 모터 사양 — 이게 실제 모터와 맞는지가 핵심이다.
        double ticksPerRev = shooter.getMotorType().getTicksPerRev();
        double specMaxRpm = shooter.getMotorType().getMaxRPM();
        String motorName = shooter.getMotorType().getName();

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        tel.addLine("슈터를 돌려도 안전한지 확인하고 START");
        tel.addData("설정된 모터", motorName);
        tel.addData("ticks/rev", "%.1f", ticksPerRev);
        tel.addData("사양상 최대 RPM", "%.0f", specMaxRpm);
        tel.update();

        waitForStart();

        double power = 0.0;
        double maxSeenRpm = 0.0;

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed())    power += 0.05;
            if (gamepad1.dpadDownWasPressed())  power -= 0.05;
            if (gamepad1.dpadRightWasPressed()) power += 0.01;
            if (gamepad1.dpadLeftWasPressed())  power -= 0.01;
            if (gamepad1.b)                     power = 0.0;
            if (gamepad1.aWasPressed())         maxSeenRpm = 0.0;

            power = Math.max(0.0, Math.min(1.0, power));
            shooter.setPower(power);

            double ticksPerSec = shooter.getVelocity();
            double rpm = ticksPerRev > 0 ? ticksPerSec * 60.0 / ticksPerRev : 0.0;
            if (rpm > maxSeenRpm) {
                maxSeenRpm = rpm;
            }

            tel.addData("출력", "%.2f", power);
            tel.addData(">>> 실제 RPM", "%.0f", rpm);
            tel.addData("최고 기록 RPM", "%.0f", maxSeenRpm);
            tel.addLine();
            tel.addData("엔코더 살아있나", (power > 0.1 && Math.abs(ticksPerSec) < 1.0)
                    ? "아니오 — 엔코더 케이블 확인!" : "예");
            tel.addData("raw ticks/s", "%.0f", ticksPerSec);
            tel.addLine();
            tel.addData("설정된 모터", motorName);
            tel.addData("ticks/rev", "%.1f", ticksPerRev);
            tel.addData("사양상 최대 RPM", "%.0f", specMaxRpm);
            tel.addLine();
            if (power >= 0.99) {
                tel.addLine("출력 1.0 — 속도가 안정되면 그 RPM이 실제 최대값이다.");
                tel.addLine("  SHOOTER_MAX_RPM 에 그 값을 적을 것.");
                tel.addLine("  SHOOTER_TARGET_RPM 은 그 값의 60~80%.");
            }
            tel.update();
        }

        shooter.setPower(0);
    }
}
