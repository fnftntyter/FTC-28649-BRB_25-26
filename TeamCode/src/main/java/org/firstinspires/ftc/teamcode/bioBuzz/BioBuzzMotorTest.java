package org.firstinspires.ftc.teamcode.bioBuzz;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * 모터 배선/방향 확인용. <b>드라이브 베이스를 조립한 직후 제일 먼저 돌릴 OpMode.</b>
 * <p>
 * 로봇을 바닥에서 들어올리거나 바퀴가 공중에 뜨게 받쳐놓고 실행한다.
 *
 * <h3>확인 순서</h3>
 * <ol>
 *   <li>X / Y / A / B로 모터를 하나씩 돌린다. 설정 이름과 실제 모터가 맞는지 본다.</li>
 *   <li>각 모터가 <b>앞으로 가는 방향</b>으로 도는지 본다.
 *       거꾸로 도는 모터가 있으면 {@code BioBuzzHardware.initMotors()}의
 *       해당 모터 Direction을 뒤집는다.</li>
 *   <li>엔코더 값이 같이 <b>양수로</b> 올라가는지 본다.
 *       출력은 양수인데 엔코더가 음수로 내려가면 엔코더 케이블이 반대로 꽂힌 것이다.</li>
 *   <li>마지막으로 RB(네 모터 동시)를 눌러 네 값이 비슷하게 올라가는지 본다.</li>
 * </ol>
 *
 * <pre>
 * ── gamepad1 ──
 *  X   leftFront
 *  Y   rightFront
 *  A   leftBack
 *  B   rightBack
 *  RB  네 모터 동시 (전진)
 *  dpad ↑↓  테스트 출력 ±0.05
 *  back     엔코더 0으로 초기화
 * </pre>
 */
@TeleOp(name = "BioBuzz MotorTest", group = "BioBuzz")
public class  BioBuzzMotorTest extends LinearOpMode {

    private double testPower = 0.3;

    @Override
    public void runOpMode() {
        BioBuzzHardware robot = new BioBuzzHardware(hardwareMap);

        telemetry.addLine("!! 바퀴를 공중에 뜨게 받쳐놓고 START !!");
        telemetry.addLine("X:LF  Y:RF  A:LB  B:RB  RB:네개동시");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed()) {
                testPower = Math.min(1.0, testPower + 0.05);
            }
            if (gamepad1.dpadDownWasPressed()) {
                testPower = Math.max(0.0, testPower - 0.05);
            }
            if (gamepad1.backWasPressed()) {
                robot.resetEncoders();
                // STOP_AND_RESET_ENCODER는 런모드를 바꿔버리므로 되돌려준다
                robot.finishMove();
            }

            // 한 번에 하나만. 누른 버튼에 해당하는 모터에만 출력을 준다.
            double lf = 0, rf = 0, lb = 0, rb = 0;
            String active = "(정지)";

            if (gamepad1.right_bumper) {
                lf = rf = lb = rb = testPower;
                active = "네 모터 동시 — 전진";
            } else if (gamepad1.x) {
                lf = testPower;
                active = BioBuzzHardware.NAME_LEFT_FRONT;
            } else if (gamepad1.y) {
                rf = testPower;
                active = BioBuzzHardware.NAME_RIGHT_FRONT;
            } else if (gamepad1.a) {
                lb = testPower;
                active = BioBuzzHardware.NAME_LEFT_BACK;
            } else if (gamepad1.b) {
                rb = testPower;
                active = BioBuzzHardware.NAME_RIGHT_BACK;
            }

            robot.setPowers(lf, lb, rf, rb);

            telemetry.addData(">>> 지금 도는 모터", active);
            telemetry.addData("테스트 출력", "%.2f  (dpad ↑↓로 조절)", testPower);
            telemetry.addLine();
            telemetry.addData("방향 기준", "양수 출력 = 로봇이 앞으로 가는 회전");
            telemetry.addLine();
            telemetry.addData(BioBuzzHardware.NAME_LEFT_FRONT, "pos %7d   power %.2f",
                    robot.leftFront.getCurrentPosition(), robot.leftFront.getPower());
            telemetry.addData(BioBuzzHardware.NAME_LEFT_BACK, "pos %7d   power %.2f",
                    robot.leftBack.getCurrentPosition(), robot.leftBack.getPower());
            telemetry.addData(BioBuzzHardware.NAME_RIGHT_FRONT, "pos %7d   power %.2f",
                    robot.rightFront.getCurrentPosition(), robot.rightFront.getPower());
            telemetry.addData(BioBuzzHardware.NAME_RIGHT_BACK, "pos %7d   power %.2f",
                    robot.rightBack.getCurrentPosition(), robot.rightBack.getPower());
            telemetry.update();
        }

        robot.stop();
    }
}

