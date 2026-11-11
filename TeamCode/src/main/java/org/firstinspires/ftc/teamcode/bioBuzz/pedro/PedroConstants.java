package org.firstinspires.ftc.teamcode.bioBuzz.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pedro Pathing 3.0.2 설정. BIOBUZZ 로봇의 <b>자율과 수동조작이 모두</b> 이걸 쓴다.
 * <ul>
 *   <li>{@code bioBuzz.auto.BioBuzzAutoRed / BioBuzzAutoBlue} — 자율</li>
 *   <li>{@code bioBuzz.BioBuzzDriving} — 수동조작 (Pedro 내장 드라이브)</li>
 * </ul>
 * 모터 이름/방향, Pinpoint 설정은 <b>이 파일 한 곳만</b> 고치면 둘 다 바뀐다.
 * ({@code imReady} 폴더는 작년 로봇용 Road Runner 코드라 여기와 관계없다.)
 *
 * <h3>중요 — 한 OpMode에서 다른 드라이브 코드와 같이 쓰지 말 것</h3>
 * {@link Mecanum}은 네 모터를 hardwareMap에서 직접 받아 <b>자기 방향 설정을 덮어쓴다.</b>
 * {@code BioBuzzHardware}나 Road Runner의 {@code MecanumDrive}도 같은 모터를 잡으므로,
 * 하나의 OpMode 안에서 둘을 동시에 만들면 모터 방향과 런모드가 서로를 망가뜨린다.
 * Pinpoint도 마찬가지로 양쪽에서 initialize하면 좌표가 틀어진다.
 * <p>
 * → Pedro를 쓰는 OpMode는 {@link #createFollower(HardwareMap)} 하나만 부른다.
 * → {@code BioBuzzMotorTest}만 예외로 {@code BioBuzzHardware}를 쓴다 (모터 배선 확인용).
 *
 * <h3>값의 출처</h3>
 * 드라이브트레인/로컬라이저 값은 기존 {@code imReady.MecanumHardware}와
 * {@code bioBuzz.BioBuzzHardware}에서 그대로 가져왔다.
 * <b>{@link #foresightConfig}의 숫자는 전부 임시값이다.</b> 아래 "튜닝" 참고.
 *
 * <h3>튜닝 순서</h3>
 * {@code PedroTuning}이 등록한 튜너를 드라이버 허브에서 실행한다.
 * 각 튜너가 끝나면 붙여넣을 Java 코드를 만들어 주니, 그걸 이 파일의
 * 해당 블록에 <b>덮어써야</b> 한다. 순서는 Pinpoint → Mecanum → Foresight.
 */
public class PedroConstants {

    // ────────────────────────────────────────────────
    //  드라이브트레인
    //  모터 이름/방향은 BioBuzzHardware와 같은 전제(왼쪽 FORWARD, 오른쪽 REVERSE).
    //  MecanumTuner를 돌리면 이 블록을 생성해 준다.
    // ────────────────────────────────────────────────

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFront");
        c.frontRightName.set("rightFront");
        c.backLeftName.set("leftBack");
        c.backRightName.set("rightBack");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    // ────────────────────────────────────────────────
    //  로컬라이저 — goBILDA Pinpoint (컨트롤 허브 I2C, 설정 이름 "pinpoint")
    //
    //  ⚠ 아래 오프셋 두 개는 "작년 로봇"의 값이다 (imReady.MecanumHardware.initPinpoint()에서 옮김).
    //    새 로봇에 포드를 새로 달았으면 반드시 다시 재야 한다. 틀리면 직진은 맞는데
    //    제자리에서 돌릴 때 위치가 슬금슬금 밀린다.
    //
    //  부호 규약 — Pedro는 이 값을 손대지 않고 그대로
    //  GoBildaPinpointDriver.setOffsets(x, y, 단위)에 넘긴다. 즉 goBILDA/FTC SDK 규약과 같다
    //  (SDK 샘플 SensorGoBildaPinpoint.java 참고):
    //
    //    xPodOffset = X(전진) 포드가 추적 중심에서 "좌우"로 얼마나 떨어져 있나
    //                 → 중심보다 왼쪽이면 +, 오른쪽이면 −
    //    yPodOffset = Y(횡이동) 포드가 추적 중심에서 "앞뒤"로 얼마나 떨어져 있나
    //                 → 중심보다 앞이면 +, 뒤면 −
    //
    //  추적 중심 = 로봇의 위치로 삼고 싶은 점 (보통 로봇 정중앙).
    //  재는 법: 그 점에서 각 포드 "바퀴"까지 줄자로, 위 부호 규칙대로 적는다.
    //  직접 재도 되고, PinpointTuner(튜너 1번)를 돌려 자동으로 뽑아도 된다.
    // ────────────────────────────────────────────────

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");

        // 포드 종류 — 4-bar 포드인지 swingarm 포드인지 확인하고 맞출 것.
        // swingarm이면 goBILDA_SWINGARM_POD 으로 바꾼다. 틀리면 이동 거리가 전체적으로
        // 비율만큼 어긋난다 (예: 24인치 가라고 했는데 30인치 감).
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // [측정 필요] 작년 로봇 값. 위 주석의 부호 규약대로 새로 재서 넣을 것.
        c.xPodOffset.set(3.0);
        c.yPodOffset.set(-72.0);
        c.offsetUnits.set(DistanceUnit.MM);      // 위 오프셋 두 개의 단위

        // [확인 필요] 포드를 어느 방향으로 달았는지에 따라 뒤집힌다.
        // 로봇을 손으로 앞으로 밀 때 x가 +로, 왼쪽으로 밀 때 y가 +로 올라가야 맞다.
        // 반대로 가면 그 축을 REVERSED ↔ FORWARD 로 바꾼다.
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);

        c.globalDistanceUnit.set(DistanceUnit.INCH); // 경로/Pose에서 쓰는 단위
    });

    // ────────────────────────────────────────────────
    //  Foresight (경로 추종 알고리즘)
    //
    //  ⚠ 아래 12개는 ForesightConfig에서 기본값이 없는 required 항목이라
    //    반드시 채워야 객체 생성이 된다. 그래서 일단 "보수적인 임시값"을 넣어 두었다.
    //    실측값이 아니다 — ForesightTuner를 돌려 나온 코드로 이 블록을 통째로 교체할 것.
    //
    //  튜닝 전에는 로봇이 경로를 제대로 못 따라간다. 처음 돌릴 때는
    //  로봇을 받쳐 바퀴를 띄우거나, 넓은 곳에서 짧은 경로로만 시험할 것.
    // ────────────────────────────────────────────────

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // 거리 오차 보정. 오차가 2.5인치보다 크면 primary, 그 안쪽이면 secondary.
        Controller primaryTranslationalForward   = Controller.proportional(0.10);
        Controller secondaryTranslationalForward = Controller.proportional(0.02);
        Controller primaryTranslationalLateral   = Controller.proportional(0.10);
        Controller secondaryTranslationalLateral = Controller.proportional(0.02);

        c.forwardTranslational.set(
                Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(
                Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.02));
        c.brake.set(Controller.proportionalFeedforward(0.02));

        c.headingFeedback.set(Controller.proportional(1.0));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.5, 0.1));

        c.linearBrakeCoefficients.set(Matrix.diag(0.5, 0.5));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.1, 0.1));

        // 전부 positive여야 한다 (validator). 단위는 inch/s, inch/s².
        c.maxAchievableForwardVelocity.set(50.0);
        c.maxAchievableStrafeVelocity.set(35.0);
        c.naturalForwardDeceleration.set(30.0);
        c.naturalStrafeDeceleration.set(40.0);

        // ── 안전 상한 ──
        // maxPathSpeed / maxVelocityConstraint의 기본값은 Infinity다. 즉 속도 제한이 없다.
        // 게인이 아직 임시값인 상태에서 그대로 돌리면 첫 주행에 로봇이 전속으로 튀어나간다.
        // 튜닝을 끝낸 뒤 올리거나 이 두 줄을 지울 것.
        c.maxPathSpeed.set(24.0);
        c.maxVelocityConstraint.set(24.0);
    });

    // ────────────────────────────────────────────────

    /**
     * 자율 OpMode에서 한 번만 호출한다.
     * <p>
     * 호출 시점에 모터와 Pinpoint를 잡으므로, 같은 OpMode에서
     * {@code BioBuzzHardware}나 Road Runner {@code MecanumDrive}를 만들면 안 된다.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig));
    }

    private PedroConstants() {
    }
}
