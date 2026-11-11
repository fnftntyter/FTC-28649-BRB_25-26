package org.firstinspires.ftc.teamcode.bioBuzz.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.bioBuzz.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.bioBuzz.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.bioBuzz.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.bioBuzz.pedro.procedures.Tests;

/**
 * Pedro Pathing 튜닝 절차 등록.
 * <p>
 * {@code @Tuner}가 붙은 <b>static 메소드</b>를 Sloth(Sinister)가 런타임에 스캔해서
 * 튜닝 웹 인터페이스에 띄운다. 필드가 아니라 메소드에 붙여야 한다.
 *
 * <h3>사용법</h3>
 * <ol>
 *   <li>로봇과 드라이버 허브를 켜고 로봇 컨트롤러에 연결한다.</li>
 *   <li>Pedro Pathing 튜닝 페이지를 열고 아래 절차를 <b>순서대로</b> 실행한다.</li>
 *   <li>각 절차가 끝나면 Java 코드를 뱉어 주니, 그걸 {@link PedroConstants}의
 *       해당 블록에 덮어쓴다.</li>
 * </ol>
 *
 * <h3>순서가 중요하다</h3>
 * Pinpoint(로컬라이저) → Mecanum(드라이브트레인) → Foresight(추종 게인).
 * Foresight 튜너는 로컬라이저와 드라이브트레인이 이미 맞다고 가정하고 움직인다.
 * 앞의 둘이 틀린 상태로 Foresight를 돌리면 엉뚱한 게인이 나온다.
 * <p>
 * Foresight 튜너와 Tests는 {@link PedroConstants}의 설정을 그대로 쓴다.
 * 즉 Pinpoint/Mecanum 결과를 먼저 반영해 두어야 한다.
 */
public class PedroTuning {

    /** 1단계 — 오도메트리 포드 방향과 오프셋을 찾는다. */
    @Tuner(name = "1. Pinpoint")
    public static Procedure pinpoint() {
        return new PinpointTuner();
    }

    /** 2단계 — 네 모터의 이름과 회전 방향을 찾는다. */
    @Tuner(name = "2. Mecanum")
    public static Procedure mecanum() {
        return new MecanumTuner();
    }

    /** 3단계 — 최대 속도/감속도와 추종 게인을 찾는다. 공간이 넓어야 한다. */
    @Tuner(name = "3. Foresight")
    public static Procedure foresight() {
        return new ForesightTuner(
                h -> new PinpointLocalizer(h, PedroConstants.localizerConfig),
                h -> new Mecanum(h, PedroConstants.drivetrainConfig));
    }

    /** 튜닝 후 확인용 — 직선/곡선/홀드/로컬라이제이션 테스트. */
    @Tuner(name = "4. Tests")
    public static Procedure tests() {
        return new Tests(
                h -> new Mecanum(h, PedroConstants.drivetrainConfig),
                h -> new PinpointLocalizer(h, PedroConstants.localizerConfig),
                () -> new Foresight(PedroConstants.foresightConfig));
    }

    private PedroTuning() {
    }
}
