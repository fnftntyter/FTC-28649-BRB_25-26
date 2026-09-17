package org.firstinspires.ftc.teamcode.imReady;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * RED 진영 자율. 경로는 BLUE를 X축 기준으로 대칭시켜 자동 생성되므로,
 * 좌표를 따로 적을 필요가 없다. 튜닝은 {@link GameConfig}에서 한다.
 */
@Autonomous(name = "AutoDrivingRED-Long", group = "imReady")
public class AutoDrivingRED_Long extends AutoBase {

    @Override
    protected GameConfig.Alliance alliance() {
        return GameConfig.Alliance.RED;
    }
}
