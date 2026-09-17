package org.firstinspires.ftc.teamcode.imReady;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * BLUE 진영 자율. 실제 동작은 {@link AutoBase}에 있고,
 * 좌표·회전량·발사 수는 {@link GameConfig}에서 조정한다.
 */
@Autonomous(name = "AutoDrivingBLUE-Long", group = "imReady")
public class AutoDrivingBLUE_Long extends AutoBase {

    @Override
    protected GameConfig.Alliance alliance() {
        return GameConfig.Alliance.BLUE;
    }
}
