package org.firstinspires.ftc.teamcode.bioBuzz;

import android.content.Context;
import android.content.SharedPreferences;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * FtcDashboard가 꺼져 있을 때 강제로 켜는 비상용 OpMode.
 * <p>
 * 원래는 대시보드 라이브러리가 자동 등록하는 <b>"Enable/Disable Dashboard"</b>(그룹 {@code dash})
 * OpMode를 쓰면 된다. 그게 목록에 안 보일 때만 이걸 쓴다.
 *
 * <h3>어떻게 켜지는가</h3>
 * 대시보드의 {@code enable()}은 private이라 코드에서 직접 못 부른다. 대신 켜짐/꺼짐 상태가
 * 안드로이드 SharedPreferences에 저장되는데(FtcDashboard 0.5.0 소스 확인),
 * 그 값을 직접 true로 써 준다:
 * <pre>
 *   파일 이름 : "FtcDashboard"
 *   키        : "autoEnable"   (기본값 true)
 * </pre>
 * 대시보드는 <b>생성될 때</b> 이 값을 읽어서 true면 스스로 켜진다
 * ({@code if (getAutoEnable()) enable();}).
 * 그래서 이 OpMode를 돌린 뒤에는 <b>로봇 앱을 재시작해야</b> 적용된다.
 *
 * <h3>쓰는 순서</h3>
 * <ol>
 *   <li>이 OpMode를 INIT → START (바로 끝난다)</li>
 *   <li>텔레메트리에 "재시작 필요"가 보이면 <b>Restart Robot</b>
 *       (드라이버 허브 ⋮ 메뉴) 또는 Control Hub 전원 재부팅</li>
 *   <li>{@code http://192.168.43.1:8080/dash} 접속</li>
 * </ol>
 * 켜 두면 계속 유지되니 이 OpMode는 한 번만 돌리면 된다.
 */
@TeleOp(name = "Dashboard 강제 켜기", group = "BioBuzz")
public class DashboardFix extends LinearOpMode {

    /** FtcDashboard 0.5.0의 PREFS_NAME */
    private static final String PREFS_NAME = "FtcDashboard";
    /** FtcDashboard 0.5.0의 PREFS_AUTO_ENABLE_KEY */
    private static final String AUTO_ENABLE_KEY = "autoEnable";

    @Override
    public void runOpMode() {
        boolean before = FtcDashboard.getInstance() != null
                && FtcDashboard.getInstance().isEnabled();

        telemetry.addData("지금 대시보드", before ? "켜짐" : "꺼짐");
        telemetry.addLine("START를 누르면 다음 재시작부터 켜지도록 설정합니다.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        SharedPreferences prefs =
                hardwareMap.appContext.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        boolean saved = prefs.getBoolean(AUTO_ENABLE_KEY, true);

        // commit()을 쓴다. apply()는 비동기라서 OpMode가 바로 끝나면 안 써질 수 있다.
        boolean ok = prefs.edit().putBoolean(AUTO_ENABLE_KEY, true).commit();

        while (opModeIsActive()) {
            telemetry.addData("설정 저장", ok ? "성공" : "실패");
            telemetry.addData("이전 저장값", saved);
            telemetry.addData("새 저장값", prefs.getBoolean(AUTO_ENABLE_KEY, true));
            telemetry.addLine();
            if (FtcDashboard.getInstance() != null && FtcDashboard.getInstance().isEnabled()) {
                telemetry.addLine(">>> 이미 켜져 있습니다. http://192.168.43.1:8080/dash");
            } else {
                telemetry.addLine(">>> 재시작 필요: 드라이버 허브 ⋮ → Restart Robot");
                telemetry.addLine("    그 다음 http://192.168.43.1:8080/dash");
            }
            telemetry.update();
            sleep(200);
        }
    }
}
