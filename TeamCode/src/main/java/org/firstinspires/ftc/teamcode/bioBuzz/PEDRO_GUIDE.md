# BIOBUZZ 로봇 — Pedro Pathing 가이드

FTC 28649 BRB · 2026-27 BIOBUZZ 시즌
Pedro Pathing 3.0.2 / FTC SDK 12.0.0 기준

> 이 문서는 "다음 사람이 혼자서 이어갈 수 있게" 쓴 것이다.
> 막히면 **7장 문제 해결**부터 보고, 값을 바꾸기 전에 **6장 체크리스트**를 확인할 것.

---

## 1. 파일 지도

```
bioBuzz/
├─ BioBuzzDriving.java        수동조작 (TeleOp)        ← Pedro 내장 드라이브 사용
├─ BioBuzzMotorTest.java      모터 배선 확인용          ← Pedro 안 씀 (BioBuzzHardware)
├─ BioBuzzHardware.java       MotorTest 전용 하드웨어
├─ PEDRO_GUIDE.md             이 문서
├─ auto/
│  ├─ BioBuzzAutoBase.java    자율 "순서" (이동→발사→파킹), 시간 설정, 발사 장치 자리
│  ├─ BioBuzzAutoRed.java     RED  좌표만
│  └─ BioBuzzAutoBlue.java    BLUE 좌표만
└─ pedro/
   ├─ PedroConstants.java     ★ 모터·Pinpoint·추종 게인 설정 (가장 중요)
   ├─ PedroTuning.java        튜너 등록 (1~4단계)
   ├─ PoseStorage.java        자율 끝 위치 → TeleOp으로 넘기기
   └─ procedures/             공식 튜너 코드 (건드릴 일 없음)
```

**규칙 하나만 기억:** 모터 이름·방향, Pinpoint 설정은 `pedro/PedroConstants.java` **한 곳**에만 있다.
여기를 고치면 자율과 TeleOp이 둘 다 바뀐다.

`imReady/` 폴더는 **작년(DECODE) 로봇**의 Road Runner 코드다. BIOBUZZ 코드와 섞지 말 것.

---

## 2. 처음 한 번 해야 하는 준비

1. **Android Studio에서 Gradle Sync** (툴바 코끼리 아이콘). 빨간 줄이 잔뜩 뜨면 대부분 이것.
2. **Robot Configuration 이름 맞추기** — 드라이버 허브 → Configure Robot 에서:

   | 장치 | 이름 |
   |---|---|
   | 왼쪽 앞 모터 | `leftFront` |
   | 왼쪽 뒤 모터 | `leftBack` |
   | 오른쪽 앞 모터 | `rightFront` |
   | 오른쪽 뒤 모터 | `rightBack` |
   | goBILDA Pinpoint (I2C) | `pinpoint` |

   이름이 하나라도 다르면 OpMode가 INIT에서 바로 죽는다.
3. **`BioBuzz MotorTest`로 배선 확인** — 바퀴를 띄우고 X/Y/A/B로 한 개씩 돌려 본다.
   양수 출력에 바퀴가 "로봇이 앞으로 가는 방향"으로 도는지 확인.
4. **튜닝 (3장)** — 이거 안 하면 자율이 제대로 안 움직인다.

---

## 3. 튜닝 — 반드시 순서대로

Pedro는 튜너를 돌리면 **붙여넣을 Java 코드를 만들어 준다.** 그걸 `PedroConstants.java`에 덮어쓰면 끝.

### 튜닝 페이지 여는 법
1. 노트북을 로봇 Control Hub Wi-Fi에 연결
2. 브라우저에서 **`http://192.168.43.1:10158`** 접속
   (포트 10158은 Pedro 튜닝 라이브러리 코드에서 확인한 값. Control Hub 기본 주소가 192.168.43.1)
3. 목록에 `1. Pinpoint`, `2. Mecanum`, `3. Foresight`, `4. Tests` 가 보이면 정상

> 안 열리면: 로봇 앱이 켜져 있는지, 같은 Wi-Fi인지 확인. 그래도 안 되면
> 공식 문서 https://pedropathing.com 의 Tuning 페이지 참고.

### 순서

| 단계 | 하는 일 | 결과를 붙여넣을 곳 |
|---|---|---|
| **1. Pinpoint** | 로봇을 손으로 밀고 돌리면 포드 방향·오프셋을 찾아 줌 | `PedroConstants.localizerConfig` |
| **2. Mecanum** | 모터 이름·방향 확인 | `PedroConstants.drivetrainConfig` |
| **3. Foresight** | 로봇이 스스로 달리며 속도·감속·게인 측정. **넓은 공간 필요** | `PedroConstants.foresightConfig` |
| **4. Tests** | 직선/곡선/홀드 테스트로 확인 | (붙여넣기 없음) |

- **순서를 지킬 것.** 3번은 1·2번이 맞다는 전제로 달린다. 앞이 틀리면 엉뚱한 게인이 나온다.
- 붙여넣은 뒤에는 **다시 빌드해서 로봇에 올려야** 다음 단계에 반영된다.
- 3번이 끝나면 `foresightConfig` 맨 아래 **안전 상한 두 줄**(`maxPathSpeed`, `maxVelocityConstraint` = 24)을
  올리거나 지운다. 그 전까지는 일부러 느리게 막아 둔 것이다.

---

## 4. Pedro 핵심 개념 (코드 읽는 법)

### 4-1. Follower — 로봇 운전사
```java
Follower follower = PedroConstants.createFollower(hardwareMap);  // init()에서 딱 한 번
```
`Follower`가 모터도 돌리고 위치(Pinpoint)도 읽는다. **OpMode 하나에 하나만** 만든다.

### 4-2. update() — 매 루프 반드시 한 번
```java
public void loop() {
    follower.update();   // 이거 빠지면 로봇이 안 움직인다 (제일 흔한 실수)
    ...
}
```

### 4-3. Pose — 위치 + 방향
```java
new Pose(x, y, heading)   // x, y는 인치, heading은 라디안!
Math.toRadians(90)        // 도 → 라디안
```
자율 파일에서는 도우미 `pose(x, y, 도)` 를 쓰면 도(degree)로 적을 수 있다.

### 4-4. 경로 만들고 따라가기
```java
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.curve;

Pose a = pose(9, 36, 0);
Pose b = pose(48, 72, 45);

Path p1 = line(a, b).linear(a, b);        // 직선, 방향은 a→b로 서서히 바뀜
Path p2 = line(a, b).constant(0);         // 직선, 방향은 0°로 고정
Path p3 = line(a, b).tangent();           // 직선, 진행 방향을 바라보며
Path p4 = curve(a, pose(30, 36, 0), b).tangent();  // 곡선 (가운데 점이 휘는 정도를 결정)
Path p5 = line(a, b).facingPoint(pose(72, 72, 0)); // 가면서 계속 (72,72)를 바라봄 ← 조준에 유용

follower.follow(p1);     // 출발
follower.isBusy();       // true = 아직 가는 중, false = 도착
follower.hold(b);        // 그 자리에서 버티기 (밀어도 돌아옴)
follower.pose();         // 지금 위치
follower.setPose(a);     // "로봇은 지금 여기 있다"고 알려주기 (자율 시작할 때)
```
±180° 경계를 넘을 때(예: 180° → −170°)는 짧은 쪽(10°)으로 돈다. 확인해 봤음.

### 4-5. 수동조작 (BioBuzzDriving 안에서 하는 일)
```java
// Pedro 규약: DrivePowers(앞, 왼쪽, 반시계) — 셋 다 그 방향이 +
DrivePowers powers = new DrivePowers(-gamepad1.left_stick_y,
                                     -gamepad1.left_stick_x,
                                     -gamepad1.right_stick_x);

powers = ManualDrive.fieldCentric(앞, 왼쪽, 회전, follower.pose().heading(), 정면방향);  // 필드중심
follower.manual(powers);                       // 그냥 주행
ManualDrive.driveOrHold(follower, powers);      // 놓으면 그 자리 버티기
ManualDrive.headingLock(follower, pid, powers, 목표각);  // 방향 고정
```

---

## 5. 좌표계와 측정 방법

### 5-1. 팀 좌표계 (RED/BLUE 공통)
```
                 먼 쪽 벽 (관중 반대편)
          y=144 ┌──────────────┬──────────────┐
                │   RED 쪽      │   BLUE 쪽    │
                │  (A B C 열)   │  (D E F 열)  │
   RED 벽 ───── │              │              │ ───── BLUE 벽
   (x = 0)      │           (72,72)           │       (x = 144)
                │           필드 중앙          │
            y=0 └──────────────┴──────────────┘
              x=0          관중석 쪽 벽         x=144
```
- 원점 (0, 0) = **관중석 쪽 벽 × RED 벽** 모서리
- +x = RED 벽 → BLUE 벽, +y = 관중석 → 먼 쪽, 단위 **인치** (타일 한 칸 = 24인치)
- 방향 0° = BLUE 벽을 바라봄, 90° = 먼 쪽 벽을 바라봄, **반시계가 +**
- 매뉴얼상 RED ALLIANCE AREA는 관중석에서 봤을 때 왼쪽 → 그래서 RED 벽이 x = 0

### 5-2. 위치 재는 법
1. 로봇을 원하는 자리에 놓는다.
2. 줄자로 **로봇 중심**에서 **RED 벽까지** = x, **관중석 쪽 벽까지** = y.
   (로봇 중심 = 보통 Pinpoint 기준점. 18인치 로봇이 벽에 붙어 있으면 중심은 벽에서 9인치)
3. 로봇이 보는 방향을 0°/90°/180°/−90° 중에서 고른다 (BLUE 벽 쪽이 0°).
4. `BioBuzzAutoRed/Blue.java`의 해당 숫자를 고친다.
   빠르게 시험하려면 FtcDashboard(`http://192.168.43.1:8080/dash`)의 `BioBuzzAutoRed` 항목에서 바꿔도 된다.
   **Dashboard에서 바꾼 값은 앱 재시작하면 사라진다 → 맞는 값은 꼭 파일에 다시 적기.**

### 5-3. 잰 값이 맞는지 확인하는 법
- 자율을 돌리고 텔레메트리의 `위치 X / Y`를 본다. 실제로 로봇이 선 자리와 줄자로 비교.
- 출발 위치를 틀리게 적으면 **모든 지점이 같은 양만큼 밀린다.** 전부 어긋나면 출발 위치부터 의심.

---

## 6. ★ 측정하고 수정해야 할 것 — 체크리스트

`[ ]` 를 하나씩 지워 나가면 된다. 위에서부터 순서대로.

### A. 로봇 하드웨어
- [ ] **Robot Configuration 이름** 5개가 2장 표와 같은지
- [ ] **모터 방향** — `BioBuzz MotorTest` 또는 튜너 2번 → `PedroConstants.drivetrainConfig`
- [ ] **Pinpoint 포드 종류** — 지금 `goBILDA_4_BAR_POD`. 스윙암 포드면 바꿔야 함
- [ ] **Pinpoint 오프셋·방향** — 지금 값(3mm, −72mm, FORWARD/REVERSED)은 **작년 로봇** 값. 튜너 1번으로 새로 측정
- [ ] **로봇 실제 크기** — 출발 X(RED 9, BLUE 135)는 18인치 로봇 가정. 작으면 바꿈

### B. 튜닝 (3장)
- [ ] 튜너 1 → 2 → 3 → 4 순서로 완료, 결과를 `PedroConstants`에 붙여넣음
- [ ] `foresightConfig`의 **임시값 12개**가 튜너 값으로 전부 바뀌었는지
- [ ] **안전 상한** `maxPathSpeed`, `maxVelocityConstraint` (= 24) 올리거나 삭제
- [ ] TeleOp `B`(놓으면 버티기) 켜서 흔들리지 않는지 확인 — 흔들리면 3번 튜닝 다시
- [ ] TeleOp dpad 방향 고정이 흔들리면 `BioBuzzDriving.HEADING_LOCK_P` 줄이기 / `HEADING_LOCK_D` 늘리기
- [ ] TeleOp 속도 `SPEED_NORMAL`(0.8), `SPEED_SLOW`(0.3) 드라이버 취향대로

### C. 필드 좌표 — RED, BLUE **각각 따로** 측정
BLUE 값은 RED를 좌우 반전해서 만든 **추측**이다. 필드가 정말 좌우 대칭인지는 확인 못 했다
(매뉴얼상 GARDEN이 "서로 반대쪽 모서리"라 회전 대칭일 수도 있음). **둘 다 직접 재야 한다.**

| 항목 | RED 지금 값 | BLUE 지금 값 | 조건 |
|---|---|---|---|
| `START_FAR` | (9, 108, 0°) | (135, 108, 180°) | 벽에 닿음, 자기 쪽, LOADING ZONE 아님 |
| `START_NEAR` | (9, 36, 0°) | (135, 36, 180°) | 〃 + 같은 편 로봇과 겹치지 않게 |
| `LAUNCH` | (48, 72, 0°) | (96, 72, 180°) | HIVE의 **위를 향한 CELL**에 공이 들어가는 자리 |
| `PARK` | (12, 72, 0°) | (132, 72, 180°) | LOADING ZONE에 **걸치되 벽엔 안 닿게** |

- [ ] RED 4개 위치 측정 → `BioBuzzAutoRed.java`
- [ ] BLUE 4개 위치 측정 → `BioBuzzAutoBlue.java`
- [ ] **LOADING ZONE이 실제로 어디 있는지** 확인 (지금 y = 72는 완전 추측)
- [ ] 출발 위치가 LOADING ZONE과 겹치지 않는지 (겹치면 규칙 위반 G304)
- [ ] 같은 편 로봇과 출발 위치 정하기 (INIT에서 dpad ↑ 먼 쪽 / ↓ 관중석 쪽)

### D. 시간 (`BioBuzzAutoBase.java` 맨 위)
- [ ] `LAUNCH_WAIT_S` (3초) — 발사 장치 만들면 실제 4발 쏘는 시간으로
- [ ] `PARK_DEADLINE_S` (25초) — 발사 위치→파킹 이동 시간을 재서 `30 − (이동시간 + 2)`
- [ ] `PATH_TIMEOUT_S` (6초) — 제일 긴 경로가 걸리는 시간 + 여유

### E. 발사 장치 (아직 없음)
- [ ] `BioBuzzAutoBase`의 `startLaunch / updateLaunch / launchFinished / stopLaunch` 4개 채우기
  (RED/BLUE 파일은 안 건드려도 됨)
- [ ] `launchFinished()`를 "시간 지났나" → "4발 다 쐈나"로 바꾸기

### F. 나중에 (조준을 카메라로 할 때)
- [ ] BIOBUZZ AprilTag는 **4장짜리 묶음(클러스터)**으로만 등록돼 있다 (ID 30~45).
  `imReady/CameraBase`의 ID 조회는 클러스터를 못 읽는다 → 새로 짜야 함
  - 30~33 RED CELL(먼 쪽), 34~37 RED CELL(관중석 쪽), 38~41 BLUE CELL(관중석 쪽), 42~45 BLUE CELL(먼 쪽)
  - 태그는 CELL **밑면에 아래를 향해** 붙어 있다 (매뉴얼 9.9)

---

## 7. 문제 해결

| 증상 | 의심할 것 |
|---|---|
| INIT 누르자마자 죽음 | Configuration 이름 오타 (2장 표) |
| 로봇이 아예 안 움직임 | `loop()`에 `follower.update()` 빠짐 |
| 앞으로 가라는데 옆/대각선으로 감 | 모터 방향 (튜너 2번) |
| 직진은 되는데 돌면 위치가 엉망 | Pinpoint 오프셋 (튜너 1번) |
| 경로 끝에서 계속 흔들림 / 지나침 | `foresightConfig` 튜닝 안 됨 (튜너 3번) |
| 모든 지점이 같은 방향으로 밀림 | 출발 위치(`START_*`)를 틀리게 적음 |
| 자율이 중간에 멈춘 듯함 | 어딘가 걸림 → `PATH_TIMEOUT_S` 후 다음 단계로 넘어감. 텔레메트리 `단계` 확인 |
| 너무 느림 | 안전 상한 24 그대로임 (3장 마지막) |
| TeleOp 필드중심이 이상한 방향 | 로봇을 드라이버 반대쪽 보게 하고 `back` |
| 빨간 줄 잔뜩 (빌드는 됨) | Android Studio Gradle Sync |
| INIT 중 로봇을 건드렸더니 방향이 틀어짐 | Pinpoint가 INIT 때 자이로를 보정한다. **INIT~START 사이 로봇 건드리지 말기** |

---

## 8. 자율에 단계 추가하는 법 (예: POLLEN 주우러 가기)

`BioBuzzAutoBase.java`에서:

1. `Step`에 단계 추가
   ```java
   private enum Step { DRIVE_TO_LAUNCH, LAUNCHING, DRIVE_TO_PICKUP, PICKING_UP, DRIVE_TO_PARK, DONE }
   ```
2. RED/BLUE가 채울 좌표 추가
   ```java
   protected abstract Pose pickupPose();
   ```
   그리고 `BioBuzzAutoRed/Blue.java`에 `PICKUP_X/Y/DEG` 와 `pickupPose()` 추가
3. `switch(step)`에 흐름 연결
   ```java
   case LAUNCHING:
       ...
       if (launchFinished()) { stopLaunch(); goTo(pickupPose()); step = Step.DRIVE_TO_PICKUP; }
       break;
   case DRIVE_TO_PICKUP:
       if (pathFinished()) { stepTimer.reset(); step = Step.PICKING_UP; }
       break;
   ```
4. **30초 안에 끝나는지** 꼭 재 볼 것. `PARK_DEADLINE_S`가 마지막 안전장치다.

---

## 9. 득점 메모 (BIOBUZZ AUTO, 매뉴얼 기준)

| 항목 | 점수 | 조건 |
|---|---|---|
| LEAVE | 3 | AUTO 끝날 때 필드 벽에 **안 닿아** 있음 |
| PARK | 5 | AUTO 끝날 때 LOADING ZONE에 **일부라도** 들어가 있음 |
| HIVE TIP | 20 | 위를 향한 CELL에 발사해서 HIVE를 기울임 |

- AUTO 30초, 미리 실을 수 있는 POLLEN 4개, 로봇 18인치 큐브 안
- FLOWER 득점은 경기 마지막 1분부터라 AUTO에선 불가
- 상대 진영(반대쪽 열)에서 상대 로봇·공 건드리면 MAJOR FOUL (G402)
- 랜덤 요소(작년 모티프 같은 것) 없음
