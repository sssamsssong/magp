# MAGP 대회 전 스위치 검증 (2026-11-07)

대상: `race_script.yaml` 의 미검증 항목, `scan_deskew`, `amcl_mask`.
원칙: 새 알고리즘 금지, 근거 없으면 현재값 유지, 한 번에 하나만 변경.

**이 문서의 결론은 전부 잠정이다.** bag 재생은 아직 돌리지 못했다. 근거는 레포의
`logs/*/driven_path.csv` 와 `gains.yaml` 이고, 주행마다 코드·맵·교통이 달라 인과를
확정할 수 없다. 확정은 아래 "bag 재생" 결과로 한다.

## 잠정 판정

| 항목 | 현재값 | 잠정 판정 | 근거 | 남은 확인 |
|---|---|---|---|---|
| `map.l1_curv_tmin` | 0.50 | 유지 | 8/20 이후 모든 주행에서 켜져 있었다. v>2·비회피·잔차<0.40 에서 car2 64런 횡오차 중앙 7.3㎝, 90% 24.4㎝, \|δ\|>0.34 0.0%. car1 4런 8.4㎝, 27.9㎝, 3.4%. 8/16 실패 증상(조향 포화 48%)은 재현되지 않았다. 끈 주행은 없어 A/B 는 아니다 | open-loop `/drive` 비교 |
| `frenet.shift_close_*` | 0.30 / 6.0 / 0.60 | 유지 | 8/20 이후 항상 켜짐. 비교군 없음 | 회피 bag 재생 |
| `stanley.skew_gain/max` | 1.0 / 0.35 | 유지 | 8/16 이후 항상 켜짐. 비교군 없음 | **자세가 나쁜** 주행의 bag 재생 |
| `stanley.raycast_wall_skip` | 0.10 | 유지 | `min_room` 0.26 ≥ 0.10+0.155 로 짝은 맞다. 효과는 미확인 (아래) | 같은 bag 0 / 0.10 비교 |
| `scan_deskew` | false | 유지 | 켠 주행에서 개선이 안 보인다 (아래) | AMCL 재생 A/B |
| `amcl_mask` | false | 유지 | deskew 와 항상 함께 켜져 분리 불가 (아래) | AMCL 재생 A/B |

### deskew·mask 를 켠 주행 (car1, `0824-2_clean`)

8/24 저녁에 둘 다 켜고 달린 기록이 있다. 정렬 실패 = 런 중앙 잔차 > 0.40m.

| | 런 | 정렬 실패 | 잔차 v<2 | 2~3.5 | v≥3.5 |
|---|---|---|---|---|---|
| 둘 다 켬 (8/24) | 20 | 0 | 21.2㎝ | 21.3㎝ | 16.2㎝ |
| 둘 다 끔 (8/25) | 11 | 6 | 17.1㎝ | 16.9㎝ | 15.5㎝ |

- 켠 쪽은 자기 입력(`/scan_deskewed`)으로 채점해 유리한 잣대인데도 낮지 않다.
- 정렬 실패 0/20 대 6/11 을 스위치 효과로 읽으면 안 된다. 두 묶음은 설정 키부터
  다르다(`corridor_filter`, `grid_adapt`, 웨이포인트 등). car1 코드를 car2 로 되돌린
  시점과도 겹쳐 보인다.
- **"정렬 실패" 라는 이름 자체도 근거가 약하다.** 6개 중 4개(`011944`, `012533`,
  `012627`, `012955`)는 잔차가 주행 내내 한 값으로 고정돼 있다. CSV 는 마지막 수신값을
  적으므로, 위치를 잃은 것인지 잔차 발행이 멈춘 것인지 CSV 로는 못 가른다.
  `localization_monitor` 는 쓸 빔이 10개 미만이면 발행하지 않는다.
- 고속 코너(v≥3.5, |ω|>1) 표본은 켠 쪽에 없다. deskew 가 효과를 낼 구간을 재지 못했다.

### wall_skip 0.20 을 8/22 에 되돌린 이유

CSV 로는 밝히지 못했다. 되돌린 커밋(`2f165f7`)은 `min_room` 0.22→0.36 도 함께
바꿨으므로 0.20 만의 문제라고 볼 수도 없다. 네 주행 중 둘에 114s·127s 의 기록
공백이 있지만 원인은 CSV 에 없다. 0.10 묶음의 `n_obs>0` 53% 대 끈 묶음 26% 도
맵과 교통이 달라 비교 대상이 아니다. 당시 팀원에게 묻는 것이 가장 빠르다.

### deskew 타이밍 메모

`개발현황.md` 는 `scan_time` 25.00ms / `time_increment×1081` 18.77ms 를 자기모순으로
적었다. 그런데 UST-10LX 가 40Hz 로 270° 를 쓸면 25ms × 270/360 = 18.75ms 라, 두 값은
모순이 아닐 수 있다 (추정, 실측 아님). 실제 주기 29.2ms 와 `stamp_at: start` 는 여전히
미확인이다.

## 규정 시간 값과 MAGP 형식

`seq_time_q3: 75` / `seq_total_limit: 255` 의 근거(60s/240s)는 8월 BEXCO 대회
규정(`docs/roboracer_rules_2026.md`)이다. 팀 지원서(9/17)에 정리된 MAGP 형식은 다르다.
공식 규정 원문으로 다시 확인할 것.

- 예선: 팀당 5분 단독주행 2회, 벽 충돌 없이 연속 주파한 랩 수 계측, 최소 랩타임 순 배열
- 결승: 코스 반대 지점에서 동시 출발, 10바퀴 선착, 3전 2승

코드에 미치는 영향:

| 실행 | 자동 정지 | MAGP 에서 |
|---|---|---|
| `./race.sh q3` | ⚠ `time_limit_sec` 가 0 이면 **`seq_time_q3`(75초)에 정지** (`race.launch.py:397`) | 5분 예선에 그대로 쓰면 75초에 멈춘다 |
| `./race.sh q1` | 없음 (`laps: 0`, `time_limit_sec: 0`) | 멈추지 않는다. 300초 정지는 없다 |
| `./race.sh quali` | Q1→Q2→Q3 시퀀서, 255초 | 형식이 달라 쓰지 않는다 |
| `./race.sh 10laprace` | 10랩 | 결승 형식과 맞는다 |

값은 바꾸지 않았다. 사람이 정할 일: 예선을 어떤 모드로 달릴지. `q3` 를 쓴다면
`mission.time_limit_sec: 300` 이 필요하다.

## 머지 브랜치 점검

레포에서 확인한 것 (`.car`, `./race.sh box` 기동 로그는 차가 있어야 본다):

이 레포는 1호차 한 대 전용이다. `main` 을 기준으로 두고 필요한 수정만 가져온다.
`car2` 는 `stuck_recovery` 등 8/25~26 작업의 참고용으로 보관한다.

- **`_align_sxy` 는 지금 코드에서 실제 버그가 아니다.** 값이 지워지지는 않지만, 정렬을
  다시 돌리는 경로 두 곳(`_manual_cb`, `_redo_cb`)이 모두 값을 먼저 설정하고
  `/race/relocalize` 발행자도 LT 뿐이다. `stuck_recovery` 를 가져오면 다시 볼 것.
- **1호차 전용 수정이 8/26 이식에서 빠졌다.** 차에서 시험한 뒤 합친다:

  | 수정 | 상태 |
  |---|---|
  | `f719001` 경기 중 정지 금지 | 브랜치 `fix/no-stop-in-race`. 충돌 없음 |
  | `e2abd3a` AMCL 전용 `_loc` 맵 | 브랜치 `fix/loc-map`. `_loc` 출력은 0821 맵으로 확인 |
  | `cbbc10f` 유령 검출 회랑 필터 | 보류. 회피 코드 충돌 2건 |
  | `30f47b0` 출발선 2개 자동 선택 | 미정. 충돌 2건, MAGP 결승 형식과 관련 |

- **`mu` 0.63 은 이 차의 측정값과 어긋난다.** 이력은 0.63 → 0.42 (8/20 실내 재검증,
  `bb3f2b3`) → 0.35 (8/24 slip, `0028a76`) 이고, 8/25 주행은 0.6, 8/26 커밋이 0.63 이다.
  주석은 "car2 동일" 이지만 car2 는 0.6 이다. 0.63 으로 달린 로그는 없다.
- **`l1_k` 0.75 / `slip_gain` 0.5 는 1호차 대책을 되돌린 값이다.** 8/22 Q3 직선 사인파
  진동 때문에 1.3 / 0.7 로 올렸었다(`e97e4da`). 이후 1호차 로그에 Q3 까지 간 주행이
  없어 이 값의 고속 안정성은 미확인이다.
- **`grip_util.q1` 주석도 틀리다.** 1호차 0.5 (주석 "car2 동일"), car2 는 0.1.
- **"빌려온 값" 경고는 안 뜬다.** `apply_vehicle.py` 는 `(1호차 값)`, `1호차에서 빌려`,
  `차용` 만 찾는데 1호차 파일은 `car2 동일` 로 적혀 있다. 경고가 없어도 이식값은 6개다.
- **이식 튜닝의 주행 기록은 8/25 의 12런뿐이다** (mu 0.6, 최고 4.77㎧). 8/26 의 복구
  기능(프리셋·LT 정렬) 은 이 브랜치 로그에 시험 기록이 없다.
- **AEB 는 꺼져 있다** (`safety.enabled: False`). `safety_node.py` 에 라이다 오프셋 보정이
  없는 것도 그대로라 켜면 안 된다.

## bag 재생 절차

bag 은 git 에 없다 (`logs/bags/` 는 `.gitignore`). 젯슨이나 bag 을 가진 ROS 2 Humble
노트북에서 돌린다. 도구는 `bag_ab.py` 이고 자식 프로세스를 도메인 99 + localhost 에
가두므로 실차와 섞이지 않는다.

⚠ `bag_ab.py` 는 채점·판정 로직만 합성 데이터로 시험했다. **ROS 노드를 띄우는 부분은
아직 한 번도 안 돌았다.** 첫 실행에서 고칠 것이 나올 수 있다. `<out>/logs/` 를 볼 것.

맵은 **그 주행 당시 것**을 쓴다. `0821_clean` 은 8/24 에 같은 이름으로 덮어써졌다.
옛 맵은 `git log -- src/race_stack/maps/<맵>.pgm` 으로 찾는다. `report` 첫 줄의 녹화
자세 잔차가 40㎝ 를 넘으면 맵부터 의심한다.

```bash
# deskew (q1 고속 주행 bag)
./bag_ab.py loc logs/bags/<주행> --gains logs/<주행>/gains.yaml --map <맵.yaml> \
    --arms raw deskew --repeat 3 --out /tmp/deskew
./bag_ab.py report /tmp/deskew --map <맵.yaml>
```

```bash
# amcl_mask (8/15·8/24 race bag). 스택을 띄운 상태에서 파라미터를 한 번 받아 둔다
ros2 param dump /frenet_planner > /tmp/frenet.yaml
./bag_ab.py loc logs/bags/<주행> --gains logs/<주행>/gains.yaml --map <맵.yaml> \
    --arms raw mask --mask-max 0.2 --frenet-params /tmp/frenet.yaml --out /tmp/mask
./bag_ab.py report /tmp/mask --map <맵.yaml>
```

```bash
# wall_skip · skew · shift_close — 값마다 label 을 달리해 돌리고 report 로 비교
./bag_ab.py node logs/bags/<주행> --exe frenet_planner --params /tmp/frenet.yaml \
    --label base --out /tmp/frenet
./bag_ab.py node logs/bags/<주행> --exe frenet_planner --params /tmp/frenet.yaml \
    --set raycast_wall_skip=0.0 --label skip0 --out /tmp/frenet
./bag_ab.py report /tmp/frenet
```

```bash
# l1_curv_tmin — map_controller 파라미터를 받아 0 과 0.50 비교
ros2 param dump /map_controller > /tmp/map.yaml
./bag_ab.py node logs/bags/<주행> --exe map_controller --params /tmp/map.yaml \
    --set l1_curv_tmin=0.0 --label tmin0 --out /tmp/tmin
./bag_ab.py node logs/bags/<주행> --exe map_controller --params /tmp/map.yaml \
    --label tmin50 --out /tmp/tmin
./bag_ab.py report /tmp/tmin
```

판정 규칙:

- `loc`: 반복 간 산포의 2배(최소 1㎝)보다 큰 차이만 인정한다. `/scan` 과
  `/scan_deskewed` **양쪽 잣대**에서 같은 방향이고 "잃음" 비율이 늘지 않아야 켠다.
- `node`: open-loop 라 지령이 어떻게 달라지는지만 말한다. `n_obs>0` 이 줄어도 진짜
  장애물까지 놓친 것일 수 있으니, 콘이 있는 q2 bag 에서 그 구간의 회피가 유지되는지
  같이 본다.
- 결론이 안 나면 현재값을 유지하고, 확정값은 bag 이름·결과와 함께 커밋 메시지에 남긴다.
