# 배터리 전압 확인 기능 구현 가이드

> 젯슨에서 직접 구현할 것 — 노트북에서 먼저 수정하면 충돌 날 수 있음.

---

## 목표

1. **`bat.sh`** — 배터리 전압을 단독으로 확인하는 스크립트 (새 파일)
2. **`race.sh`** — 기동 시 배터리 전압 한 줄 출력 (기존 파일 수정)

---

## 구현 내용

### 1. `bat.sh` 신규 생성

`~/yax/bat.sh` 로 생성, `chmod +x` 필수.

```bash
#!/usr/bin/env bash
# ============================================================
#  bat.sh — 배터리 전압 확인
#
#  사용:
#    ./bat.sh              전압 출력 (스택 미실행 시 VESC를 잠깐 띄웠다 닫음)
#    ./bat.sh --inline     race.sh 내부 호출용 (에러 시 경고만, exit 안 함)
# ============================================================
INLINE=${1:-}
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS=${RACE_WS:-$HOME/f1tenth_ws}

# 환경이 아직 안 소싱된 경우를 대비
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash 2>/dev/null || true
[ -f "$WS/install/setup.bash"   ] && source "$WS/install/setup.bash"   2>/dev/null || true

# VESC 심링크 (없으면 만들기)
if [ ! -e /dev/sensors/vesc ]; then
    sudo mkdir -p /dev/sensors
    sudo ln -sf /dev/ttyACM0 /dev/sensors/vesc 2>/dev/null || true
fi

VESC_PID=""
# 스택이 이미 돌고 있으면 토픽 바로 구독, 없으면 vesc_driver 잠깐 띄우기
if ros2 topic info /sensors/core 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    : # 이미 발행 중 — 아래에서 바로 읽음
else
    ros2 run vesc_driver vesc_driver_node --ros-args \
        --params-file "$SCRIPT_DIR/config/vesc.yaml" \
        > /dev/null 2>&1 &
    VESC_PID=$!
    sleep 2.5
fi

VOLT=$(timeout 3 ros2 topic echo /sensors/core --once 2>/dev/null \
    | grep "voltage_input" | awk '{print $2}' | head -1)

# 임시로 띄운 vesc_driver 정리
if [ -n "$VESC_PID" ]; then
    kill "$VESC_PID" 2>/dev/null; wait "$VESC_PID" 2>/dev/null || true
fi

if [ -z "$VOLT" ]; then
    echo "[bat] ⚠ 전압을 읽지 못했습니다 (VESC 연결 확인)"
    [ "$INLINE" = "--inline" ] && exit 0 || exit 1
fi

V=$(printf "%.1f" "$VOLT" 2>/dev/null || echo "$VOLT")

# 전압 기준 (4S LiPo 14.8V 공칭)
#   ≥ 14.8V  초록 — 충분
#   ≥ 12.8V  노랑 — 주의 (곧 충전)
#    < 12.8V  빨강 — 위험 (~11.8V 에서 젯슨 셧다운)
if   awk "BEGIN{exit !($VOLT >= 14.8)}"; then COLOR="\033[32m"
elif awk "BEGIN{exit !($VOLT >= 12.8)}"; then COLOR="\033[33m"
else                                           COLOR="\033[31m"
fi
RESET="\033[0m"

echo -e "[bat] 배터리  ${COLOR}${V} V${RESET}  (충전: ~16.8V  주의: <12.8V  셧다운: ~11.8V)"
```

---

### 2. `race.sh` 수정

`----------1-1. loopback 멀티캐스트----------` 블록 바로 뒤,
`----------2. 모드 결정----------` 시작 전에 아래를 추가한다.

찾을 위치 (약 69번째 줄):
```bash
fi

# ---------- 2. 모드 결정 ----------
```

삽입할 내용:
```bash
fi

# ---------- 1-2. 배터리 전압 ----------
"$SCRIPT_DIR/bat.sh" --inline || true

# ---------- 2. 모드 결정 ----------
```

> `--inline` 플래그: 전압 읽기 실패해도 `exit 0` 처리해서 race.sh 기동을 막지 않는다.

---

## 예상 출력

**기동 시 (race.sh):**
```
[dds] CycloneDDS  |  인터페이스 192.168.45.246 고정  |  피어 1개
[bat] 배터리  15.3 V  (충전: ~16.8V  주의: <12.8V  셧다운: ~11.8V)
============================================
  RoboRacer 2026  |  MODE = q2
  config: /home/jetson/yax/race_script.yaml
============================================
```

**단독 실행:**
```bash
./bat.sh
[bat] 배터리  13.1 V  (충전: ~16.8V  주의: <12.8V  셧다운: ~11.8V)
```
노랑으로 출력되면 슬슬 충전 타이밍.

---

## 주의사항

- **스택이 안 돌고 있을 때** `bat.sh` 는 `vesc_driver_node` 를 2.5초 임시 기동한다.
  `race.sh` 기동 시간이 약 3초 늘어남 — 허용 범위.
- **스택이 이미 돌고 있을 때** (`./race.sh check` 등 별도 터미널) 는 토픽 바로 읽어 빠름.
- `vesc_driver_node` 의 `--params-file` 경로는 `~/yax/config/vesc.yaml` — 이미 있는 파일.
  경로가 다르면 `ros2 param dump /vesc_driver` 로 현재 노드 파라미터 확인.

---

## 구현 후 커밋

```bash
cd ~/yax
chmod +x bat.sh
git add bat.sh race.sh
git commit -m "feat: bat.sh 배터리 전압 확인 + race.sh 기동 시 전압 표시"
git push origin main
```
