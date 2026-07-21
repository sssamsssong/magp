#!/usr/bin/env bash
# ============================================================
#  tune.sh — 주행 게인 튜닝 (race_script.yaml 값만 안전하게 수정)
#
#  주행 → RViz 관찰 → 게인 수정 → 재주행 루프용.
#  race.launch.py 가 매 실행마다 yaml 을 새로 읽으므로,
#  값만 바꾸면 다음 ./race.sh q2 에 바로 반영된다 (재빌드 불필요).
#
#  사용법:
#    ./tune.sh status                   ★ 현재 튜닝 상태 — 조절 우선순위 + 커밋 대비
#                                         변경분 + 최근 변경 이력
#                                         (우선순위 출처: docs/stanley_gain_tuning.md)
#    ./tune.sh                          설정값 전체 보기
#    ./tune.sh stanley                  한 섹션만 보기
#    ./tune.sh stanley.gain             한 값 조회
#    ./tune.sh stanley.gain=1.2         설정
#    ./tune.sh stanley.gain=1.2 stanley.k=2.0    여러 개 한 번에
#    ./tune.sh stanley.gain+=0.2        상대 조정 (+= / -=)
#    ./tune.sh --diff                   마지막 커밋 대비 변경분
#    ./tune.sh --log                    이번 튜닝 세션의 변경 이력
#    ./tune.sh --undo                   직전 변경 되돌리기
#    ./tune.sh --force ...              안전 가드 무시하고 강제 적용
#
#  노트북에서 한 줄로:
#    ssh jetson "cd ~/yax && ./tune.sh stanley.gain=1.2"
# ============================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
exec python3 "$SCRIPT_DIR/tune.py" "$@"
