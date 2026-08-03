#!/usr/bin/env bash
# ============================================================
#  dds_env.sh — DDS(미들웨어) 환경변수 설정. race.sh / rviz.sh 가 source 한다.
#
#  하는 일:
#    1) race_script.yaml 의 network: 섹션을 읽는다
#    2) 주행망 대역에서 이 기기의 IP 를 찾는다
#    3) config/cyclonedds.xml.in 템플릿을 채워 /tmp 에 실제 XML 을 만든다
#    4) RMW_IMPLEMENTATION / CYCLONEDDS_URI 를 export 한다
#
#  ★ 젯슨과 노트북의 RMW 가 다르면 통신이 0 이 된다. 그래서 대역을 못 찾아도
#    RMW 를 되돌리지 않고, 인터페이스 고정만 포기한 채 CycloneDDS 를 유지한다.
#    (오늘처럼 인터넷용 WiFi 에 붙어 있어도 양쪽이 계속 통신되게)
# ============================================================

# 이 파일의 위치 = repo/config
_DDS_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
_DDS_YAML="${RACE_SCRIPT:-$_DDS_DIR/../race_script.yaml}"
_DDS_TPL="$_DDS_DIR/cyclonedds.xml.in"

_dds_yaml_get () {   # $1=키경로(점 구분)  $2=기본값
    python3 - "$_DDS_YAML" "$1" "$2" <<'PY' 2>/dev/null || echo "$3"
import sys, yaml
path, key, default = sys.argv[1], sys.argv[2], sys.argv[3]
try:
    d = yaml.safe_load(open(path))
    for k in key.split('.'):
        d = d[k]
    if isinstance(d, list):
        print('\n'.join(str(x) for x in d))
    else:
        print(d)
except Exception:
    print(default)
PY
}

setup_dds () {
    [ -f "$_DDS_YAML" ] || return 0

    local dds; dds=$(_dds_yaml_get 'network.dds' 'fastdds')
    case "$dds" in
        cyclone|cyclonedds) ;;
        *) return 0 ;;   # 기본 FastDDS — 아무것도 안 건드림
    esac

    if [ ! -e /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]; then
        echo "[dds] ⚠ CycloneDDS 미설치 — FastDDS 로 진행합니다."
        echo "      sudo apt install ros-humble-rmw-cyclonedds-cpp   (젯슨·노트북 양쪽 다)"
        return 0
    fi

    # ros2 CLI 데몬은 처음 뜰 때의 RMW 를 계속 물고 있다. RMW 를 바꾼 뒤에도
    # 옛 데몬이 살아 있으면 `ros2 topic list` 가 예전 미들웨어 기준의 목록을
    # 돌려줘 "바꿨는데 그대로 보인다"는 착각을 만든다 (실제로 디버깅 중 속았음).
    # 데몬은 필요할 때 자동으로 다시 뜨므로 그냥 내려둔다.
    if [ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ]; then
        ros2 daemon stop >/dev/null 2>&1 || true
    fi
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

    local subnet; subnet=$(_dds_yaml_get 'network.subnet' '192.168.50.')
    local peers;  peers=$(_dds_yaml_get 'network.peers' '')

    # 주행망 대역에서 내 IP 찾기
    local myip; myip=$(ip -4 -br addr 2>/dev/null \
        | grep -oE "${subnet//./\\.}[0-9]+" | head -1)

    if [ -z "$myip" ]; then
        # 주행망이 아님 (예: 인터넷용 WiFi). 인터페이스 고정과 유니캐스트 피어를
        # 포기하고 멀티캐스트 자동 탐색으로 둔다. RMW 는 그대로 Cyclone 유지 —
        # 여기서 FastDDS 로 되돌리면 상대 기기와 RMW 가 어긋나 통신이 끊긴다.
        unset CYCLONEDDS_URI
        echo "[dds] CycloneDDS (자동 탐색) — ${subnet}x 대역을 못 찾았습니다."
        echo "      주행 내부망에서는 인터페이스 고정 + 유니캐스트로 자동 전환됩니다."
        return 0
    fi

    [ -f "$_DDS_TPL" ] || { echo "[dds] ⚠ 템플릿 없음: $_DDS_TPL"; return 0; }

    # 피어 목록을 XML 조각으로 (자기 자신은 빼도 무해하므로 그대로 둔다)
    local peer_xml=""
    local p
    while IFS= read -r p; do
        [ -n "$p" ] && peer_xml+="        <Peer address=\"$p\"/>"$'\n'
    done <<< "$peers"

    local out="/tmp/cyclonedds-${myip//./_}.xml"
    # 템플릿 치환 — @PEERS@ 는 여러 줄이라 awk 로 넣는다
    awk -v ip="$myip" -v peers="$peer_xml" '
        { gsub(/@LOCAL_IP@/, ip)
          if ($0 ~ /@PEERS@/) { printf "%s", peers; next }
          print }
    ' "$_DDS_TPL" > "$out" || { echo "[dds] ⚠ XML 생성 실패"; return 0; }

    export CYCLONEDDS_URI="file://$out"
    echo "[dds] CycloneDDS  |  인터페이스 $myip 고정  |  피어 $(echo "$peers" | grep -c .)개"
}

setup_dds
