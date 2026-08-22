#!/usr/bin/env python3
"""statusline.py — 주행 로그는 위로 흘리고, 현재 랩/단계는 **맨 아래 줄에 고정**.

    ros2 launch ... 2>&1 | python3 statusline.py

왜 필요한가
───────────
mission_manager 가 2초마다 찍는 진행 배너([Q1 2/3] 단계 1/5 …)가 다른 노드
로그에 밀려 화면 밖으로 올라간다. 대회장에서 "지금 Q 몇이고 몇 바퀴째냐"를
보려고 스크롤을 뒤지게 된다. 그 한 줄만 떼어 맨 아래에 붙여 둔다.

어떻게
──────
ANSI 스크롤 영역(DECSTBM)으로 **맨 아랫줄을 스크롤에서 제외**한다.
위쪽 영역은 평소처럼 흐르고, 아랫줄은 우리가 직접 다시 그린다.

    ESC[1;{H-1}r     스크롤 영역을 1~H-1 로
    ESC7 … ESC8      커서 저장/복원 (로그 출력 위치를 안 흔들려고)

터미널이 아니면(파이프·리다이렉트·비대화형 ssh) **아무것도 안 하고 그대로
흘려보낸다.** 그래야 로그 파일이나 tee 가 깨지지 않는다.

주의
────
· 파이프를 물리면 파이썬 노드 stdout 이 블록 버퍼링으로 바뀐다.
  race.sh 가 PYTHONUNBUFFERED=1 / RCUTILS_LOGGING_BUFFERED_STREAM=0 을
  export 한다. 그게 없으면 로그가 뭉텅이로 늦게 나온다.
· 종료 시 스크롤 영역을 반드시 되돌린다. 안 그러면 셸 프롬프트가 화면
  아래쪽에 갇힌다 (그때는 `reset` 으로 푼다).
"""
import os
import re
import signal
import sys
import unicodedata

# [INFO] [1787327971.9] [mission_manager]: 본문
LOG_RE = re.compile(r'\[(?:INFO|WARN|ERROR)\]\s*\[[\d.]+\]\s*\[([^\]]+)\]:\s*(.*)$')
# 진행 배너: "[Q1 2/3] 단계 1/5  이번 랩 8.3s  회피 off  grip_util 0.50"
BANNER_RE = re.compile(r'^(\[[^\]]+\]\s*)?단계\s*\d+/\d+')
# 랩 완주:   "[Q2 1/3] LAP 1 완료! (12.43s)"
LAP_RE = re.compile(r'LAP\s+(\d+)\s*완료!\s*\(([\d.]+)s\)')
# 단계 전환: "════════ 다음 단계 — Q2 (3/5, 3랩, 회피 ON) ════════"
STAGE_RE = re.compile(r'다음 단계\s*—\s*(.+?)\s*════')
# 출발
GO_RE = re.compile(r'════════\s*출발!')

CSI = '\033['
SAVE, REST = '\0337', '\0338'


class Bar:
    def __init__(self, stream):
        self.out = stream
        self.rows, self.cols = self._size()
        self.text = '대기 중 — 출발 버튼을 누르세요'
        self.last_lap = ''
        self.armed = False

    def _size(self):
        """(행, 열). 못 재거나 말이 안 되는 값이면 안전한 기본값으로.

        ★ 0 이나 음수가 나오는 경우가 실제로 있다 — 크기를 안 정한 pty,
          일부 ssh 클라이언트, tmux 재부착 직후. 그대로 쓰면
          ESC[1;-1r 같은 제어열을 뱉어 터미널이 망가진다.
        """
        for fd in (getattr(self.out, 'fileno', lambda: None)(), 1, 2):
            if fd is None:
                continue
            try:
                sz = os.get_terminal_size(fd)
            except (OSError, ValueError):
                continue
            if sz.lines >= 5 and sz.columns >= 20:
                return sz.lines, sz.columns
        return 24, 80

    def enable(self):
        # 맨 아랫줄을 스크롤 영역에서 뺀다. 커서는 위쪽 영역 맨 아래로.
        self.out.write(f'{SAVE}{CSI}1;{self.rows - 1}r{REST}')
        self.out.write(f'{CSI}{self.rows - 1};1H\n')
        self.armed = True
        self.draw()

    def disable(self):
        if not self.armed:
            return
        self.armed = False
        # 스크롤 영역 복원 + 아랫줄 지우고 커서를 거기로
        self.out.write(f'{SAVE}{CSI}1;{self.rows}r{REST}')
        self.out.write(f'{CSI}{self.rows};1H{CSI}2K')
        self.out.flush()

    def resize(self, *_):
        self.rows, self.cols = self._size()
        if self.armed:
            self.out.write(f'{SAVE}{CSI}1;{self.rows - 1}r{REST}')
            self.draw()

    @staticmethod
    def _dw(text: str) -> int:
        """터미널에서 차지하는 칸 수. 한글·기호는 2칸이다.

        len() 으로 자르면 한글이 섞인 줄에서 반전 배경이 넘치거나 모자란다.
        """
        return sum(2 if unicodedata.east_asian_width(c) in 'WF' else 1
                   for c in text)

    def _fit(self, text: str, width: int) -> str:
        """표시 폭 기준으로 자른다 (문자 수가 아니라)."""
        if self._dw(text) <= width:
            return text
        out, w = [], 0
        for c in text:
            cw = 2 if unicodedata.east_asian_width(c) in 'WF' else 1
            if w + cw > width:
                break
            out.append(c)
            w += cw
        return ''.join(out)

    def draw(self):
        if not self.armed:
            return
        body = self.text + (('   |   ' + self.last_lap) if self.last_lap else '')
        avail = max(self.cols - 2, 8)          # 양옆 여백 1칸씩
        body = self._fit(body, avail)
        pad = ' ' * max(0, avail - self._dw(body))
        self.out.write(
            f'{SAVE}{CSI}{self.rows};1H{CSI}2K'
            f'{CSI}7m {body}{pad} {CSI}0m{REST}')
        self.out.flush()


def main():
    out = sys.stdout
    # 터미널이 아니면 그대로 흘려보낸다 (tee·리다이렉트·비대화형 ssh)
    if not out.isatty() or os.environ.get('RACE_STATUSLINE') == '0':
        for line in sys.stdin:
            out.write(line)
            out.flush()
        return

    bar = Bar(out)
    try:
        signal.signal(signal.SIGWINCH, bar.resize)
    except (AttributeError, ValueError):
        pass
    bar.enable()
    try:
        for line in sys.stdin:
            raw = line.rstrip('\n')
            m = LOG_RE.search(raw)
            node, msg = (m.group(1), m.group(2)) if m else ('', raw)

            if node == 'mission_manager' or not m:
                if BANNER_RE.match(msg):
                    # 이 줄은 **화면에 안 흘리고** 아랫줄로만 보낸다.
                    # 2초마다 오므로 그대로 두면 다른 로그를 덮는다.
                    bar.text = msg
                    bar.draw()
                    continue
                lm = LAP_RE.search(msg)
                if lm:
                    bar.last_lap = f'최근 LAP {lm.group(1)} · {lm.group(2)}s'
                    bar.draw()
                sm = STAGE_RE.search(msg)
                if sm:
                    bar.text = f'▶ {sm.group(1)}'
                    bar.last_lap = ''
                    bar.draw()
                elif GO_RE.search(msg):
                    bar.text = '출발!'
                    bar.draw()

            out.write(line)
            out.flush()
            bar.draw()          # 로그가 지나가며 아랫줄을 건드렸을 수 있다
    except KeyboardInterrupt:
        pass
    finally:
        bar.disable()


if __name__ == '__main__':
    main()
