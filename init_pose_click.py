#!/usr/bin/env python3
"""init_pose_click.py — 맵을 브라우저에 띄우고 **두 번 클릭**해서 초기 포즈를 잡는다.

    1번째 클릭 → 차를 놓을 위치 (init_x, init_y)
    2번째 클릭 → 차가 바라볼 방향 (그 지점을 향하는 각도가 init_yaw)

왜 웹인가
─────────
맥북에서 SSH 로만 붙어 작업한다. matplotlib GUI(set_init_pose.py --gui)는
DISPLAY 가 필요해서 못 쓴다. 반대로 격자 PNG 를 눈으로 읽어 좌표를 타이핑하는
방식은 되지만 느리고 틀리기 쉽다. HTTP 페이지 하나면 **SSH 만으로 진짜 클릭**이
된다 — 젯슨이 그리고, 브라우저는 클릭 좌표만 돌려준다.

값을 쓰는 것은 **set_init_pose.py 가 한다.** 여기서 또 yaml 을 건드리면
쓰는 곳이 두 군데가 되어 반드시 어긋난다 (이 저장소가 max_steer 로 이미 겪었다).

사용법
──────
    # 젯슨에서
    python3 init_pose_click.py                 # race_script 의 현재 맵
    python3 init_pose_click.py --map 0818_clean --wp-file 0818_clean_free_wp

    # 맥북에서 (SSH 터널 — 같은 망이 아니어도 된다)
    ssh -L 8088:localhost:8088 orin@192.168.50.77
    브라우저에서  http://localhost:8088

    두 번 클릭 → 값 확인 → [race_script.yaml 에 저장] 버튼

옵션
────
    --port N     기본 8088
    --map NAME / --wp-file NAME    기본은 race_script.yaml 의 값
"""
import argparse
import base64
import io
import json
import math
import os
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


def load_map(name):
    base = os.path.join(HERE, 'src', 'race_stack', 'maps', name)
    meta = yaml.safe_load(open(base + '.yaml'))
    from PIL import Image
    img = np.array(Image.open(os.path.join(os.path.dirname(base), meta['image'])))
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    return img, res, ox, oy


def render_png(img, res, ox, oy, wps, cur):
    """맵 + 라인 + 현재 포즈를 PNG(base64) 로. 월드 범위도 같이 돌려준다."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    H, W = img.shape
    ext = [ox, ox + W * res, oy, oy + H * res]
    fig, ax = plt.subplots(figsize=(W / 100, H / 100), dpi=100)
    ax.imshow(np.flipud(img), cmap='gray', origin='lower', extent=ext, vmin=0, vmax=255)
    if wps is not None:
        ax.plot(wps[:, 0], wps[:, 1], '-', color='limegreen', lw=1.6)
    if cur is not None:
        cx, cy, cth = cur
        ax.plot(cx, cy, 'o', color='magenta', ms=7)
        ax.arrow(cx, cy, 0.6 * math.cos(cth), 0.6 * math.sin(cth),
                 head_width=0.15, color='magenta', lw=2)
    ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3])
    ax.set_position([0, 0, 1, 1]); ax.axis('off')     # 여백 0 — 픽셀↔월드가 선형이어야 한다
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=100)
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode(), ext


PAGE = """<!doctype html><meta charset=utf-8><title>init pose</title>
<style>
 body{background:#111;color:#eee;font-family:ui-monospace,monospace;margin:0;padding:12px}
 #wrap{position:relative;display:inline-block;cursor:crosshair}
 img{display:block;max-width:100%%;height:auto}
 svg{position:absolute;left:0;top:0;width:100%%;height:100%%;pointer-events:none}
 #out{margin:10px 0;font-size:15px;line-height:1.7}
 b{color:#6cf} .warn{color:#fa6}
 button{font:inherit;padding:8px 14px;margin-right:8px;cursor:pointer}
</style>
<h3 style="margin:4px 0 10px">%(map)s — 1번째 클릭 = 위치, 2번째 클릭 = 바라볼 방향</h3>
<div id=wrap><img id=m src="data:image/png;base64,%(png)s"><svg id=ov></svg></div>
<div id=out>클릭을 기다리는 중…</div>
<button onclick="reset()">다시 찍기</button>
<button id=save onclick="save()" disabled>race_script.yaml 에 저장</button>
<script>
const EXT=%(ext)s;           // [x0,x1,y0,y1]
let p1=null,p2=null;
const im=document.getElementById('m'),ov=document.getElementById('ov');
function toWorld(e){
  const r=im.getBoundingClientRect();
  const fx=(e.clientX-r.left)/r.width, fy=(e.clientY-r.top)/r.height;
  return [EXT[0]+fx*(EXT[1]-EXT[0]), EXT[3]-fy*(EXT[3]-EXT[2])];
}
function toPct(w){return [100*(w[0]-EXT[0])/(EXT[1]-EXT[0]),100*(EXT[3]-w[1])/(EXT[3]-EXT[2])];}
function draw(){
  let s='';
  if(p1){const a=toPct(p1);s+=`<circle cx="${a[0]}%%" cy="${a[1]}%%" r="6" fill="#f0f"/>`;}
  if(p1&&p2){const a=toPct(p1),b=toPct(p2);
    s+=`<line x1="${a[0]}%%" y1="${a[1]}%%" x2="${b[0]}%%" y2="${b[1]}%%" stroke="#f0f" stroke-width="3"/>`;}
  ov.innerHTML=s;
}
document.getElementById('wrap').onclick=e=>{
  const w=toWorld(e);
  if(!p1||p2){p1=w;p2=null;} else {p2=w;}
  draw();
  if(p1&&!p2){document.getElementById('out').innerHTML=
     `위치 <b>x=${p1[0].toFixed(4)}  y=${p1[1].toFixed(4)}</b><br>이제 <b>바라볼 방향</b>을 클릭하세요`;
   document.getElementById('save').disabled=true;}
  else if(p2){const yaw=Math.atan2(p2[1]-p1[1],p2[0]-p1[0]);
   window._pose={x:p1[0],y:p1[1],yaw:yaw};
   document.getElementById('out').innerHTML=
     `init_x <b>${p1[0].toFixed(4)}</b>　init_y <b>${p1[1].toFixed(4)}</b>　`+
     `init_yaw <b>${yaw.toFixed(4)}</b> rad (<b>${(yaw*180/Math.PI).toFixed(1)}°</b>)`;
   document.getElementById('save').disabled=false;}
};
function reset(){p1=p2=null;draw();document.getElementById('out').textContent='클릭을 기다리는 중…';
  document.getElementById('save').disabled=true;}
function save(){
  fetch('/apply',{method:'POST',body:JSON.stringify(window._pose)})
   .then(r=>r.text()).then(t=>{document.getElementById('out').innerHTML=
     '<span class=warn>'+t.replace(/\\n/g,'<br>')+'</span>';});
}
</script>"""


def main():
    cfg_path = os.path.join(HERE, 'race_script.yaml')
    cfg = yaml.safe_load(open(cfg_path))
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=cfg.get('map_name'))
    ap.add_argument('--wp-file', default=cfg.get('waypoints'), dest='wp')
    ap.add_argument('--port', type=int, default=8088)
    args = ap.parse_args()

    img, res, ox, oy = load_map(args.map)
    wps = None
    wpp = os.path.join(HERE, 'src', 'race_stack', 'waypoints', str(args.wp) + '.csv')
    if os.path.exists(wpp):
        wps = np.loadtxt(wpp, delimiter=',', skiprows=1)
    loc = cfg.get('localization', {})
    cur = (float(loc.get('init_x', 0)), float(loc.get('init_y', 0)),
           float(loc.get('init_yaw', 0)))
    png, ext = render_png(img, res, ox, oy, wps, cur)
    page = (PAGE % {'png': png, 'ext': json.dumps([round(v, 6) for v in ext]),
                    'map': args.map}).encode()

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(page)))
            self.end_headers()
            self.wfile.write(page)

        def do_POST(self):
            n = int(self.headers.get('Content-Length', 0))
            d = json.loads(self.rfile.read(n) or b'{}')
            # ★ 쓰는 것은 set_init_pose.py 한 곳에서만 한다.
            cmd = [sys.executable, os.path.join(HERE, 'set_init_pose.py'),
                   '--map', args.map, '--x', f"{d['x']:.4f}", '--y', f"{d['y']:.4f}",
                   '--yaw', f"{d['yaw']:.4f}", '--write']
            r = subprocess.run(cmd, capture_output=True, text=True)
            out = (r.stdout + r.stderr).strip() or '(출력 없음)'
            print(out)
            body = out.encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/plain; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    print(f"맵 {args.map}   웨이포인트 {args.wp}")
    print(f"월드 범위 x {ext[0]:.2f}~{ext[1]:.2f}   y {ext[2]:.2f}~{ext[3]:.2f}")
    print()
    print(f"  맥북에서:  ssh -L {args.port}:localhost:{args.port} {os.environ.get('USER','orin')}@<젯슨IP>")
    print(f"  브라우저:  http://localhost:{args.port}")
    print()
    print("Ctrl+C 로 종료")
    HTTPServer(('127.0.0.1', args.port), H).serve_forever()


if __name__ == '__main__':
    main()
