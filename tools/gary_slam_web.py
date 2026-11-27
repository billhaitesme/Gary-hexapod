#!/usr/bin/env python3
"""gary_slam_web.py -- LIVE scan-matched map in the browser.

Same idea as gary_web_dashboard.py, but instead of placing each lidar ring at
Gary's reported pose (which is 0,0,0 during EXPLORE, or the drifting gyro heading
otherwise -> a smear), it runs gary_slam's scan-matching to derive his TRUE pose
from the lidar and places rings there. The result holds fixed locations.

    python gary_slam_web.py --source tcp:gary.local:3333
    open http://localhost:8771

Top-down map: green = the scan-matched cloud, blue = his path, dot = where he is.
"Clear" restarts the map from his current spot.
"""
import argparse
import json
import os
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gary_lidar_io import open_source, set_scanner_mode
from gary_slam import Slam, voxel
from gary_odom_fit import Reader


class State:
    def __init__(self, mode):
        self.lock = threading.Lock()
        self.slam = Slam(mode=mode)
        self.mode = mode
        self.connected = False
        self.last_used = "-"

    def snapshot(self):
        with self.lock:
            sl = self.slam
            pts = sl.map_pts
            if len(pts) > 20000:                      # keep the payload sane
                pts = voxel(pts, cell=50.0)
                if len(pts) > 20000:
                    pts = pts[np.random.choice(len(pts), 20000, replace=False)]
            traj = np.array(sl.traj)
            if len(traj) > 800:
                step = int(np.ceil(len(traj) / 800))
                traj = traj[::step]
            return {
                "connected": self.connected,
                "pts": np.round(pts / 10.0, 1).flatten().tolist(),   # mm -> cm, flat [x,y,...]
                "path": np.round(traj / 10.0, 1).flatten().tolist(),
                "pose": [round(sl.pose[0] / 10.0, 1), round(sl.pose[1] / 10.0, 1), round(sl.pose[2], 1)],
                "stats": {"icp": sl.stats["icp"], "fallback": sl.stats["odom_fallback"],
                          "still": sl.stats["still"], "npts": len(sl.map_pts), "used": self.last_used},
            }

    def reset(self):
        with self.lock:
            keep_mode = self.mode
            self.slam = Slam(mode=keep_mode)


def reader_thread(state, args, stop):
    try:
        src = open_source(args.source)
    except Exception as e:
        print("connect failed: %s" % e)
        return
    try:
        set_scanner_mode(src, True)
    except Exception:
        pass
    time.sleep(0.4)
    state.connected = True
    print("connected to %s, scanner ON" % args.source)
    reader = Reader(src)
    try:
        while not stop.is_set():
            any_frame = False
            for pose, dists in reader.frames():
                any_frame = True
                with state.lock:
                    state.last_used = state.slam.step(pose, dists)
            if not any_frame:
                time.sleep(0.01)
    finally:
        try:
            set_scanner_mode(src, False); src.close()
        except Exception:
            pass


PAGE = r"""<!doctype html><html><head><meta charset=utf-8><title>Gary -- scan-matched map</title>
<style>
 html,body{margin:0;background:#0d1117;color:#c9d1d9;font:14px system-ui}
 #bar{padding:8px 12px;display:flex;gap:14px;align-items:center;border-bottom:1px solid #222}
 #bar b{color:#7ee787}
 button{background:#21262d;color:#c9d1d9;border:1px solid #444;border-radius:6px;padding:5px 10px;cursor:pointer}
 button:hover{background:#30363d}
 canvas{display:block}
</style></head><body>
<div id=bar>
  <span>Gary <b>scan-matched</b> map</span>
  <span id=stat>connecting...</span>
  <button onclick="reset()">clear</button>
  <span style="margin-left:auto;color:#666">green = map &nbsp; blue = path &nbsp; dot = Gary</span>
</div>
<canvas id=c></canvas>
<script>
const cv=document.getElementById('c'), cx=cv.getContext('2d');
function fit(){cv.width=innerWidth; cv.height=innerHeight-42;}
addEventListener('resize',fit); fit();
async function reset(){await fetch('/reset',{method:'POST'});}
function draw(d){
  cx.fillStyle='#0d1117'; cx.fillRect(0,0,cv.width,cv.height);
  const P=d.pts, T=d.path;
  let xs=[],ys=[];
  for(let i=0;i<P.length;i+=2){xs.push(P[i]);ys.push(P[i+1]);}
  for(let i=0;i<T.length;i+=2){xs.push(T[i]);ys.push(T[i+1]);}
  if(xs.length<2){requestAnimationFrame(()=>{}); return;}
  let minx=Math.min(...xs),maxx=Math.max(...xs),miny=Math.min(...ys),maxy=Math.max(...ys);
  const pad=40, w=cv.width-2*pad, h=cv.height-2*pad;
  const sp=Math.min(w/Math.max(maxx-minx,50), h/Math.max(maxy-miny,50));
  const cxp=(maxx+minx)/2, cyp=(maxy+miny)/2;
  // world(cm) -> screen; y up
  const X=x=>cv.width/2+(x-cxp)*sp, Y=y=>cv.height/2-(y-cyp)*sp;
  // grid
  cx.strokeStyle='#1b2029'; cx.lineWidth=1;
  for(let g=Math.ceil(minx/100)*100; g<maxx; g+=100){cx.beginPath();cx.moveTo(X(g),0);cx.lineTo(X(g),cv.height);cx.stroke();}
  for(let g=Math.ceil(miny/100)*100; g<maxy; g+=100){cx.beginPath();cx.moveTo(0,Y(g));cx.lineTo(cv.width,Y(g));cx.stroke();}
  // map points
  cx.fillStyle='#2ea043';
  for(let i=0;i<P.length;i+=2){cx.fillRect(X(P[i])-1,Y(P[i+1])-1,2,2);}
  // path
  cx.strokeStyle='#58a6ff'; cx.lineWidth=2; cx.beginPath();
  for(let i=0;i<T.length;i+=2){const sx=X(T[i]),sy=Y(T[i+1]); i?cx.lineTo(sx,sy):cx.moveTo(sx,sy);}
  cx.stroke();
  // robot
  const px=X(d.pose[0]),py=Y(d.pose[1]),th=d.pose[2]*Math.PI/180;
  cx.fillStyle='#f0f6fc'; cx.beginPath(); cx.arc(px,py,5,0,7); cx.fill();
  cx.strokeStyle='#f0f6fc'; cx.beginPath(); cx.moveTo(px,py); cx.lineTo(px+Math.cos(th)*14,py-Math.sin(th)*14); cx.stroke();
}
async function tick(){
  try{
    const d=await (await fetch('/data.json')).json();
    document.getElementById('stat').textContent =
      (d.connected?'live':'offline')+'  |  icp '+d.stats.icp+'  fallback '+d.stats.fallback+
      '  |  pts '+d.stats.npts+'  |  pose '+d.pose[0].toFixed(0)+','+d.pose[1].toFixed(0)+'cm @'+d.pose[2].toFixed(0)+'°';
    draw(d);
  }catch(e){document.getElementById('stat').textContent='(waiting for server)';}
}
setInterval(tick, POLL_MS); tick();
</script></body></html>"""


def make_handler(state, poll_ms):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype):
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path.startswith("/reset"):
                state.reset()
                self._send(b'{"ok":true}', "application/json")
            else:
                self.send_response(404); self.end_headers()

        def do_GET(self):
            if self.path.startswith("/data.json"):
                self._send(json.dumps(state.snapshot()).encode(), "application/json")
            else:
                self._send(PAGE.replace("POLL_MS", str(poll_ms)).encode(), "text/html; charset=utf-8")
    return H


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--source", default="tcp:gary.local:3333")
    ap.add_argument("--port", type=int, default=8771)
    ap.add_argument("--mode", choices=["fuse", "scanmatch"], default="scanmatch",
                    help="scanmatch (default; works during EXPLORE where odom is frozen) "
                         "or fuse (odom translation + scan-match heading, for pad driving)")
    ap.add_argument("--poll-ms", type=int, default=400)
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args()

    state = State(args.mode)
    stop = threading.Event()
    threading.Thread(target=reader_thread, args=(state, args, stop), daemon=True).start()

    srv = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(state, args.poll_ms))
    url = "http://localhost:%d" % args.port
    print("\n  Gary scan-matched map:  %s\n  mode=%s  source=%s\n  Ctrl+C to stop.\n"
          % (url, args.mode, args.source))
    if not args.no_browser:
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        stop.set()


if __name__ == "__main__":
    main()
