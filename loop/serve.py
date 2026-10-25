"""Local server for the results pages and the live demo.

Serves the repository as static files, plus one endpoint for a live flight:
    GET /api/fly?artifact=runs/d1/honest/round2/controller.js[&seed=N]
runs that controller on one trial through the trusted judge (the controller stays in its locked-down child process),
records the flight, and returns {seed, result, track}. Without a seed it draws a fresh random one. Seeds the loops
could have studied (the 16 validation seeds and the 4 smoke seeds) are refused, so a live flight is always new.

    python loop/serve.py --port 8777
"""
import argparse, json, pathlib, re, secrets, subprocess, sys, time
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from kit import ROOT, TASKS, node_env

LIVE = ROOT / "tmp" / "live"
TASK_SRC = (ROOT / "tasks/drone/task.mjs").read_text()
STUDIED = {int(x) for x in re.findall(r"\d{3,}", TASK_SRC[TASK_SRC.index("SPLITS"):TASK_SRC.index("};", TASK_SRC.index("SPLITS"))])}


def fly(artifact, seed):
    path = (ROOT / artifact).resolve()
    if not str(path).startswith(str(ROOT / "runs")) and not str(path).startswith(str(ROOT / "tasks/drone/baselines")):
        return 400, {"error": "artifact must be a controller under runs/ or the drone baselines"}
    if not path.is_file() or path.suffix != ".js":
        return 404, {"error": "no such controller"}
    if seed is None:
        seed = 100000 + secrets.randbelow(4_000_000_000)
    if seed in STUDIED:
        return 400, {"error": f"seed {seed} is one the loops studied; pick a new one"}
    out = LIVE / f"{int(time.time() * 1000)}"
    out.mkdir(parents=True, exist_ok=True)
    T = TASKS["drone"]
    r = subprocess.run(T["run"] + [str(path), "--seeds", str(seed), "--record", str(out), "--json", str(out / "result.json")],
                       cwd=T["cwd"], env=node_env(), capture_output=True, text=True, timeout=120)
    if r.returncode != 0 and not (out / "result.json").exists():
        return 500, {"error": (r.stderr or r.stdout)[-400:]}
    res = json.loads((out / "result.json").read_text())
    result = res["results"][0] if res.get("results") else {"success": False, "reason": res.get("error", "invalid")}
    track = out / f"seed-{seed}.json"
    return 200, {"seed": seed, "result": result, "track": "/" + track.relative_to(ROOT).as_posix() if track.exists() else None}


class Handler(SimpleHTTPRequestHandler):
    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/api/fly":
            q = parse_qs(u.query)
            try:
                seed = int(q["seed"][0]) if q.get("seed", [""])[0] else None
            except ValueError:
                return self._json(400, {"error": "seed must be a whole number"})
            return self._json(*fly(q.get("artifact", [""])[0], seed))
        if u.path.startswith(("/private", "/.git", "/tmp/") ) and not u.path.startswith("/tmp/live/"):
            return self._json(404, {"error": "not served"})
        return super().do_GET()

    def _json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("content-type", "application/json")
        self.send_header("cache-control", "no-store")
        self.send_header("content-length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def end_headers(self):
        self.send_header("cache-control", "no-store")
        super().end_headers()

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8777)
    a = ap.parse_args()
    ThreadingHTTPServer(("127.0.0.1", a.port), partial(Handler, directory=str(ROOT))).serve_forever()
