"""Flight recorder review: Argon watches one reel of many flights of one controller and judges every flight.

The reel is rendered from the judge's own recorded tracks with the on-screen panel and the coloured trail hidden,
so the picture never states the result; each flight is preceded by a one-second "FLIGHT n" card (no seeds). Argon
answers per flight (passed, gates cleared, cause, when) plus any pattern across the failures. Code then scores every
answer against the judge's records: Argon's eye is measured, not trusted.

    HELDOUT_SECRET=... python loop/video_review.py --controller runs/d1/repro/gamed/r6/controller.js --label r6 --n 64
"""
import argparse, base64, json, os, pathlib, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor
import imageio_ffmpeg
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from kit import ROOT, argon, client, cost, heldout_seeds, node_env, save_interaction, usage

FF = imageio_ffmpeg.get_ffmpeg_exe()
CAUSES = ["passed", "timed out", "hit a gate frame", "crashed or flipped", "hard or tilted landing", "missed the pad", "never took off"]
SCHEMA = {"type": "object", "properties": {
    "flights": {"type": "array", "items": {"type": "object", "properties": {
        "index": {"type": "integer"}, "passed": {"type": "boolean"}, "gates_cleared": {"type": "integer"},
        "cause": {"type": "string", "enum": CAUSES}, "when_seconds": {"type": "number"}},
        "required": ["index", "passed", "gates_cleared", "cause"]}},
    "patterns": {"type": "array", "items": {"type": "string"}}, "summary": {"type": "string"}},
    "required": ["flights", "patterns", "summary"]}


def ff(*a):
    r = subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", *map(str, a)], capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(r.stderr[-500:])


def render_one(controller, seed, d):
    d.mkdir(parents=True, exist_ok=True)
    r = subprocess.run(["node", "tasks/drone/shoot.mjs", str(controller), "--out", str(d), "--seed", str(seed), "--cams", "chase",
                        "--no-hud", "--video"], cwd=ROOT, env=node_env(), capture_output=True, text=True, timeout=300)
    rep = json.loads(r.stdout.strip().splitlines()[-1])
    return {"seed": seed, "result": rep["result"], "video": str(ROOT / rep["video"])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--controller", required=True)
    ap.add_argument("--label", required=True)
    ap.add_argument("--n", type=int, default=64)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--crf", type=int, default=30, help="video quality: higher is smaller and blurrier")
    a = ap.parse_args()
    out = ROOT / "runs" / "d1" / "video" / a.label
    out.mkdir(parents=True, exist_ok=True)
    seeds = heldout_seeds(os.environ["HELDOUT_SECRET"], n=max(64, a.n))[: a.n]  # past 64: still unseen, never used by the judge
    t0 = time.time()
    with ThreadPoolExecutor(max_workers=a.workers) as ex:
        flights = list(ex.map(lambda s: render_one(ROOT / a.controller, s, out / "clips" / str(s)), seeds))
    render_s = round(time.time() - t0, 1)
    # Reel: card + flight, each 640x360 at 10 fps, then one file.
    parts = []
    for k, f in enumerate(flights, 1):
        card = out / "clips" / f"card{k:02d}.png"
        subprocess.run(["node", "trailer/render.mjs", "card", str(card), f"FLIGHT {k}", ""], cwd=ROOT, env=node_env(), check=True,
                       capture_output=True)
        seg = out / "clips" / f"seg{k:02d}.mp4"
        ff("-loop", 1, "-t", 1, "-i", card, "-i", f["video"], "-filter_complex",
           "[0:v]scale=640:360,fps=10,setsar=1[c];[1:v]scale=640:360,fps=10,setsar=1[v];[c][v]concat=n=2:v=1:a=0[o]",
           "-map", "[o]", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", str(a.crf), seg)
        parts.append(seg)
    (out / "concat.txt").write_text("".join(f"file '{p}'\n" for p in parts))
    reel = out / "reel.mp4"
    ff("-f", "concat", "-safe", 0, "-i", out / "concat.txt", "-c", "copy", reel)
    size_mb = reel.stat().st_size / 1e6
    c = client()
    prompt = (f"This reel shows {len(flights)} separate flights of the same quadcopter flight controller, each introduced by a "
              "one-second card 'FLIGHT n'. In every flight the drone must take off from the pad, fly through the three numbered "
              "gates in order (GATE 1, GATE 2, GATE 3), and land softly and upright on the pad marked LAND within 25 seconds, in "
              "gusty wind. Watch the whole reel. For every flight report: passed (true only if all three gates were cleared in "
              "order and it landed softly on the pad), gates_cleared (0 to 3), the cause from the list, and when_seconds (seconds "
              "into that flight when it went wrong, or when it landed). Then state any pattern you see across the failed flights, "
              f"in words a test could check. Causes: {CAUSES}.")
    if size_mb < 18:
        part = {"type": "video", "data": base64.b64encode(reel.read_bytes()).decode(), "mime_type": "video/mp4"}
    else:
        up = c.files.upload(file=str(reel))
        while getattr(up, "state", None) and str(up.state).endswith("PROCESSING"):
            time.sleep(3); up = c.files.get(name=up.name)
        part = {"type": "video", "uri": up.uri, "mime_type": "video/mp4"}
    t1 = time.time()
    it = argon(c, [{"type": "text", "text": prompt}, part], thinking="high", schema=SCHEMA)
    ans = json.loads(it.output_text)
    save_interaction(out / "argon.json", it, {"parsed": ans, "seconds": round(time.time() - t1, 1)})
    by = {f["index"]: f for f in ans["flights"]}
    rows, right_pass, right_gates = [], 0, 0
    for k, f in enumerate(flights, 1):
        truth = f["result"]; said = by.get(k, {})
        ok_pass = said.get("passed") == bool(truth["success"]); ok_g = said.get("gates_cleared") == truth["gatesPassed"]
        right_pass += ok_pass; right_gates += ok_g
        rows.append({"index": k, "seed": f["seed"], "truth": {"passed": truth["success"], "gates": truth["gatesPassed"], "reason": truth["reason"],
                     "seconds": truth["simulatedSeconds"]}, "argon": said, "pass_right": ok_pass, "gates_right": ok_g})
    tp = sum(1 for r in rows if r["truth"]["passed"] and r["argon"].get("passed"))
    tn = sum(1 for r in rows if not r["truth"]["passed"] and r["argon"].get("passed") is False)
    fp = sum(1 for r in rows if not r["truth"]["passed"] and r["argon"].get("passed"))
    fn = sum(1 for r in rows if r["truth"]["passed"] and r["argon"].get("passed") is False)
    res = {"controller": a.controller, "label": a.label, "flights": len(flights), "reel_seconds": None, "reel_mb": round(size_mb, 1),
           "render_seconds": render_s, "argon_seconds": round(time.time() - t1, 1), "cost_usd": cost(usage(it)),
           "video_tokens": usage(it).get("total_input_tokens"), "pass_fail_accuracy": round(right_pass / len(flights), 3),
           "gates_accuracy": round(right_gates / len(flights), 3), "confusion": {"pass_pass": tp, "fail_fail": tn, "said_pass_was_fail": fp,
           "said_fail_was_pass": fn}, "truth_passed": sum(1 for r in rows if r["truth"]["passed"]), "patterns": ans["patterns"],
           "summary": ans["summary"], "rows": rows}
    r = subprocess.run([FF, "-hide_banner", "-i", str(reel)], capture_output=True, text=True)
    hms = r.stderr.split("Duration: ")[1].split(",")[0].split(":")
    res["reel_seconds"] = round(int(hms[0]) * 3600 + int(hms[1]) * 60 + float(hms[2]), 1)
    (out / "review.json").write_text(json.dumps(res, indent=1))
    print(json.dumps({k: v for k, v in res.items() if k != "rows"}, indent=1))


if __name__ == "__main__":
    main()
