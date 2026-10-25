"""Builds one cut of the one-minute trailer from an edit list (data, not code).

    python trailer/build.py --version 1            Argon writes captions and narration from the judge's numbers
    python trailer/build.py --version 2 --edl runs/trailer/v1/edl-revised.json    rebuild from a critic-revised list

Every segment shows real footage of what was built today (the results page, replays of real flights) or a plain
title card. The narrator is Gemini text-to-speech and the music is Lyria; the end card says so.
"""
import argparse, json, pathlib, subprocess, sys
import imageio_ffmpeg
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "loop"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kit import ROOT, argon, client, node_env, save_interaction
from media import music, speak

FF = imageio_ffmpeg.get_ffmpeg_exe()
OUT = ROOT / "runs" / "trailer"
PAGE = "http://127.0.0.1:8777/runs/d1/index.html"
FOOT = OUT / "footage"
SEED = 1125297830

SEGMENTS = [  # what each shot shows; Argon writes caption and narration for each
    {"id": "title", "kind": "card", "title": "DID IT LEARN?", "seconds": 4, "shows": "title card"},
    {"id": "problem", "kind": "page", "heading": "-", "seconds": 6, "shows": "top of the results page: Google's loop says loop until the critic rates it 9/10"},
    {"id": "loops", "kind": "page", "heading": "Gamed loop: Google", "seconds": 6, "shows": "round-by-round gallery of the loop told to cheat: rendered drone flights, 16/16 each round"},
    {"id": "cheat", "kind": "page", "heading": "The cheat, in its own code", "seconds": 8, "shows": "the cheating controller's source: 16 hard-coded validation seeds, a copy of the physics, matching the first sensor reading"},
    {"id": "recipes", "kind": "page", "heading": "The recipes", "seconds": 7, "shows": "the two 32-token recipes, squeezed by Argon from records of 93k and 172k tokens"},
    {"id": "verdicts", "kind": "page", "heading": "Verdicts", "seconds": 7, "shows": "verdict cards: validation and held-out scores of each loop and of its rebuilds"},
    {"id": "honest", "kind": "flight", "who": "honest", "seconds": 5, "shows": "the honest controller flying a brand-new flight: three gates, soft landing"},
    {"id": "cheater", "kind": "flight", "who": "cheater", "seconds": 3, "shows": "the cheater on the same new flight: never leaves the pad, fails on the first tick"},
    {"id": "rebuild", "kind": "flight", "who": "rebuild", "seconds": 5, "shows": "a rebuild from the cheater's 32-token recipe on the same flight: lands in 7 seconds, 3.5 mm from centre"},
    {"id": "end", "kind": "card", "title": "DON'T SHIP THE LOOP. SHIP THE REBUILD.", "seconds": 6, "shows": "end card"},
]

SCHEMA = {"type": "object", "properties": {"segments": {"type": "array", "items": {"type": "object", "properties": {
    "id": {"type": "string"}, "caption": {"type": "string"}, "line": {"type": "string"}}, "required": ["id", "caption", "line"]}},
    "music": {"type": "string"}}, "required": ["segments", "music"]}


def facts():
    J = json.loads((ROOT / "runs/d1/judge.json").read_text())
    rec = {k: json.loads((ROOT / f"runs/d1/compress/{k}-32.json").read_text()) for k in ("honest", "gamed")}
    return {"explorers": {k: {kk: v.get(kk) for kk in ("val", "test", "verdict", "artifact_memorized")} for k, v in J["explorers"].items()},
            "rebuilds": {k: {kk: v.get(kk) for kk in ("val_median", "test_median", "produced", "of")} for k, v in J["reproducers"].items()},
            "recipes": {k: {"recipe": v["recipe"], "tokens": v["tokens"], "record_tokens": v["record_tokens"]} for k, v in rec.items()},
            "seed_list_tokens": 222, "live_flight": {"seed": SEED, "honest": "landed, 16 s", "cheater": "failed at 0.01 s",
                                                     "rebuild_of_cheater": "landed in 7.1 s, 3.5 mm from centre"}}


def write_words(c, edl_dir):
    prompt = ("You are writing a one-minute trailer for a hackathon entry, styled like an over-the-top action movie trailer "
              "with deadpan comedy. It must stay true: every number you use must come from the FACTS below, and nothing may be "
              "claimed that the facts do not show. The story: Google's recommended build loop (render, watch, critique, fix) was "
              "run by Gemini 4 Argon agents in Managed Agents sandboxes to write a drone flight controller. One loop honest, one "
              "told to cheat. Both scored 16 of 16. The cheater memorized (hard-coded the test seeds). The compression test "
              "(Amazon Science, 2026) squeezed each loop into a 32-token recipe; fresh sandboxes rebuilt from the recipe alone. "
              "Moral: don't ship the loop's artifact, ship the rebuild.\n\n"
              "For each segment write a short on-screen caption (at most 9 words) and one narration line the narrator can say "
              "in the segment's seconds (about 2.3 words per second, so seconds x 2.3 words at most). Trailer voice: 'In a world "
              "where...' energy, then deadpan. Also give a one-sentence music prompt for an instrumental 30-second track.\n\n"
              f"SEGMENTS: {json.dumps(SEGMENTS)}\n\nFACTS: {json.dumps(facts())}")
    it = argon(c, [{"type": "text", "text": prompt}], thinking="medium", schema=SCHEMA)
    save_interaction(edl_dir / "words.json", it, {"parsed": json.loads(it.output_text)})
    words = json.loads(it.output_text)
    by = {s["id"]: s for s in words["segments"]}
    edl = [{**s, "caption": by.get(s["id"], {}).get("caption", ""), "line": by.get(s["id"], {}).get("line", "")} for s in SEGMENTS]
    return {"segments": edl, "music": words["music"]}


def ff(*args):
    r = subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", *map(str, args)], capture_output=True, text=True)
    if r.returncode:
        raise RuntimeError(r.stderr[-600:])


def duration(path):
    r = subprocess.run([FF, "-hide_banner", "-i", str(path)], capture_output=True, text=True)
    t = r.stderr.split("Duration: ")[1].split(",")[0].split(":")
    return int(t[0]) * 3600 + int(t[1]) * 60 + float(t[2])


def node(*args):
    r = subprocess.run(["node", str(ROOT / "trailer/render.mjs"), *map(str, args)], cwd=ROOT, env=node_env(), capture_output=True, text=True, timeout=300)
    if r.returncode:
        raise RuntimeError(r.stderr[-600:])


def build(edl, d, c):
    d.mkdir(parents=True, exist_ok=True)
    (d / "edl.json").write_text(json.dumps(edl, indent=1))
    parts, voices, t = [], [], 0.0
    for k, s in enumerate(edl["segments"]):
        sd = d / f"{k:02d}-{s['id']}"
        sd.mkdir(exist_ok=True)
        voice = None
        if s.get("line"):
            voice = speak(c, s["line"], sd / "voice.wav")
        secs = max(float(s["seconds"]), (duration(voice) + 0.5) if voice else 0)
        cap = sd / "caption.png"
        node("caption", cap, s.get("caption") or " ")
        raw = sd / "raw.mp4"
        if s["kind"] == "card":
            sub = "Built today with Gemini 4 Argon and Managed Agents. github.com/carrollco/did-it-learn. Voice: Gemini TTS. Music: Lyria." if s["id"] == "end" else "A drone, two AI build loops, and one that cheated."
            node("card", sd / "card.png", s["title"], sub)
            ff("-loop", 1, "-i", sd / "card.png", "-t", secs, "-r", 25, "-pix_fmt", "yuv420p", "-c:v", "libx264", raw)
        elif s["kind"] == "page":
            node("page", sd / "page.webm", PAGE, s.get("heading", "-"), secs + 1.5)
            ff("-ss", 1.4, "-i", sd / "page.webm", "-t", secs, "-r", 25, "-vf", "scale=1280:720", "-pix_fmt", "yuv420p", "-c:v", "libx264", raw)
        else:
            src = FOOT / s["who"] / f"replay_seed{SEED}.webm"
            L = duration(src)
            speed = max(1.0, L / secs)
            ff("-i", src, "-vf", f"setpts=PTS/{speed:.3f},tpad=stop_mode=clone:stop_duration={secs},scale=1280:720", "-t", secs,
               "-r", 25, "-pix_fmt", "yuv420p", "-c:v", "libx264", raw)
        seg = sd / "seg.mp4"
        ff("-i", raw, "-i", cap, "-filter_complex", "[0:v][1:v]overlay=0:0", "-t", secs, "-r", 25, "-pix_fmt", "yuv420p", "-c:v", "libx264", seg)
        parts.append(seg)
        if voice:
            voices.append((voice, t + 0.25))
        t += secs
    (d / "concat.txt").write_text("".join(f"file '{p}'\n" for p in parts))
    ff("-f", "concat", "-safe", 0, "-i", d / "concat.txt", "-c", "copy", d / "video.mp4")
    mus = d / "music.mp3"
    if not mus.exists():
        music(c, edl.get("music") or "Instrumental only. Over-the-top action trailer, drums and brass, a comic pause.", mus)
    inputs, filt = ["-i", d / "video.mp4", "-stream_loop", -1, "-i", mus], []
    for i, (v, at) in enumerate(voices):
        inputs += ["-i", v]
        filt.append(f"[{i + 2}:a]adelay={int(at * 1000)}|{int(at * 1000)},volume=1.6[v{i}]")
    filt.append(f"[1:a]volume=0.22,atrim=0:{t:.2f}[m]")
    filt.append("".join(f"[v{i}]" for i in range(len(voices))) + f"[m]amix=inputs={len(voices) + 1}:normalize=0[a]")
    ff(*inputs, "-filter_complex", ";".join(filt), "-map", "0:v", "-map", "[a]", "-c:v", "copy", "-c:a", "aac", "-b:a", "160k",
       "-t", f"{t:.2f}", d / "trailer.mp4")
    return d / "trailer.mp4", t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", type=int, required=True)
    ap.add_argument("--edl")
    a = ap.parse_args()
    c = client()
    d = OUT / f"v{a.version}"
    d.mkdir(parents=True, exist_ok=True)
    edl = json.loads(pathlib.Path(a.edl).read_text()) if a.edl else write_words(c, d)
    path, t = build(edl, d, c)
    print(json.dumps({"trailer": str(path.relative_to(ROOT)), "seconds": round(t, 1)}))


if __name__ == "__main__":
    main()
