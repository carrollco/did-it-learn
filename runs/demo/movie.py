"""The movie cut: generated drone footage (Gemini Omni Flash) for the story, the real live flights and Argon's live check
for the proof, the narration lines already voiced, Argon's Lyria music, and big number cards.

    python runs/demo/movie.py runs/demo/movie-v1.json
The JSON lists shots in order: {"src": path, "start": s, "len": s, "card": "text or empty", "line": narration index or 0,
"ambient": 0.0 to 1.0}. Generated shots carry their own sound (ambient); recorded page shots are silent.
"""
import json, pathlib, subprocess, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from voice import D, FF, dur

ROOT = D.parents[1]
MUSIC = ROOT / "runs/trailer/v14/music.mp3"
PW = "/home/cagst/projects/argon-hackathon/harness/node_modules/playwright/index.mjs"


def card_png(text, out):
    """A big, bold number card: white text with a dark drop shadow, lower third."""
    js = f"""import {{ chromium }} from '{PW}';
const b = await chromium.launch(); const p = await b.newPage({{ viewport: {{ width: 1280, height: 720 }} }});
await p.setContent(`<style>body{{margin:0;background:transparent}}.c{{position:absolute;left:0;right:0;bottom:70px;text-align:center;
font:900 62px/1.1 Arial,Helvetica,sans-serif;color:#fff;letter-spacing:1px;text-shadow:0 3px 14px rgba(0,0,0,.85),0 0 2px #000}}
.c small{{display:block;font:700 28px/1.3 Arial,Helvetica,sans-serif;margin-top:8px;color:#ffe9a8}}</style>
<div class=c>{text}</div>`);
await p.screenshot({{ path: '{out}', omitBackground: true }}); await b.close();"""
    s = out.with_suffix(".mjs"); s.write_text(js)
    subprocess.run(["node", str(s)], check=True, env={**__import__("os").environ, "LD_LIBRARY_PATH": str(pathlib.Path.home() / ".cache/ms-playwright-local-libs/root/usr/lib/x86_64-linux-gnu")})


def main():
    plan = json.loads(pathlib.Path(sys.argv[1]).read_text())
    name = pathlib.Path(sys.argv[1]).stem
    work = D / "coldopen" / name; work.mkdir(parents=True, exist_ok=True)
    segs, t, narr, amb = [], 0.0, [], []
    for k, sh in enumerate(plan["shots"]):
        src = ROOT / sh["src"]; ln = float(sh["len"]); st = float(sh.get("start", 0))
        seg = work / f"seg{k:02d}.mp4"
        vf = f"scale=1280:720:force_original_aspect_ratio=decrease,pad=1280:720:(ow-iw)/2:(oh-ih)/2,fps=25,setsar=1,format=yuv420p"
        if sh.get("hold"):  # freeze the last frame to stretch a shot
            vf += f",tpad=stop_mode=clone:stop_duration={float(sh['hold']):.2f}"
        cmd = [FF, "-hide_banner", "-loglevel", "error", "-y", "-ss", f"{st:.2f}", "-t", f"{ln:.2f}", "-i", str(src)]
        if sh.get("card"):
            png = work / f"card{k:02d}.png"; card_png(sh["card"], png)
            cmd += ["-i", str(png), "-filter_complex", f"[0:v]{vf}[v0];[v0][1:v]overlay=0:0:enable='gte(t,{sh.get('card_at', 0.4)})'[v]", "-map", "[v]"]
        else:
            cmd += ["-vf", vf]
        cmd += ["-an", "-c:v", "libx264", "-preset", "fast", "-crf", "19", str(seg)]
        subprocess.run(cmd, check=True)
        if sh.get("ambient"):
            amb.append((src, st, ln, t, float(sh["ambient"])))
        if sh.get("line"):
            narr.append((D / f"voice{sh['line']}.wav", t + float(sh.get("line_at", 0.3))))
        segs.append(seg); t += ln
    total = t
    (work / "list.txt").write_text("".join(f"file '{s}'\n" for s in segs))
    video = work / "video.mp4"
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(work / "list.txt"), "-c", "copy", str(video)], check=True)
    # audio: narration + each generated shot's own sound at its place + music bed, faded at the end
    ins, fl, labels = [], [], []
    for i, (w, at) in enumerate(narr):
        ins += ["-i", str(w)]; fl.append(f"[{i + 1}:a]aformat=channel_layouts=stereo,aresample=44100,adelay={int(at * 1000)}|{int(at * 1000)}[n{i}]"); labels.append(f"[n{i}]")
    base = len(narr)
    for j, (src, st, ln, at, vol) in enumerate(amb):
        ins += ["-ss", f"{st:.2f}", "-t", f"{ln:.2f}", "-i", str(src)]
        fl.append(f"[{base + j + 1}:a]volume={vol},aformat=channel_layouts=stereo,aresample=44100,afade=t=out:st={max(0, ln - 0.6):.2f}:d=0.6,adelay={int(at * 1000)}|{int(at * 1000)}[a{j}]"); labels.append(f"[a{j}]")
    mi = base + len(amb) + 1
    ins += ["-stream_loop", "-1", "-i", str(MUSIC)]
    fl.append(f"[{mi}:a]volume={plan.get('music', 0.16)},afade=t=out:st={total - 4:.2f}:d=4[m]"); labels.append("[m]")
    fl.append("".join(labels) + f"amix=inputs={len(labels)}:normalize=0,apad=whole_dur={total:.2f}[a]")
    out = D / f"{name}.mp4"
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-i", str(video), *ins, "-filter_complex", ";".join(fl),
                    "-map", "0:v", "-map", "[a]", "-c:v", "copy", "-c:a", "aac", "-b:a", "192k", "-t", f"{total:.2f}", str(out)], check=True)
    print(out, round(dur(out), 2), "s", "shots", len(segs))


if __name__ == "__main__":
    main()
