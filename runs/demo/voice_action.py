"""Candidate A2: same 60-second screen recording, action-trailer narrator and a Lyria music bed. Writes demo-action.mp4."""
import json, pathlib, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from voice import D, FF, LINES, dur, speak, client

STYLE = "deep movie-trailer narrator, deadpan comic timing, punchy, short pauses, never rushed"
MUSIC = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else None


def main():
    c = client()
    with ThreadPoolExecutor(max_workers=8) as ex:
        list(ex.map(lambda kt: speak(c, kt[1][2], D / f"voiceA{kt[0]}.wav", style=STYLE), enumerate(LINES, 1)))
    parts, log = [], []
    for k, (a, b, text) in enumerate(LINES, 1):
        raw = D / f"voiceA{k}.wav"; d = dur(raw); room = b - a
        tempo = min(1.4, max(1.0, d / room)); fit = D / f"voiceA{k}-fit.wav"
        subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-i", str(raw), "-filter:a",
                        f"atempo={tempo:.3f},adelay={int(a * 1000)}|{int(a * 1000)}", "-ar", "44100", "-ac", "2", str(fit)], check=True)
        parts.append(fit); log.append({"shot": k, "seconds": round(d, 2), "room": round(room, 2), "tempo": round(tempo, 3), "fits": d / tempo <= room + 0.05})
    ins = sum((["-i", str(p)] for p in parts), [])
    n = len(parts)
    voice = "".join(f"[{i + 1}:a]" for i in range(n)) + f"amix=inputs={n}:normalize=0[v]"
    if MUSIC:
        ins += ["-stream_loop", "-1", "-i", str(MUSIC)]
        mix = voice + f";[{n + 1}:a]volume=0.16,afade=t=out:st=55.8:d=4[m];[v][m]amix=inputs=2:normalize=0,apad=whole_dur=59.84[a]"
    else:
        mix = voice + ";[v]apad=whole_dur=59.84[a]"
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-i", str(D / "demo.mp4"), *ins, "-filter_complex", mix,
                    "-map", "0:v", "-map", "[a]", "-c:v", "copy", "-c:a", "aac", "-b:a", "160k", "-t", "59.84", str(D / "demo-action.mp4")], check=True)
    (D / "voice-action.json").write_text(json.dumps(log, indent=1))
    print(json.dumps(log)); print(D / "demo-action.mp4", round(dur(D / "demo-action.mp4"), 2))


if __name__ == "__main__":
    main()
