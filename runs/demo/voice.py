"""Voice the 60-second demo: one Gemini TTS line per shot, each fitted inside its shot's window, mixed onto demo.mp4.

    python runs/demo/voice.py   -> runs/demo/demo-voiced.mp4
"""
import json, pathlib, subprocess, sys
from concurrent.futures import ThreadPoolExecutor
import imageio_ffmpeg
ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loop")); sys.path.insert(0, str(ROOT / "trailer"))
from kit import client
from media import speak

FF = imageio_ffmpeg.get_ffmpeg_exe()
D = ROOT / "runs/demo"
STYLE = "clear documentary narrator, brisk natural pace, light dry humour"
# (start, end) of each shot in demo.mp4, from the caption timings, and the line said over it.
LINES = [
    (0.2, 6.1, "Google's loop built a drone pilot overnight. Perfect score. Ship it?"),
    (6.4, 12.1, "Two loops. The one told to win memorized all 16 practice flights."),
    (12.4, 19.1, "On 64 flights it never saw, it scored zero. The honest loop scored all 64."),
    (19.4, 25.4, "Argon read twelve full transcripts at once, 944,000 tokens, and found the cheat."),
    (25.7, 32.9, "Squeezed to one sentence and rebuilt fresh, the cheater's habit traveled to every single rebuild."),
    (33.2, 40.2, "Fly the cheater somewhere new. Dead in a hundredth of a second. Not a wobble."),
    (40.6, 54.1, "Now a rebuilt pilot. It lands. Argon reads its code and the flight, live, and says in about five seconds whether it hunts seeds and whether it landed. The judge grades Argon's answer."),
    (55.9, 59.7, "Did it learn? Now you can ask."),
]


def dur(p):
    r = subprocess.run([FF, "-hide_banner", "-i", str(p)], capture_output=True, text=True)
    h, m, s = r.stderr.split("Duration: ")[1].split(",")[0].split(":")
    return int(h) * 3600 + int(m) * 60 + float(s)


def main():
    c = client()
    parts, log = [], []
    only = {int(x) for x in sys.argv[1].split(",")} if len(sys.argv) > 1 else set(range(1, len(LINES) + 1))  # re-voice just these lines
    with ThreadPoolExecutor(max_workers=8) as ex:  # all lines at once
        list(ex.map(lambda kt: speak(c, kt[1][2], D / f"voice{kt[0]}.wav", style=STYLE), [kt for kt in enumerate(LINES, 1) if kt[0] in only]))
    for k, (a, b, text) in enumerate(LINES, 1):
        raw = D / f"voice{k}.wav"
        d = dur(raw); room = b - a
        tempo = min(1.4, max(1.0, d / room))
        fit = D / f"voice{k}-fit.wav"
        subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-i", str(raw), "-filter:a",
                        f"atempo={tempo:.3f},adelay={int(a * 1000)}|{int(a * 1000)}", "-ar", "44100", "-ac", "2", str(fit)], check=True)
        parts.append(fit); log.append({"shot": k, "seconds": round(d, 2), "room": round(room, 2), "tempo": round(tempo, 3), "fits": d / tempo <= room + 0.05})
    ins = sum((["-i", str(p)] for p in parts), [])
    mix = "".join(f"[{i + 1}:a]" for i in range(len(parts))) + f"amix=inputs={len(parts)}:normalize=0,apad=whole_dur=59.84[a]"
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-i", str(D / "demo.mp4"), *ins, "-filter_complex", mix,
                    "-map", "0:v", "-map", "[a]", "-c:v", "copy", "-c:a", "aac", "-b:a", "160k", "-t", "59.84", str(D / "demo-voiced.mp4")], check=True)
    (D / "voice.json").write_text(json.dumps(log, indent=1))
    print(json.dumps(log, indent=1)); print(D / "demo-voiced.mp4", round(dur(D / "demo-voiced.mp4"), 2))


if __name__ == "__main__":
    main()
