"""Cold open: an 8-second generated drone shot (Gemini Omni Flash) replaces the title shot of the 60-second recording.

    python runs/demo/coldopen.py runs/demo/coldopen/night.mp4   -> runs/demo/demo-coldopen.mp4
Audio: line 1 of the narration over the clip's own sound (wind, rotors) plus the Lyria bed; then the recording from shot 2.
"""
import pathlib, subprocess, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from voice import D, FF, dur

CLIP = pathlib.Path(sys.argv[1]).resolve()
MUSIC = D.parent / "trailer/v14/music.mp3"
CUT = 6.4          # the recording continues from shot 2
OUT = D / "demo-coldopen.mp4"


def main():
    clen = dur(CLIP)
    # 1) cold open: clip video (scaled, 25 fps) + its own sound (quiet) + narration line 1 + music bed
    cold = D / "coldopen" / "cold.mp4"
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-i", str(CLIP), "-i", str(D / "voice1.wav"), "-stream_loop", "-1", "-i", str(MUSIC),
                    "-filter_complex",
                    "[0:v]scale=1280:720:force_original_aspect_ratio=decrease,pad=1280:720:(ow-iw)/2:(oh-ih)/2,fps=25,setsar=1,format=yuv420p[v];"
                    "[0:a]volume=0.5[c];[1:a]adelay=600|600,aformat=channel_layouts=stereo[n];[2:a]volume=0.16[m];"
                    f"[c][n][m]amix=inputs=3:normalize=0,apad=whole_dur={clen:.2f}[a]",
                    "-map", "[v]", "-map", "[a]", "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-c:a", "aac", "-b:a", "160k", "-ar", "44100",
                    "-t", f"{clen:.2f}", str(cold)], check=True)
    # 2) the rest of the recording, from shot 2 (re-encoded so the cut is clean)
    rest = D / "coldopen" / "rest.mp4"
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-ss", str(CUT), "-i", str(D / "demo-music.mp4"),
                    "-vf", "fps=25,setsar=1", "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-c:a", "aac", "-b:a", "160k", "-ar", "44100", str(rest)], check=True)
    lst = D / "coldopen" / "list.txt"
    lst.write_text(f"file '{cold}'\nfile '{rest}'\n")
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(lst), "-c", "copy", str(OUT)], check=True)
    print(OUT, round(dur(OUT), 2), "s")


if __name__ == "__main__":
    main()
