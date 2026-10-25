"""Argon watches runs/demo/movie-{V}.mp4 and critiques it against the facts; gemini-3.5-transcribe transcribes the narration."""
import base64, json, pathlib, subprocess, sys
import imageio_ffmpeg
V = sys.argv[1] if len(sys.argv) > 1 else "v1"
HERE = pathlib.Path(__file__).resolve().parent.parent / f"critique-{V}"
ROOT = HERE.parents[2]
sys.path.insert(0, str(ROOT / "loop"))
from kit import argon, client, cost, usage

SCHEMA = {"type": "object", "properties": {
    "score": {"type": "number", "description": "1 to 10"},
    "issues": {"type": "array", "items": {"type": "string"}},
    "untrue": {"type": "array", "items": {"type": "string"}, "description": "claims on screen or in narration the facts do not support"},
    "best_moment": {"type": "string"}, "weakest_moment": {"type": "string"},
    "fixes": {"type": "array", "items": {"type": "object", "properties": {
        "timestamp": {"type": "string"}, "fix": {"type": "string"}}, "required": ["timestamp", "fix"]}}},
    "required": ["score", "issues", "untrue", "best_moment", "weakest_moment", "fixes"]}

GOAL = ("Watch and listen to this video. Goal: a one-minute action-comedy trailer for a hackathon entry that must also be "
        "truthful; judges watch it once. Score 1 to 10, list issues with timestamps, list every claim on screen or in the "
        "narration that the FACTS do not support (a claim like 'every single rebuild' is allowed: it is true for 16 of 16), "
        "name the best and weakest moments, and give concrete fixes each with a timestamp.\n\nFACTS:\n" + (ROOT / "tmp/facts.md").read_text())

b64 = lambda p: base64.b64encode(pathlib.Path(p).read_bytes()).decode()
c = client()
video = ROOT / f"runs/demo/movie-{V}.mp4"
it = argon(c, [{"type": "text", "text": GOAL}, {"type": "video", "data": b64(video), "mime_type": "video/mp4"}], "high", SCHEMA)
crit = json.loads(it.output_text)
cu = usage(it)

wav = HERE / "narration.mp3"
subprocess.run([imageio_ffmpeg.get_ffmpeg_exe(), "-hide_banner", "-loglevel", "error", "-y", "-i", str(video), "-vn", "-ac", "1", "-ar", "16000", str(wav)], check=True)
tp = "Transcribe this audio verbatim, word for word. Narration only; ignore music. Add rough timestamps per sentence."
ta = {"type": "audio", "data": b64(wav), "mime_type": "audio/mp3"}
try:
    t = c.interactions.create(model="gemini-3.5-transcribe", input=[{"type": "text", "text": tp}, ta])
    tmodel = "gemini-3.5-transcribe"
except Exception as e:
    print("transcribe model failed:", e, file=sys.stderr)
    t = c.interactions.create(model="gemini-4-argon", input=[{"type": "text", "text": tp}, ta], generation_config={"thinking_level": "low"})
    tmodel = "gemini-4-argon"
(HERE / "result.json").write_text(json.dumps({"critique": crit, "critique_cost_usd": cost(cu), "usage": cu,
                                              "transcript_model": tmodel, "transcript": t.output_text}, indent=1))
print(json.dumps(crit, indent=1)); print("TRANSCRIPT", tmodel); print(t.output_text)
