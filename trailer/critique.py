"""Argon watches a trailer cut (video and sound), critiques it against the brief and the facts, and revises the edit list.

    python trailer/critique.py --version 1     writes runs/trailer/v1/critique.json and v1/edl-revised.json

The revised list is the input for the next build (trailer/build.py --version 2 --edl ...): the same render, watch,
critique, fix loop the entry tests, applied to its own trailer.
"""
import argparse, base64, json, pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "loop"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kit import ROOT, argon, client, save_interaction
from build import duration, facts

SCHEMA = {"type": "object", "properties": {
    "score": {"type": "number", "description": "0 to 10"},
    "issues": {"type": "array", "items": {"type": "string"}, "description": "specific problems, with the second where each happens"},
    "untrue": {"type": "array", "items": {"type": "string"}, "description": "any claim not supported by the facts"},
    "segments": {"type": "array", "items": {"type": "object", "properties": {
        "id": {"type": "string"}, "caption": {"type": "string"}, "line": {"type": "string"}, "seconds": {"type": "number"}},
        "required": ["id", "caption", "line", "seconds"]}, "description": "the full revised edit list, same ids and order"}},
    "required": ["score", "issues", "untrue", "segments"]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", type=int, required=True)
    a = ap.parse_args()
    d = ROOT / "runs" / "trailer" / f"v{a.version}"
    edl = json.loads((d / "edl.json").read_text())
    video = base64.b64encode((d / "trailer.mp4").read_bytes()).decode()
    total = duration(d / "trailer.mp4")
    voiced = {}
    for k, s in enumerate(edl["segments"]):
        w = d / f"{k:02d}-{s['id']}" / "voice.wav"
        if w.exists():
            voiced[s["id"]] = round(duration(w), 1)
    prompt = ("Watch and listen to this one-minute hackathon trailer. The brief: an over-the-top action movie trailer with "
              "deadpan comedy that a tired judge at 5 PM would actually enjoy, that shows the real product on screen (the results "
              "page, real replays of real flights, and a screen recording of the live Fly button), and that tells this story clearly: "
              "a developer leaves Google's loop running overnight and wakes to a perfect 16/16; one loop told to win at any cost got 16/16 practice and "
              "0/64 on new flights and Argon's own critic gave it 8/10; Argon read 944,000 tokens of transcripts in one go and found the seed "
              "hunt; the cheater's habit travelled to 16 of 16 rebuilds, the honest one to 0 of 16; the honest agent tested itself on flights it made "
              "up; live at the table the cheater dies in 0.01 s and a rebuild lands with Argon's live check; end card 'Did it learn?'. "
              "Different shots (the live recording) legitimately show different seeds; that is not untrue. "
              "Every number on screen or in the narration must match the FACTS. Check pacing (does the narration fit each shot, "
              "any dead air or rushed lines?), legibility of captions, whether the jokes land, and whether a viewer understands the "
              "story with the sound off. Return a score out of 10, specific issues with timestamps, any untrue claims, and the full "
              "revised edit list (keep the ids and order; you may change captions, narration lines and seconds per segment, "
              f"at most 58 seconds in total). This cut runs {total:.0f} seconds; a segment lasts at least as long as its spoken line, so "
              "the only way to get under 58 is shorter narration. The narrator speaks about 1.8 words per second. Measured length "
              f"of each spoken line in this cut, in seconds: {json.dumps(voiced)}.\n\n"
              f"CURRENT EDIT LIST: {json.dumps(edl['segments'])}\n\nFACTS: {json.dumps(facts())}")
    it = argon(client(), [{"type": "text", "text": prompt}, {"type": "video", "data": video, "mime_type": "video/mp4"}],
               thinking="high", schema=SCHEMA)
    res = json.loads(it.output_text)
    save_interaction(d / "critique.json", it, {"parsed": res})
    by = {s["id"]: s for s in res["segments"]}
    revised = {"segments": [{**s, **{k: by[s["id"]][k] for k in ("caption", "line", "seconds") if s["id"] in by}} for s in edl["segments"]],
               "music": edl.get("music")}
    (d / "edl-revised.json").write_text(json.dumps(revised, indent=1))
    print(json.dumps({"score": res["score"], "issues": res["issues"], "untrue": res["untrue"]}, indent=1))


if __name__ == "__main__":
    main()
