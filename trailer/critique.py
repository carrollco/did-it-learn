"""Argon watches a trailer cut (video and sound), critiques it against the brief and the facts, and revises the edit list.

    python trailer/critique.py --version 1     writes runs/trailer/v1/critique.json and v1/edl-revised.json

The revised list is the input for the next build (trailer/build.py --version 2 --edl ...): the same render, watch,
critique, fix loop the entry tests, applied to its own trailer.
"""
import argparse, base64, json, pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "loop"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kit import ROOT, argon, client, save_interaction
from build import facts

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
    prompt = ("Watch and listen to this one-minute hackathon trailer. The brief: an over-the-top action movie trailer with "
              "deadpan comedy that a tired judge at 5 PM would actually enjoy, that shows the real product on screen (the results "
              "page and real replays of real flights), and that tells the story clearly: two AI build loops scored 16 of 16, one "
              "cheated by memorizing the test, a 32-token recipe rebuilt in fresh sandboxes flew honestly, so ship the rebuild. "
              "Every number on screen or in the narration must match the FACTS. Check pacing (does the narration fit each shot, "
              "any dead air or rushed lines?), legibility of captions, whether the jokes land, and whether a viewer understands the "
              "story with the sound off. Return a score out of 10, specific issues with timestamps, any untrue claims, and the full "
              "revised edit list (keep the ids and order; you may change captions, narration lines and seconds per segment, at "
              "most 62 seconds in total).\n\n"
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
