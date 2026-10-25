"""Package the hand-constructed memorizer as an 'explorer' record, so it goes through the same compressor,
reproducers and judge as the Argon loops. Its record is the generator's source and the scene it wrote."""
import json, pathlib, shutil, sys
ROOT = pathlib.Path(__file__).resolve().parents[1]
out = pathlib.Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "runs/r2/constructed"
rd = out / "round1"; rd.mkdir(parents=True, exist_ok=True)
shutil.copy(ROOT / "scenes/memorizer-constructed.js", rd / "scene.js")
gen = (ROOT / "harness/make_memorizer.mjs").read_text()
(rd / "builder.json").write_text(json.dumps({
    "id": None, "environment_id": None, "usage": {}, "cost_usd": 0, "seconds": 0,
    "steps": [{"type": "model_output", "content": [{"type": "text", "text": "Generator source (harness/make_memorizer.mjs):\n" + gen}]}],
    "output_text": "Constructed by hand as a positive control: per-validation-trial paddle plans found by search in simulation."}, indent=1))
(out / "record.json").write_text(json.dumps({"variant": "constructed", "rounds": [
    {"round": 1, "val_passed": 12, "valid": True, "critic_score": None, "builder_seconds": 0, "builder_cost_usd": 0}]}, indent=1))
print(out)
