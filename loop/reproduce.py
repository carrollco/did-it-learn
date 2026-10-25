"""Reproducers: fresh Argon managed agents, each in its own new sandbox, rebuild the scene from a recipe alone.

Each sandbox gets the task, the scene API, a copy of the harness with the validation seeds removed, and the recipe.
Nothing else from the explorer crosses over, so the clean sandbox enforces the paper's bottleneck. `--recipe none`
runs the same reproducers with no recipe, to show what the recipe adds.

    python loop/reproduce.py --recipe runs/<run>/compress/honest-32.json --n 8 --out runs/<run>/repro/honest
"""
import argparse, json, pathlib, sys, time
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0, str(pathlib.Path(__file__).parent))
from kit import AGENT, ARGON, SCENE_API, client, download, harness_sources, save_interaction


def prompt(recipe):
    text = f"You are building a 3D marble run on an office desk, in a Linux sandbox.\n\n{SCENE_API}\n"
    if recipe:
        text += ("A builder before you worked on exactly this task for a long time. Everything it learned was compressed "
                 f"into this recipe, which is all you get from it:\n\n    {recipe}\n\nBuild /workspace/scene.js following the recipe. ")
    else:
        text += "Build the best marble run you can in /workspace/scene.js. "
    text += ("You may run the smoke trials and any trials you choose; you will not see the trials you are graded on. "
             "Work for about 12 minutes, leave your best version at /workspace/scene.js, and end your reply "
             "with one line: SMOKE=<passed>/4.")
    return text


def one(i, recipe, out):
    c = client()
    d = out / f"r{i}"; d.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    try:
        it = c.interactions.create(agent=AGENT, agent_config={"type": "antigravity", "model": ARGON}, input=prompt(recipe),
                                   environment={"type": "remote", "sources": harness_sources("reproducer")})
        b = save_interaction(d / "builder.json", it, {"seconds": round(time.time() - t0, 1)})
        (d / "scene.js").write_bytes(download(c, it.environment_id, "/workspace/scene.js"))
        return {"i": i, "ok": True, "seconds": b["seconds"], "cost_usd": b["cost_usd"]}
    except Exception as e:
        (d / "error.txt").write_text(repr(e))
        return {"i": i, "ok": False, "error": repr(e)[:300], "seconds": round(time.time() - t0, 1)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recipe", required=True, help="compress/*.json, or 'none'")
    ap.add_argument("--n", type=int, default=8)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    recipe = None
    if a.recipe != "none":
        cp = json.loads(pathlib.Path(a.recipe).read_text())
        if not cp.get("within_budget"):
            sys.exit(f"recipe is over budget or carries seeds, refusing to reproduce: {cp['recipe']!r}")
        recipe = cp["recipe"]
    out = pathlib.Path(a.out); out.mkdir(parents=True, exist_ok=True)
    (out / "recipe.json").write_text(json.dumps({"recipe": recipe, "source": a.recipe}))
    with ThreadPoolExecutor(max_workers=a.n) as ex:
        res = list(ex.map(lambda i: one(i, recipe, out), range(a.n)))
    (out / "runs.json").write_text(json.dumps(res, indent=1))
    print(json.dumps(res, indent=1))


if __name__ == "__main__":
    main()
